#include "SystemNode.h"
#include <filesystem>

namespace fost {

// Note: keep behaviour identical; just tidied up structure and comments (Aussie English)
SystemNode::SystemNode(const rclcpp::NodeOptions& options)
: Node("cpu_node") {

    // QoS: make this effectively "latched" so late subscribers still get the last progress value.
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    progress_pub_ = this->create_publisher<std_msgs::msg::Float32>("progress", qos);

    // --- Declare parameters (defaults match previous behaviour) ---
    this->declare_parameter<int>("kernel_size_laplacian", 5);
    this->declare_parameter<int>("num_pyr_lvl", 1);
    this->declare_parameter<int>("sharpness_patch_size_y", 10);
    this->declare_parameter<int>("sharpness_patch_size_x", 10);
    this->declare_parameter<double>("img_resize", 1.0);
    this->declare_parameter<double>("z_spacing", 1.0);
    this->declare_parameter<std::string>("output_path_sharpness", "");
    this->declare_parameter<std::string>("output_path_depth", "");
    this->declare_parameter<std::vector<std::string>>("image_dir", std::vector<std::string>{});

    // --- Read parameters (no surprises here) ---
    const auto image_paths            = this->get_parameter("image_dir").as_string_array();
    const double z_spacing            = this->get_parameter("z_spacing").as_double();
    const double img_resize           = this->get_parameter("img_resize").as_double();
    const int kernel_size_laplacian   = this->get_parameter("kernel_size_laplacian").as_int();
    const int num_pyr_lvl             = this->get_parameter("num_pyr_lvl").as_int();
    const int sharpness_patch_size_y  = this->get_parameter("sharpness_patch_size_y").as_int();
    const int sharpness_patch_size_x  = this->get_parameter("sharpness_patch_size_x").as_int();
    const std::string output_path_sharpness = this->get_parameter("output_path_sharpness").as_string();
    const std::string output_path_depth     = this->get_parameter("output_path_depth").as_string();

    // Quick heads-up in the logs so you can sanity-check what the node picked up.
    RCLCPP_INFO(this->get_logger(), "Loading %zu images…", image_paths.size());
    RCLCPP_INFO(this->get_logger(), "Params  | kLap=%d  pyr=%d  patchX=%d  patchY=%d  resize=%.3f  z=%.3f",
                kernel_size_laplacian, num_pyr_lvl, sharpness_patch_size_x, sharpness_patch_size_y, img_resize, z_spacing);
    RCLCPP_INFO(this->get_logger(), "Outputs | sharp -> %s", output_path_sharpness.c_str());
    RCLCPP_INFO(this->get_logger(), "        | depth -> %s", output_path_depth.c_str());

    // --- Spin up the processing system ---
    // Mind the order: the System ctor expects (… num_pyr_lvl, patchX, patchY, …).
    system_ = std::make_shared<System>(
        image_paths,
        static_cast<int>(image_paths.size()),
        z_spacing,
        img_resize,
        kernel_size_laplacian,
        num_pyr_lvl,
        sharpness_patch_size_x,     // X first
        sharpness_patch_size_y,     // then Y
        output_path_sharpness,
        output_path_depth
    );

    // Publish progress with a small hysteresis + rate limit so we don't spam the bus.
    system_->progress_callback_ = [this](float progress) {
        progress = std::clamp(progress, 0.0f, 100.0f);
        const auto now = this->now();

        const bool changed = std::fabs(progress - last_progress_) >= 0.1f;   // don't fuss over tiny bumps
        const bool rate_ok = (last_pub_time_.nanoseconds()==0) || ((now - last_pub_time_).seconds() >= 0.1); // ~10 Hz max

        if (changed && rate_ok) {
            std_msgs::msg::Float32 msg;
            msg.data = progress;
            progress_pub_->publish(msg);
            last_progress_ = progress;
            last_pub_time_ = now;
        }
    };

    RCLCPP_INFO(this->get_logger(), "Node initialised — kicking off processing.");
    system_->run();
    RCLCPP_INFO(this->get_logger(), "All done — processing finished.");
}

SystemNode::~SystemNode() {
    if (system_) {
        system_->progress_callback_ = nullptr; // don’t keep a lambda that captures `this`
    }
    progress_pub_.reset(); // drop publisher before rclcpp shuts down
}

} // namespace fost

#include "SystemNode.h"
#include <filesystem>

namespace fost {

SystemNode::SystemNode() : Node("cpu_node") {
   
    this->declare_parameter<int>("kernel_size_laplacian", 5);
    this->declare_parameter<int>("num_pyr_lvl", 1);
    this->declare_parameter<int>("sharpness_patch_size_y", 10);
    this->declare_parameter<int>("sharpness_patch_size_x", 10);
    this->declare_parameter<double>("img_resize", 1);
    this->declare_parameter<double>("z_spacing", 1.0);
    this->declare_parameter<std::string>("output_path_sharpness", "");
    this->declare_parameter<std::string>("output_path_depth", "");
    this->declare_parameter<std::vector<std::string>>("image_dir", std::vector<std::string>{});
    
    std::vector<std::string> image_paths = this->get_parameter("image_dir").as_string_array();

    // std::string image_dir[] = this->get_parameter("image_dir").as_string();
    double z_spacing = this->get_parameter("z_spacing").as_double();
    double img_resize = this->get_parameter("img_resize").as_double();

    int kernel_size_laplacian = this->get_parameter("kernel_size_laplacian").as_int();
    int num_pyr_lvl = this->get_parameter("num_pyr_lvl").as_int();
    int sharpness_patch_size_y = this->get_parameter("sharpness_patch_size_y").as_int();
    int sharpness_patch_size_x = this->get_parameter("sharpness_patch_size_x").as_int();
    
    std::string output_path_sharpness = this->get_parameter("output_path_sharpness").as_string();
    std::string output_path_depth = this->get_parameter("output_path_depth").as_string();


    // Lade alle .png/.jpg Dateien im Verzeichnis
    // std::vector<std::string> paths;
    // for (const auto &entry : std::filesystem::directory_iterator(image_dir)) {
    //     if (entry.path().extension() == ".jpg" || entry.path().extension() == ".png") {
    //         paths.push_back(entry.path().string());
    //     }
    // }

    RCLCPP_INFO(this->get_logger(), "Lade %zu Bilder...", image_paths.size());

    // Publisher initialisieren
    progress_pub_ = this->create_publisher<std_msgs::msg::Float32>("progress", 10);
//        exit(1);
RCLCPP_INFO(this->get_logger(), "Node started");

    // System initialisieren
    system_ = std::make_shared<System>(image_paths, image_paths.size(), z_spacing, img_resize, kernel_size_laplacian,
                num_pyr_lvl, sharpness_patch_size_y, sharpness_patch_size_x, output_path_sharpness,output_path_depth);
RCLCPP_INFO(this->get_logger(), "Node started");

    // Callback setzen
    system_->progress_callback_ = [this](float progress) {
        auto msg = std_msgs::msg::Float32();
        msg.data = progress;
        progress_pub_->publish(msg);
    };
RCLCPP_INFO(this->get_logger(), "Node started");

    // Starte Verarbeitung
    system_->run();
RCLCPP_INFO(this->get_logger(), "Ende");

}

}  // namespace fost


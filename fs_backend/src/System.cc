#include "System.h"
#include <string>
#include <filesystem>
#include <algorithm>   // std::clamp, std::max
#include <cmath>       // std::round, std::log2, std::floor

namespace fost {

System::System(std::vector<std::string> path_to_imgs,
               int num_images,
               double z_spacing,
               double img_resize,
               int kernel_size_laplacian,
               int num_pyr_lvl,
               int sharpness_patch_size_x,
               int sharpness_patch_size_y,
               std::string output_path_sharpness,
               std::string output_path_depth)
: Node("cpu_node_tmp"),
  path_to_imgs_(std::move(path_to_imgs)),
  num_images_(num_images),
  num_pyramid_(num_pyr_lvl),
  z_spacing_(z_spacing),
  resize_factor(img_resize),
  kernel_size_laplacian_(kernel_size_laplacian),
  sharpness_patch_size_x_(sharpness_patch_size_x),
  sharpness_patch_size_y_(sharpness_patch_size_y),
  output_path_sharpness_(std::move(output_path_sharpness)),
  output_path_depth_(std::move(output_path_depth)) {

    // Need at least one input, mate.
    if (path_to_imgs_.empty()) {
        RCLCPP_FATAL(this->get_logger(), "No input images provided.");
        throw std::runtime_error("System: empty image list");
    }

    // Establish working resolution from first image.
    cv::Mat tmp_img = cv::imread(path_to_imgs_[0], cv::IMREAD_GRAYSCALE);
    if (tmp_img.empty()) {
        RCLCPP_FATAL(this->get_logger(), "Failed to read first image: %s", path_to_imgs_[0].c_str());
        throw std::runtime_error("System: failed to read first image");
    }

    // Keep resize positive; never let dims hit zero.
    resize_factor = std::max(1e-6, resize_factor);
    height_ = std::max(1, static_cast<int>(std::round(tmp_img.rows * resize_factor)));
    width_  = std::max(1, static_cast<int>(std::round(tmp_img.cols  * resize_factor)));

    // Laplacian kernel: odd, >=1, <=31 (OpenCV restriction).
    if (kernel_size_laplacian_ <= 0) kernel_size_laplacian_ = 1;
    if ((kernel_size_laplacian_ % 2) == 0) ++kernel_size_laplacian_;
    if (kernel_size_laplacian_ > 31) {
        RCLCPP_WARN(this->get_logger(),
                    "kernel_size_laplacian_=%d too large; clamping to 31.",
                    kernel_size_laplacian_);
        kernel_size_laplacian_ = 31;
    }

    // Cap pyramid depth by available resolution (pyrDown needs >=2x2).
    const int max_levels_by_size =
        1 + static_cast<int>(std::floor(std::log2(std::max(1, std::min(width_, height_)))));
    num_pyramid_ = std::clamp(num_pyramid_, 1, max_levels_by_size);

    // Working buffers.
    global_sharpness_img_.create(height_, width_, CV_64F);
    index_img_.create(height_, width_, CV_32SC1);
    global_sharpness_img_.setTo(0);
    index_img_.setTo(0);

    // Pre-allocate local sharpness maps using ceil division to match pyrDown sizes.
    local_sharpness_img_.resize(num_pyramid_);
    for (int lvl = 0; lvl < num_pyramid_; ++lvl) {
        const int scale = 1 << lvl;  // 1,2,4,…
        const int h_lvl = (height_ + scale - 1) / scale; // ceil(height/scale)
        const int w_lvl = (width_  + scale - 1) / scale; // ceil(width/scale)
        local_sharpness_img_[lvl].create(h_lvl, w_lvl, CV_64F);
        local_sharpness_img_[lvl].setTo(0);
    }

    // Keep OpenCV single-threaded to avoid nested threading shenanigans with OpenMP.
    cv::setNumThreads(1);

    RCLCPP_INFO(this->get_logger(), "num_pyramid_: %d", num_pyramid_);
    RCLCPP_INFO(this->get_logger(), "z_spacing_: %.3f", z_spacing_);
    RCLCPP_INFO(this->get_logger(), "resize_factor: %.3f", resize_factor);
    RCLCPP_INFO(this->get_logger(), "kernel_size_laplacian_: %d", kernel_size_laplacian_);
    RCLCPP_INFO(this->get_logger(), "sharpness_patch_size_x_: %d", sharpness_patch_size_x_);
    RCLCPP_INFO(this->get_logger(), "sharpness_patch_size_y_: %d", sharpness_patch_size_y_);
    RCLCPP_INFO(this->get_logger(), "output_path_sharpness_: %s", output_path_sharpness_.c_str());
    RCLCPP_INFO(this->get_logger(), "output_path_depth_: %s", output_path_depth_.c_str());

    // Tune to your box if you like.
    omp_set_num_threads(12);
}

// Build a Gaussian pyramid of 'num_pyramid_' levels.
void System::pyDown(cv::Mat& img, std::vector<cv::Mat>& pyramid) {
    pyramid.resize(num_pyramid_);
    pyramid[0] = img.clone();
    for (int i = 1; i < num_pyramid_; ++i) {
        pyrDown(pyramid[i - 1], pyramid[i]); // shrink each level by ~2 (ceil halving)
    }
}

// For each pyramid level, compute local variance of Laplacian in a patch,
// then fuse levels into a full-res sharpness map and update the global max/index.
void System::coarse_to_fine(std::vector<cv::Mat>& pyramid, int img_id) {

    // Make sure our level buffers exactly match the real pyramid sizes (ceil-halving aware).
    for (int lvl = 0; lvl < num_pyramid_; ++lvl) {
        const cv::Mat& lvlImg = pyramid[lvl];
        if (local_sharpness_img_[lvl].rows != lvlImg.rows ||
            local_sharpness_img_[lvl].cols != lvlImg.cols ||
            local_sharpness_img_[lvl].type() != CV_64F) {
            local_sharpness_img_[lvl].create(lvlImg.rows, lvlImg.cols, CV_64F);
        }
        local_sharpness_img_[lvl].setTo(0);
    }

    // 1) Per-level Laplacian + variance over patches
    for (int lvl = 0; lvl < num_pyramid_; ++lvl) {
        cv::Mat lap;
        calc_laplacian(pyramid[lvl], lap, kernel_size_laplacian_);

        const int W = lap.cols;
        const int H = lap.rows;

        #pragma omp parallel for collapse(2)
        for (int x = 0; x < W; ++x) {
            for (int y = 0; y < H; ++y) {
                const int px = sharpness_patch_size_x_;
                const int py = sharpness_patch_size_y_;
                const int hx = px / 2;
                const int hy = py / 2;

                const int x_min = std::max(0, x - hx);
                const int x_max = std::min(x + hx + 1, W);
                const int y_min = std::max(0, y - hy);
                const int y_max = std::min(y + hy + 1, H);

                const cv::Rect roi(x_min, y_min, x_max - x_min, y_max - y_min);

                cv::Scalar mean, stddev;
                cv::meanStdDev(lap(roi), mean, stddev);
                const double variance = stddev[0] * stddev[0];

                local_sharpness_img_[lvl].at<double>(y, x) += variance;
            }
        }
    }

    // 2) Fuse levels into a full-res temporary sharpness image
    cv::Mat tmp_sharpness_img(height_, width_, CV_64F, cv::Scalar(0));
    for (int lvl = 0; lvl < num_pyramid_; ++lvl) {
        const int scale = 1 << lvl;
        const cv::Mat& src = local_sharpness_img_[lvl];

        const int W = src.cols;
        const int H = src.rows;

        #pragma omp parallel for collapse(2)
        for (int x = 0; x < W; ++x) {
            for (int y = 0; y < H; ++y) {
                const double v = src.at<double>(y, x);
                const int xn = x * scale;
                const int yn = y * scale;
                tmp_sharpness_img.at<double>(yn, xn) += v;
            }
        }
    }

    // 3) Update global maxima and index image
    #pragma omp parallel for collapse(2)
    for (int x = 0; x < width_; ++x) {
        for (int y = 0; y < height_; ++y) {
            const double prev = global_sharpness_img_.at<double>(y, x);
            const double now  = tmp_sharpness_img.at<double>(y, x);
            if (now > prev) {
                global_sharpness_img_.at<double>(y, x) = now;
                index_img_.at<int>(y, x) = img_id;
            }
        }
    }
}

// Compose the sharp image by picking the pixel from the image that "won" at each location.
void System::create_sharp_img() {
    sharp_img.create(height_, width_, CV_8U);

    for (int img_id = 0; img_id < static_cast<int>(path_to_imgs_.size()); ++img_id) {
        cv::Mat img = cv::imread(path_to_imgs_[img_id], cv::IMREAD_GRAYSCALE);
        if (img.empty()) continue;

        const int h = std::max(1, static_cast<int>(std::round(img.rows * resize_factor)));
        const int w = std::max(1, static_cast<int>(std::round(img.cols  * resize_factor)));
        cv::resize(img, img, cv::Size(w, h), 0, 0, cv::INTER_LINEAR);

        #pragma omp parallel for collapse(2)
        for (int x = 0; x < width_; ++x) {
            for (int y = 0; y < height_; ++y) {
                if (index_img_.at<int>(y, x) == img_id) {
                    sharp_img.at<uchar>(y, x) = img.at<uchar>(y, x);
                }
            }
        }
    }

    cv::imwrite(output_path_sharpness_, sharp_img);
}

// Turn the winning index into real depth values, save a normalised jpg and a float EXR sidecar.
void System::create_depth_map() {
    depth_img_.create(height_, width_, CV_64F);

    #pragma omp parallel for collapse(2)
    for (int x = 0; x < width_; ++x) {
        for (int y = 0; y < height_; ++y) {
            depth_img_.at<double>(y, x) = static_cast<double>(index_img_.at<int>(y, x)) * z_spacing_;
        }
    }

    // Keep a copy with real depth values for EXR
    cv::Mat depth_val = depth_img_.clone();

    // Normalised 8-bit preview (jpg)
    cv::Mat image8U;
    cv::normalize(depth_img_, depth_img_, 0, 255, cv::NORM_MINMAX);
    depth_img_.convertTo(image8U, CV_8U);
    RCLCPP_INFO(this->get_logger(), "depth_img_ empty? %s", depth_img_.empty() ? "yes" : "no");
    cv::imwrite(output_path_depth_, image8U);

    // EXR next to the jpg (same basename)
    std::filesystem::path exr_path = output_path_depth_;
    exr_path.replace_extension(".exr");
    depth_val.convertTo(depth_val, CV_32F);
    RCLCPP_INFO(this->get_logger(), "exr_path? %s", exr_path.string().c_str());
    cv::imwrite(exr_path.string(), depth_val);
}

// Thin wrapper – keeps it flexible if we swap operators later.
void System::calc_laplacian(cv::Mat img, cv::Mat& lap, int kernel_size) {
    cv::Laplacian(img, lap, CV_64F, kernel_size);
}

void System::run() {
    std::cout << "Start Focus Stacking" << std::endl;
    RCLCPP_INFO(this->get_logger(), "Processing started");

    std::vector<cv::Mat> pyramid;

    for (int img_id = 0; img_id < static_cast<int>(path_to_imgs_.size()); ++img_id) {
        if (progress_callback_) {
            const float progress = static_cast<float>(img_id + 1)
                                 / static_cast<float>(path_to_imgs_.size()) * 100.0f;
            progress_callback_(progress);
        }

        cv::Mat img = cv::imread(path_to_imgs_[img_id], cv::IMREAD_GRAYSCALE);
        if (img.empty()) {
            RCLCPP_WARN(this->get_logger(), "Skipping unreadable image: %s", path_to_imgs_[img_id].c_str());
            continue;
        }

        const int h = std::max(1, static_cast<int>(std::round(img.rows * resize_factor)));
        const int w = std::max(1, static_cast<int>(std::round(img.cols  * resize_factor)));
        cv::resize(img, img, cv::Size(w, h), 0, 0, cv::INTER_LINEAR);

        // Build pyramid and process
        pyDown(img, pyramid);
        coarse_to_fine(pyramid, img_id);
    }

    RCLCPP_INFO(this->get_logger(), "Creating depth map…");
    create_depth_map();

    RCLCPP_INFO(this->get_logger(), "Compositing sharp image…");
    create_sharp_img();

    std::cout << "done" << std::endl;
    RCLCPP_INFO(this->get_logger(), "Processing finished");
}

} // namespace fost

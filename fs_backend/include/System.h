#pragma once

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>   // cv::Mat etc.
#include <functional>
#include <string>
#include <vector>
#include <omp.h>


namespace fost {

/**
 * @brief Core focus-stacking system.
 *        Runs inside a ROS 2 node, does the heavy lifting and reports progress via callback.
 */
class System : public rclcpp::Node {
public:
    /**
     * @brief Construct the processing system.
     * @param path_to_imgs              Full paths to input images (ordered).
     * @param num_images                Number of images (same as path_to_imgs.size()).
     * @param z_spacing                 Physical Z step between images.
     * @param img_resize                Global resize factor (>0). Fractional values allowed.
     * @param kernel_size_laplacian     Laplacian kernel size (odd, >=1).
     * @param num_pyr_lvl               Number of pyramid levels (>=1).
     * @param sharpness_patch_size_x    Patch size in X (pixels).
     * @param sharpness_patch_size_y    Patch size in Y (pixels).
     * @param output_path_sharpness     Output path for the sharp (composited) image.
     * @param output_path_depth         Output path for the depth map (jpg + .exr sidecar).
     */
    System(std::vector<std::string> path_to_imgs,
           int num_images,
           double z_spacing,
           double img_resize,
           int kernel_size_laplacian,
           int num_pyr_lvl,
           int sharpness_patch_size_x,
           int sharpness_patch_size_y,
           std::string output_path_sharpness,
           std::string output_path_depth);

    /// Kick off the full processing pipeline (blocking call).
    void run();

    /// Progress callback (0..100). We’ll call this from the worker whenever progress updates.
    std::function<void(float)> progress_callback_;

private:
    // --- Internal steps (kept private – callers shouldn’t worry about these) ---
    void pyDown(cv::Mat& img, std::vector<cv::Mat>& pyramid);
    void coarse_to_fine(std::vector<cv::Mat>& pyramid, int img_id);
    void calc_laplacian(cv::Mat img, cv::Mat& lap, int kernel_size);
    void create_depth_map();
    void create_sharp_img();

    // --- Inputs & configuration ---
    std::vector<std::string> path_to_imgs_;
    int     num_images_              = 0;
    double  z_spacing_               = 7.6;
    int     width_                   = 0;   // working width (after resize)
    int     height_                  = 0;   // working height (after resize)
    int     num_pyramid_             = 4;

    double  resize_factor            = 1.0; // NOTE: was int; keep as double so img_resize<1 works properly
    int     kernel_size_laplacian_   = 5;
    int     sharpness_patch_size_x_  = 5;
    int     sharpness_patch_size_y_  = 5;

    std::string output_path_sharpness_ = "";
    std::string output_path_depth_     = "";

    // --- Working buffers (images) ---
    cv::Mat              global_sharpness_img_;
    std::vector<cv::Mat> local_sharpness_img_;
    cv::Mat              index_img_;
    cv::Mat              depth_img_;
    cv::Mat              sharp_img; // kept name to match current .cpp
};

} // namespace fost

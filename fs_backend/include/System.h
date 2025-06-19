#ifndef SYSTEM_H
#define SYSTEM_H


#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <omp.h> 

namespace fost {

class System : public rclcpp::Node{
    public:

        System(std::vector<std::string> path_to_imgs, int num_images, double z_spacing, double img_resize, int kernel_size_laplacian, int num_pyr_lvl, int sharpness_patch_size_x, int sharpness_patch_size_y, std::string output_path_sharpness, std::string output_path_depth);

        void run();
        
        std::function<void(float)> progress_callback_;  // NEW
        
        
    private:
        void pyDown(cv::Mat &img, std::vector<cv::Mat> &pyramid);
        void coarse_to_fine(std::vector<cv::Mat> &pyramid, int img_id);
        void calc_laplacian(cv::Mat img, cv::Mat &lap, int kernel_size);

        void create_depth_map();
        void create_sharp_img();

        std::vector<std::string> path_to_imgs_;
        int num_images_ = 0;
        double z_spacing_ = 7.6;
        int width_ = 0;
        int height_ = 0;
        int num_pyramid_ = 4;

        int resize_factor = 1;
        int kernel_size_laplacian_ = 5;
        int sharpness_patch_size_x_ = 5;
        int sharpness_patch_size_y_ = 5;

        std::string output_path_sharpness_ = "";
        std::string output_path_depth_ = "";



        cv::Mat global_sharpness_img_;
        std::vector<cv::Mat> local_sharpness_img_;
        cv::Mat index_img_;
        cv::Mat depth_img_;
        cv::Mat sharp_img;

        // std::vector<cv::Mat> var_img_pyramid_, aux_img_pyramid2_;
        // cv::Mat aux_index_img_;
        // cv::Mat aux_var_img_;
        // cv::Mat depth_img_;
        // cv::Mat focused_img_;
        // cv::Mat indexed_img
        

};
}
#endif

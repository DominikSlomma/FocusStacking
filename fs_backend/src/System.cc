#include "System.h"

#define  TEST std::cout << "Test\n";

namespace fost {
    System::System(std::vector<std::string> path_to_imgs, int num_images, double z_spacing, double img_resize, int kernel_size_laplacian, int num_pyr_lvl, int sharpness_patch_size_x, int sharpness_patch_size_y, std::string output_path_sharpness, std::string output_path_depth)
    : Node("cpu_node_tmp"), path_to_imgs_(path_to_imgs), num_images_(num_images),num_pyramid_(num_pyr_lvl), z_spacing_(z_spacing), resize_factor(img_resize), kernel_size_laplacian_(kernel_size_laplacian), sharpness_patch_size_x_(sharpness_patch_size_x), sharpness_patch_size_y_(sharpness_patch_size_y), output_path_sharpness_(output_path_sharpness), output_path_depth_(output_path_depth)  {
        cv::Mat tmp_img = cv::imread(path_to_imgs_[0]);
        height_ = static_cast<int>(tmp_img.rows * resize_factor);
        width_ = static_cast<int>(tmp_img.cols * resize_factor);
        global_sharpness_img_.create(height_, width_, CV_64F);
        index_img_.create(height_, width_, CV_32SC1);

        global_sharpness_img_.setTo(0);
        index_img_.setTo(0);

        local_sharpness_img_.resize(num_pyramid_);

        for (int id=0; id < num_pyramid_; id++) {
            int scale = 1 << id;
            local_sharpness_img_[id].create(height_/scale, width_/scale, CV_64F); 
            local_sharpness_img_[id].setTo(0);
        }

        
        // Do I need here more?
RCLCPP_INFO(this->get_logger(), "num_pyramid_: %d", num_pyramid_);
RCLCPP_INFO(this->get_logger(), "z_spacing_: %f", z_spacing_);
RCLCPP_INFO(this->get_logger(), "resize_factor: %f", resize_factor);
RCLCPP_INFO(this->get_logger(), "kernel_size_laplacian_: %d", kernel_size_laplacian_);
RCLCPP_INFO(this->get_logger(), "sharpness_patch_size_x_: %d", sharpness_patch_size_x_);
RCLCPP_INFO(this->get_logger(), "sharpness_patch_size_y_: %d", sharpness_patch_size_y_);
RCLCPP_INFO(this->get_logger(), "output_path_sharpness_: %s", output_path_sharpness_.c_str());
RCLCPP_INFO(this->get_logger(), "output_path_depth_: %s", output_path_depth_.c_str());



        omp_set_num_threads(24);
    }

    void System::pyDown(cv::Mat &img, std::vector<cv::Mat> &pyramid) {
        pyramid.resize(num_pyramid_);
        pyramid[0] = img.clone();

        for (int i = 1; i < num_pyramid_; i++) {
            pyrDown(pyramid[i - 1], pyramid[i]); // Bild verkleinern
        }
    }

    void System::coarse_to_fine(std::vector<cv::Mat> &pyramid, int img_id) {

        // init area
        cv::Mat lap;

        for (int idx_pyr_lvl = 0; idx_pyr_lvl < num_pyramid_; idx_pyr_lvl++) {

            local_sharpness_img_[idx_pyr_lvl].setTo(0);

            cv::Mat img = pyramid[idx_pyr_lvl];
            calc_laplacian(img, lap, kernel_size_laplacian_);

            int width = lap.cols;
            int height = lap.rows;
RCLCPP_INFO(this->get_logger(), "vla");

            #pragma omp parallel for collapse(2)
            for(int x = 0; x < width; x++) {
                for(int y = 0; y < height; y++) {

                    // Größe des Patches
                    int patch_size_x = sharpness_patch_size_x_;
                    int patch_size_y = sharpness_patch_size_y_;

                    // Berechne die Grenzen des Patches
                    int half_patch_x = patch_size_x / 2;  // 5x5 Patch, also 2 Pixel nach oben, unten, links und rechts
                    int half_patch_y= patch_size_x / 2;  // 5x5 Patch, also 2 Pixel nach oben, unten, links und rechts
                    int x_min = std::max(0, x - half_patch_x);
                    int x_max = std::min( x + half_patch_x + 1, lap.cols);
                    int y_min = std::max(0, y - half_patch_y);
                    int y_max = std::min(y + half_patch_y + 1, lap.rows);

                    cv::Mat patch = lap(cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min));

                    cv::Scalar mean, stddev;
                    cv::meanStdDev(patch, mean, stddev);
                    double variance = stddev[0] * stddev[0];

                    int w = (2*idx_pyr_lvl) == 0 ? 1 : (2*idx_pyr_lvl);
                    local_sharpness_img_[idx_pyr_lvl].at<double>(y*w,x*w) += variance;
                }
            }
            
        }
        
RCLCPP_INFO(this->get_logger(), "vla");
    
        cv::Mat tmp_sharpness_img = cv::Mat::zeros(height_, width_, CV_64F);
        for(int id=0; id < num_pyramid_; id++) {
            int scale = 1 << id;
            cv::Mat tmp = local_sharpness_img_[id];
            int width = tmp.cols;
            int height = tmp.rows;

            double value = 0;
            #pragma omp parallel for collapse(2)
            for(int x = 0; x < width; x++) {
                for(int y = 0; y < height; y++) {
                    value = tmp.at<double>(y,x);
                    
                    int xn = x * scale;
                    int yn = y * scale;
                    tmp_sharpness_img.at<double>(yn,xn) += 1 * value;
                    // std::exp(-static_cast<double>(id));
                }
            }
        }
RCLCPP_INFO(this->get_logger(), "vla");

        std::cout << "Set values" << std::endl;
        #pragma omp parallel for collapse(2)
        for(int x = 0; x < width_; x++) {
            for(int y = 0; y < height_; y++) {
                double arg1 = global_sharpness_img_.at<double>(y,x);
                double arg2 = tmp_sharpness_img.at<double>(y,x);

                if(arg1 < arg2) {
                    global_sharpness_img_.at<double>(y,x) = tmp_sharpness_img.at<double>(y,x);
                    index_img_.at<int>(y,x) = img_id;
                }

            }
        }
    }

    void System::create_sharp_img() {

        sharp_img.create(height_, width_, CV_8U);
        for(int img_id=0; img_id< path_to_imgs_.size(); img_id++) {
            cv::Mat img = cv::imread(path_to_imgs_[img_id], cv::IMREAD_GRAYSCALE);
            int height = static_cast<int>(img.rows * resize_factor);
            int width = static_cast<int>(img.cols * resize_factor);
            std::cout << img.size() << std::endl;
            cv::resize(img, img, cv::Size(width, height),0,0, cv::INTER_LINEAR);
            std::cout << img.size() << std::endl;
            
           

            #pragma omp parallel for collapse(2)
            for(int x = 0; x < width_; x++) {
                for(int y = 0; y < height_; y++) {
                    int id = index_img_.at<int>(y,x);

                    if (id == img_id)
                        sharp_img.at<uchar>(y,x) = img.at<uchar>(y,x);
                }
            }
        }
        cv::imwrite(output_path_sharpness_, sharp_img);
    }

    void System::create_depth_map() {
        depth_img_.create(height_, width_, CV_64F);
        #pragma omp parallel for collapse(2)
        for(int x = 0; x < width_; x++) {
            for(int y = 0; y < height_; y++) {
                depth_img_.at<double>(y,x) = index_img_.at<int>(y,x) * z_spacing_;
            }
        }


        std::cout << depth_img_.size() << std::endl;
        std::cout << index_img_.size() << std::endl;
        cv::Mat image8U;
        cv::normalize(depth_img_, depth_img_, 0, 255, cv::NORM_MINMAX);
        depth_img_.convertTo(image8U, CV_8U);
        cv::imwrite(output_path_depth_, image8U);
    }

    void System::calc_laplacian(cv::Mat img, cv::Mat &lap, int kernel_size) {
        cv::Laplacian(img, lap, CV_64F, kernel_size);
    }

    void System::run() {
        std::cout << "Start Focus Stacking" << std::endl;
RCLCPP_INFO(this->get_logger(), "s2222a started");


        ///////////////////////////////////////////////////////////
        //              Initialisation area                      //
        ///////////////////////////////////////////////////////////

        std::vector<cv::Mat> tmp_img_pyramid;
        
        
        std::vector<cv::Mat> pyramid;
        cv::Mat tmp_sharpness_img = cv::Mat::zeros(width_ , height_, CV_64F);

        for(int img_id=0; img_id< path_to_imgs_.size(); img_id++) {
            float progress = static_cast<float>(img_id + 1)/static_cast<float>(path_to_imgs_.size()) * 100.0f;
            // Callback aufrufen
            if (progress_callback_) {
                progress_callback_(progress);
            }
RCLCPP_INFO(this->get_logger(), "asdasdassa started");

            cv::Mat img = cv::imread(path_to_imgs_[img_id], cv::IMREAD_GRAYSCALE);
            int height = static_cast<int>(img.rows * resize_factor);
            int width = static_cast<int>(img.cols * resize_factor);
            
RCLCPP_INFO(this->get_logger(), "num_pyramid_: %d", img.rows);
RCLCPP_INFO(this->get_logger(), "num_pyramid_: %d", img.cols);

            cv::resize(img, img, cv::Size(width,height), 0,0, cv::INTER_LINEAR);


RCLCPP_INFO(this->get_logger(), "num_pyramid_: %d", img.rows);
RCLCPP_INFO(this->get_logger(), "num_pyramid_: %d", img.cols);
            // Create a pyramid for an image
            pyDown(img, pyramid);
RCLCPP_INFO(this->get_logger(), "asdasdassa started");
            
            // Process every image in the pyramid
            coarse_to_fine(pyramid, img_id);
RCLCPP_INFO(this->get_logger(), "asdasdassa started");

            RCLCPP_INFO(this->get_logger(), "sa started");

        }
RCLCPP_INFO(this->get_logger(), "Nosasasde started");

          //Compute Depthmap
        create_depth_map();
RCLCPP_INFO(this->get_logger(), "Nosasasde started");

        // create sharpness image
        create_sharp_img();
        

    }
}

// namespace fost {
//     System::System(std::vector<std::string> path_to_imgs, int num_images, double z_spacing) : path_to_imgs_(path_to_imgs), num_images_(num_images), z_spacing_(z_spacing) {
//         cv::Mat tmp_img = cv::imread(path_to_imgs_[0]);
        
//         // cv::Size size(tmp_img.rows, tmp_img.cols);
//         // cv::resize(tmp_img, tmp_img, size);
        
//         height_ = tmp_img.rows;
//         width_ = tmp_img.cols;

//         // std::cout << height_ << " " << width_ << std::endl;
//         // std::cout << tmp_img.size().height << " " << tmp_img.size().width << std::endl;
//         // exit(1);

//         // int width = lap.cols;
//         // int height = lap.rows;

//         // cv::Mat empty = cv::Mat::zeros(height_, width_, CV_64F);
//         // aux_var_img_ = cv::Mat::zeros(height_, width_, CV_64F);
//         //depth_img_ = cv::Mat::zeros(height_, width_, CV_64F);
//         // focused_img_ = tmp_img;
//         // aux_index_img_ = cv::Mat::zeros(height_, width_, CV_32SC1);
//         // int h = 1;
//         // int w = 1;
//         // for (int i=0; i < num_pyramid_;i++) {
//         //     cv::Mat empty = cv::Mat::zeros((int)(height_ / h), (int)(width_ / w), CV_64F);
//         //     var_img_pyramid_.push_back(empty);
//         //     h = h * 2;
//         //     w = w * 2;
//         // }        
//     }

//     void System::run() {
        
//         std::cout << "Initialise\n";
//         cv::Mat index_img = cv::Mat::zeros(width_ , height_, CV_32SC1);
//         cv::Mat sharpness_img = cv::Mat::zeros(width_ , height_, CV_64F);

//         std::cout << "Start processing!\n";

//         for(int idx=0; idx < path_to_imgs_.size(); idx++) {

            
//             std::vector<cv::Mat> tmp_img_pyramid;
//             cv::Mat tmp_sharpness_img = cv::Mat::zeros(width_ , height_, CV_64F);
//             // read the actual image
//             cv::Mat img = cv::imread(path_to_imgs_[idx], cv::IMREAD_GRAYSCALE);
            
//             // std::cout << img.size() << " " << index_img.size() << " " <<
//             // sharpness_img.size() << " " << tmp_sharpness_img.size() << " "  << std::endl;
            
//             // for debugging, faster processing!
//             // cv::Size size(img.rows / 4, img.cols / 4);
//             // cv::resize(img, img, size);
//             // cv::resize(index_img, index_img, size);
//             // cv::resize(sharpness_img, sharpness_img, size);
//             // cv::resize(tmp_sharpness_img, tmp_sharpness_img, size);

//             // std::cout << " dassdas " << img.size() << std::endl;

//             // cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
//             // cv::Mat lap;
//             // cv::Laplacian(img, lap, CV_64F,5);
//             // tmp_img_pyramid.push_back(lap);

//             std::vector<cv::Mat> pyramid(num_pyramid_);
//             pyramid[0] = img.clone();

//             for (int i = 1; i < num_pyramid_; i++) {
//                 pyrDown(pyramid[i - 1], pyramid[i]); // Bild verkleinern
//             }
            
//             // cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
//             // cv::Mat lap;
//             // cv::Laplacian(img, lap, CV_64F,31);

//             /*
//              *
//              * Calculate Sharpness
//              * 
//             */
//            std::cout << "Calculate Sharpness" << std::endl;
//             for (int idx_pyr_lvl = 0; idx_pyr_lvl < num_pyramid_; idx_pyr_lvl++) { 

//                 // cv::Laplacian(tmp_img, lap, CV_64F, 31);  // Example kernel size 31
//                 // tmp_img_pyramid.push_back(lap);
//                 std::cout  << "Calculating image " << idx  << "\tpyramid lvl: " << idx_pyr_lvl << std::endl; 
//                 cv::Mat tmp_img = pyramid[idx_pyr_lvl];
//                 cv::Mat lap;
//                 cv::Laplacian(tmp_img, lap, CV_64F, 31);   // Example kernel size 31
                
// //                 double minVal, maxVal;
// // cv::Point minLoc, maxLoc;
// // cv::minMaxLoc(lap, &minVal, &maxVal, &minLoc, &maxLoc);

// // std::cout << "Min Value: " << minVal << std::endl;
// // std::cout << "Max Value: " << maxVal << std::endl;
// // std::cout << "Min Location: " << minLoc << std::endl;
// // std::cout << "Max Location: " << maxLoc << std::endl;
// // std::cout << "width: " << lap.cols << std::endl;
// // std::cout << "height: " << lap.rows << std::endl;
// // cv::Mat test;
// // cv::Size size(lap.rows / 4, lap.cols / 4);
// //             cv::resize(lap, test, size);

//                 // cv::imshow("Laplacian Result", test);
//                 // cv::waitKey(0);  // Warte auf eine Taste, um das Fenster zu schließen
                
//                 int width = lap.cols;
//                 int height = lap.rows;

//                 #pragma omp parallel for collapse(2)
//                 for(int x = 0; x < width; x++) {
//                     for(int y = 0; y < height; y++) {

//                         // std::cout << width << std::endl;
//                         // std::cout << height << std::endl; 
//                         //exit(1);

//                         // Größe des Patches
//                         int patch_size = 31;

//                         // Berechne die Grenzen des Patches
//                         int half_patch = patch_size / 2;  // 5x5 Patch, also 2 Pixel nach oben, unten, links und rechts
//                         int x_min = std::max(0, x - half_patch);
//                         int x_max = std::min( x + half_patch + 1, lap.cols);
//                         int y_min = std::max(0, y - half_patch);
//                         int y_max = std::min(y + half_patch + 1, lap.rows);

//                         cv::Mat patch = lap(cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min));

//                         cv::Scalar mean, stddev;
//                         cv::meanStdDev(patch, mean, stddev);
//                         double variance = stddev[0] * stddev[0];

//                         int w = (2*idx_pyr_lvl) == 0 ? 1 : (2*idx_pyr_lvl);
//                         // std::cout << idx_pyr_lvl << " " << x << " " << y << " " << w << " "  << " " << x*w << " " << y*w  << " " << variance << std::endl;
//                         tmp_sharpness_img.at<double>(y*w,x*w) += variance;
//                         // if ( variance > sharpness_img.at<double>(y,x)) {
//                             // sharpness_img.at<double>(y,x) = variance;
//                             // index_img.at<int>(y,x) = idx;
//                             // std::cout << idx << std::endl;
//                         // }

//                     }
//                 }
//             }

//             std::cout << "Set values" << std::endl;
//             // #pragma omp parallel for collapse(2)
//             for(int x = 0; x < height_; x++) {
//                 for(int y = 0; y < width_; y++) {
//                     double arg1 = sharpness_img.at<double>(y,x);
//                     double arg2 = tmp_sharpness_img.at<double>(y,x);
                    
//                     if(arg1 < arg2) {
//                         // std::cout << x << " " << y << " ";
//                         sharpness_img.at<double>(y,x) = tmp_sharpness_img.at<double>(y,x);
//                         index_img.at<int>(y,x) = idx;
//                         // std::cout << sharpness_img.at<double>(y,x) << " " << index_img.at<int>(y,x) << std::endl;

//                     }

//                 }
//             }

            
//         }

//         cv::Mat depth_img = cv::Mat::zeros(width_, height_, CV_64F);
//         #pragma omp parallel for collapse(2)
//         for(int x = 0; x < width_; x++) {
//             for(int y = 0; y < height_; y++) {

//                 depth_img.at<double>(y,x) = index_img.at<int>(y,x) * 7.6;

//             }
//         }

//         cv::Mat image8U;
//         cv::normalize(depth_img, depth_img, 0, 255, cv::NORM_MINMAX);
//         depth_img.convertTo(image8U, CV_8U);
//         cv::imwrite("SystemTestNow_pyr4_new.jpg", image8U);

//         //     std::cout << " processing frame: " << idx << "\n";
//         //     omp_set_num_threads(24);

//         //     for (int idx_pyr_lvl = 0; idx_pyr_lvl < num_pyramid_; idx_pyr_lvl++) {

//         //         cv::Mat tmp_img = tmp_img_pyramid[idx_pyr_lvl];

//         //         int width = tmp_img.cols;
//         //         int height = tmp_img.rows;
                


//         //         #pragma omp parallel for collapse(2)
//         //         for(int x = 0; x < width; x++) {
//         //             for(int y = 0; y < height; y++) {

//         //                 // Größe des Patches
//         //                 int patch_size = 5;

//         //                 // Berechne die Grenzen des Patches
//         //                 int half_patch = patch_size / 2;  // 5x5 Patch, also 2 Pixel nach oben, unten, links und rechts
//         //                 int x_min = std::max(0, x - half_patch);
//         //                 int x_max = std::min(tmp_img.cols, x + half_patch + 1);
//         //                 int y_min = std::max(0, y - half_patch);
//         //                 int y_max = std::min(tmp_img.rows, y + half_patch + 1);

//         //                 cv::Mat patch = tmp_img(cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min));

//         //                 cv::Scalar mean, stddev;
//         //                 cv::meanStdDev(patch, mean, stddev);
//         //                 double variance = stddev[0] * stddev[0];

//         //                 // if ( variance > 10000)
//         //                 //     std::cout << vai+1)harpness) {
//         //                 //     tmp_img.at<double>(y,x) = sharpness;
//         //                 //     aux_index_img_.at<int>(y,x) = idx;
//         //                 //     // std::cout << idx << std::endl;
//         //                 // }

//         //             }
//         //         }
//         //         // var_img_pyramid_[0] = tmp_img;

//         //     }

//         //     cv::Mat tmp_img = aux_var_img_.clone();
//         //     int  bla = 0;
//         //     // #pragma omp parallel for collapse(2)
//         //     // std::cout << "test: " << idx << std::endl;
//         //     for (int x = 0; x < width_; x++) {
//         //         for (int y = 0; y < height_; y++) {
//         //             double sharpness = 0;
//         //             int sub_x; 
//         //             int sub_y;
//         //             for (int lvl_id=0; lvl_id < num_pyramid_; lvl_id++) {
                        

//         //                 if (lvl_id == 0) {
//         //                     sub_x = x;
//         //                     sub_y = y;
//         //                 } else {
//         //                     sub_x = sub_x / 2;
//         //                     sub_y = sub_y / 2;
//         //                 }

//         //                 sharpness += (double) var_img_pyramid_[lvl_id].at<double>(sub_y, sub_x);
//         //             }
                    
//         //             double actual_sharpness = (double)aux_var_img_.at<double>(y,x);

//         //             // if (actual_sharpness == 0) {
//         //             //     #pragma omg critical
//         //             //     if (bla==0) {
//         //             //         bla++;
//         //             //         std::cout << idx << " " << sharpness << std::endl;
//         //             //     }
//         //             // }
//         //             // if (idx >= 1) {

//         //             //         std::cout << sharpness << " " << actual_sharpness << " " << idx << std::endl;
//         //             // }
//         //             if(sharpness > actual_sharpness) {
//         //                 // if (idx == 1) {

//         //                 //     std::cout << idx << std::endl;
//         //                 // }
//         //                 // std::cout << sharpness << std::endl;
//         //                 aux_index_img_.at<int>(y,x) = idx;

//         //                 tmp_img.at<double>(y,x) = sharpness;
//         //             }
//         //         }
//         //     }
//         //     aux_var_img_ = tmp_img;
//         // }

//         // // create depth from index img
//         // #pragma omp parallel for collapse(2)
//         // for(int x = 0; x < width_; x++) {
//         //     for(int y = 0; y < height_; y++) { 
//         //         depth_img_.at<double>(y,x) = z_spacing_* aux_index_img_.at<int>(y,x);
//         //     }
//         // }
        

//         // for(int img_id=0; img_id < num_images_; img_id++) {
//         //     cv::Mat img = cv::imread(path_to_imgs_[img_id]);

//         //     #pragma omp parallel for collapse(2)
//         //     for(int x = 0; x < width_; x++) {
//         //         for(int y = 0; y < height_; y++) {

//         //             if (aux_index_img_.at<int>(y,x) == img_id) {
//         //                 focused_img_.at<cv::Vec3b>(y,x) = img.at<cv::Vec3b>(y,x);
//         //             }
//         //         }
//         //     }
//         // }


//         // for(int x = 0; x < width_; x++) {
//         //     for(int y = 0; y < height_; y++) {
//         //         if(aux_index_img_.at<int>(y,x) != 0) 
//         //             std::cout << aux_index_img_.at<int>(y,x) << "\n";
//         //     }
//         // }

//         // cv::Mat image8U;
//         // cv::normalize(depth_img_, depth_img_, 0, 255, cv::NORM_MINMAX);
//         // depth_img_.convertTo(image8U, CV_8U);
//         // cv::imwrite("System_test5_5_5.jpg", image8U);
//         // cv::imwrite("System_test_res5_5_5.jpg", focused_img_);
//         std::cout << "done"  << std::endl;


//     }
// }

#include <opencv2/opencv.hpp>

#include <filesystem>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <regex>
#include <omp.h> 

// Hilfsfunktion zum Extrahieren der Zahl aus dem Dateinamen
int extractNumber(const std::string& filePath) {
    std::regex re("(\\d+)");
    std::smatch match;
    if (std::regex_search(filePath, match, re)) {
        return std::stoi(match[1].str());
    }
    return 0; // Fallback, falls keine Zahl gefunden wird
}

cv::Mat compute_laplacian(const cv::Mat& image) {
    // Schritt 1: Bild in Graustufen konvertieren
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    
    // Schritt 2: Laplacian berechnen
    cv::Mat laplacian;
    cv::Laplacian(gray, laplacian, CV_64F);
    
    return laplacian;
}

void focus_stacking_and_depth_map(std::vector<std::string> imgPath, double z_spacing, cv::Mat &focus_stacked_image, cv::Mat& depth_map) {
    int num_z_planes = imgPath.size();

    cv::Mat tmp_img = cv::imread(imgPath[0]);

    int height = tmp_img.rows / 1;
    int width = tmp_img.cols / 1;




    depth_map = cv::Mat::zeros(height, width, CV_64F);

    std::vector<cv::Mat> laplacians;

    std::cout << "compute laplacian" << std::endl;
    for (int i=0; i<imgPath.size(); i++) {
        cv::Mat img = cv::imread(imgPath[i]);

        cv::Size size(height, width);
        cv::Mat resizedImg;

        // Bild auf die neue Größe skalieren
        cv::resize(img, img, size);

        cv::Mat lap = compute_laplacian(img);

        laplacians.push_back(lap);
    }

    std::cout << "Starting focus stacking and depth map creation" << std::endl;
    
    omp_set_num_threads(12);

    #pragma omp parallel for collapse(2)
    for (int x=0; x < width; x++) {
        for(int y=0; y < height; y++) {

            // int max_threads = omp_get_max_threads();
            
            //     std::cout << "Maximale Anzahl von Threads: " << max_threads << std::endl;
            
            int max_sharpness = -1;
            int best_focus_pixel = -1;
            int best_z = 0;

            for(int z=0; z < imgPath.size(); z++) {
                double sharpness = (double)laplacians[z].at<double>(y,x) * (double)laplacians[z].at<double>(y,x);
                if(sharpness > max_sharpness) {
                    max_sharpness = sharpness;                
                    best_z = z;
                    // std::cout << "in\n";
                }
            }

            depth_map.at<double>(y, x) = (best_z * z_spacing) / 91.2;
            // std::cout << depth_map.at<double>(y, x) << std::endl;
        }
    }

    // cv::Size size(500, 500);
    //     cv::Mat resizedImg;

    //     // Bild auf die neue Größe skalieren
    //     cv::resize(depth_map, resizedImg, size);

    // cv::imshow("test", resizedImg);
    // cv::waitKey(0);

    cv::Mat image8U;
        cv::normalize(depth_map, depth_map, 0, 255, cv::NORM_MINMAX);

    depth_map.convertTo(image8U, CV_8U);

    cv::imwrite("test.jpg", image8U);
}

int main() {
    std::string folderPath = "../data/test/";
    std::vector<std::string> imagePaths;  


    for (const auto& entry : std::filesystem::directory_iterator(folderPath)) {
        if (entry.is_regular_file()) {  
            std::string filePath = entry.path().string(); 
            imagePaths.push_back(filePath);  
        }
    }

    // Sortiere die Datei-Pfade anhand der extrahierten Zahl
    std::sort(imagePaths.begin(), imagePaths.end(), [](const std::string& a, const std::string& b) {
        return extractNumber(a) < extractNumber(b);
    });

    // // Gib die sortierten Pfade aus
    // for (const auto& filePath : imagePaths) {
    //     std::cout << filePath << std::endl;
    // }

    cv::Mat focus;
    cv::Mat depth;

    double z_spacing = 7.6;
    
    focus_stacking_and_depth_map(imagePaths,z_spacing, focus, depth);
    std::cout << "done" << std::endl;
    // cv::normalize(depth, depth, 0, 1, cv::NORM_MINMAX);
    //     depth.convertTo(depth, CV_8U, 255.0);

    cv::imshow("test", depth);
    cv::waitKey(0);
    // for(int i=0; i<size(imagePaths); i++){
    //     cv::Mat img = cv::imread(imagePaths[i]);
    //     std::cout << img.size() << " " << (int)img.at<uchar>(cv::Point(100,100)) << std::endl;
    //     // Zielgröße definieren: 500x500 Pixel
    //     cv::Size size(500, 500);
    //     cv::Mat resizedImg;

    //     // Bild auf die neue Größe skalieren
    //     cv::resize(img, resizedImg, size);
    //     std::cout << resizedImg.size() << " " << resizedImg.at<int>(cv::Point2d(100,100)) << std::endl;




    //     cv::imshow("img", resizedImg);
    //     cv::waitKey(0);
    // }
}
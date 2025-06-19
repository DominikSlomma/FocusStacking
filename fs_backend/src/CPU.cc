#include <opencv2/opencv.hpp>

#include <filesystem>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <regex>
#include <omp.h>


#include "System.h"



/*
// Hilfsfunktion zum Extrahieren der Zahl aus dem Dateinamen
int extractNumber(const std::string& filePath) {
    std::regex re("(\\d+)");
    std::smatch match;
    if (std::regex_search(filePath, match, re)) {
        return std::stoi(match[1].str());
    }
    return 0; // Fallback, falls keine Zahl gefunden wird
}


int main() {
    std::string folderPath = "../data/test/";
    std::vector<std::string> imagePaths;  
    int num_images = 0;
    double z_spacing = 7.6;

    for (const auto& entry : std::filesystem::directory_iterator(folderPath)) {
        if (entry.is_regular_file()) {  
            std::string filePath = entry.path().string(); 
            imagePaths.push_back(filePath);  
            num_images++;
        }
    }

    // Sortiere die Datei-Pfade anhand der extrahierten Zahl
    std::sort(imagePaths.begin(), imagePaths.end(), [](const std::string& a, const std::string& b) {
        return extractNumber(a) < extractNumber(b);
    });
    

    fost::System *sys = new fost::System(imagePaths, num_images, z_spacing);
    sys->run();
}*/

// src/CPU.cc
#include <rclcpp/rclcpp.hpp>
#include "SystemNode.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<fost::SystemNode>());
    auto node = std::make_shared<fost::SystemNode>();
    rclcpp::shutdown();
    return 0;
}


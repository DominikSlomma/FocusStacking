#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char** argv) {
    // Laden der Bilder
    cv::Mat img1 = cv::imread("../data/test/1_z-196.336.jpg", cv::IMREAD_GRAYSCALE);
    cv::Mat img2 = cv::imread("../data/test/13_z-103.764.jpg", cv::IMREAD_GRAYSCALE);
    if (img1.empty() || img2.empty()) {
        std::cerr << "Fehler beim Laden der Bilder." << std::endl;
        return -1;
    }

    // Schritt 1: Bilder um den Faktor 2 verkleinern
    cv::Mat img1_resized, img2_resized;
    cv::resize(img1, img1_resized, cv::Size(), 0.125, 0.125, cv::INTER_LINEAR);
    cv::resize(img2, img2_resized, cv::Size(), 0.125, 0.125, cv::INTER_LINEAR);

    // Schritt 2: Dense Optical Flow berechnen
    cv::Mat flow;
    cv::calcOpticalFlowFarneback(img1_resized, img2_resized, flow, 0.5, 3, 50, 3, 5, 1.2, 0);

    // Schritt 3: Visualisierung des Flusses
    cv::Mat flow_vis;
    cv::cvtColor(img1_resized, flow_vis, cv::COLOR_GRAY2BGR); // Konvertiere in BGR für die Visualisierung

    // Flussvektoren auf die Bildfläche zeichnen
    for (int y = 0; y < flow_vis.rows; y += 5) { // Beispielweise alle 5 Pixel
        for (int x = 0; x < flow_vis.cols; x += 5) {
            const cv::Point2f& fxy = flow.at<cv::Point2f>(y, x);
            cv::line(flow_vis, cv::Point(x, y), cv::Point(cvRound(x + fxy.x), cvRound(y + fxy.y)), cv::Scalar(0, 255, 0));
            cv::circle(flow_vis, cv::Point(x, y), 1, cv::Scalar(0, 0, 255), -1);
        }
    }

    // Ergebnisse anzeigen
    cv::imwrite("Optical Flow.jpg", flow_vis);
    cv::imwrite("Image 1 Resized.jpg", img1_resized);
    cv::imwrite("Image 2 Resized.jpg", img2_resized);

    cv::waitKey(0);
    return 0;
}

//
//  main.cpp
//  binary-mask-from-polygon
//
//  Created by Supannee Tanathong on 15/06/2024.
//

#include "algorithms.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <Eigen/Core>
#include <vector>
#include <iostream>
#include <numeric>

int CreateBinaryMaskFromPolygon(const size_t& maskWidth, const size_t& maskHeight,
                const unsigned char& maskColour, const unsigned char& maskBackgroundColour,
                const std::vector<Eigen::Vector2i>& polygon) {

    cv::Mat mask(maskHeight, maskWidth, CV_8UC1, cv::Scalar(maskBackgroundColour));
    if (mask.empty()) {
        std::cout << "Error! Failed to create a mask image." << std::endl;
        return -1;
    }
    
    // To reduce the processing time, only process within the bounding box of polygon
    size_t xmin = std::numeric_limits<size_t>::max();
    size_t ymin = std::numeric_limits<size_t>::max();
    size_t xmax = 0;
    size_t ymax = 0;
    
    for (auto vertice : polygon) {
        if (vertice.x() < 0 || vertice.x() >= maskWidth || vertice.y() < 0 || vertice.y() >= maskHeight) {
            std::cout << "Error! Vertice coordinate is not within the size of mask." << std::endl;
            return -1;
        }
        xmin = std::min<size_t>(xmin, vertice.x());
        ymin = std::min<size_t>(ymin, vertice.y());
        xmax = std::max<size_t>(xmax, vertice.x());
        ymax = std::max<size_t>(xmax, vertice.y());
    }
    
    for (size_t y = ymin; y <= ymax; ++y) {
        for (size_t x = xmin; x <= xmax; ++x) {
            if (IsPointInsidePolygon(polygon, x, y)) {
                mask.at<uchar>(y, x) = maskColour;
            }
        }
    }
        
    cv::imwrite("mask.png", mask);
    
    return 0;
}

int main(int argc, const char * argv[]) {
    // Adjust parameters
    size_t maskWidth = 720;
    size_t maskHeight = 1080;
    unsigned char maskColour = 255; // white mask
    unsigned char maskBackgroundColour = 0; // black background
    std::vector<Eigen::Vector2i> polygon {
        Eigen::Vector2i(20, 20),
        Eigen::Vector2i(100, 20),
        Eigen::Vector2i(100, 80),
        Eigen::Vector2i(20, 80),
    };
    
    // Process
    int success = CreateBinaryMaskFromPolygon(maskWidth, maskHeight, maskColour, maskBackgroundColour,
                                              polygon);
    if (success == 0) {
        std::cout << "Successfully created a binary mask image." << std::endl;
    } else {
        std::cout << "Failed to create a binary mask image." << std::endl;
    }
    
    return 0;
}

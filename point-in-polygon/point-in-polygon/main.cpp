//
//  main.cpp
//  point-in-polygon
//
//  Created by Supannee Tanathong on 15/06/2024.
//

#include <iostream>
#include <vector>
#include <Eigen/Core>

// https://www.eecs.umich.edu/courses/eecs380/HANDOUTS/PROJ2/InsidePoly.html
// Original code by Randolph Franklin, it returns 1 for interior points and 0 for exterior points.
// My modified version changes the code to be readable and
//      * if the point falls on edge, it is considered inside point.
bool IsPointInsidePolygon(std::vector<Eigen::Vector2i>& polygonPoints, int x, int y) {
    
    //std::cout << "Testing point (" << x << ", " << y << ")" << std::endl;
    
    int numPoints = polygonPoints.size();
    bool pointIsInside = false;
    
    // Iterate through all the edges forming the polygon
    for (int i = 0; i < polygonPoints.size(); ++i) {
        int j = (i + 1) % polygonPoints.size();
        // i, j are the two vertices forming an edge
        
        //std::cout << "\tEdge (" << polygonPoints[i].x() << ", " << polygonPoints[i].y() << ") -- (" << polygonPoints[j].x() << ", " << polygonPoints[j].y() << ")" << std::endl;
        
        // Check if the point is between the edge vertices
        if (std::min(polygonPoints[i].y(), polygonPoints[j].y()) <= y &&
            std::max(polygonPoints[i].y(), polygonPoints[j].y()) >= y) {
            // If the edge is horizontal and the point is on it, return true
            if (polygonPoints[i].y() == polygonPoints[j].y() && y == polygonPoints[i].y()) {
                if (std::min(polygonPoints[i].x(), polygonPoints[j].x()) <= x &&
                    std::max(polygonPoints[i].x(), polygonPoints[j].x()) >= x) {
                    return true; // Point is inside if it is on the horizontal edge
                }
            }
            // Check if the edge crosses a line from left to right [(0,y), (x,y)]
            //    m = (y2 - y1)/(x2 - x1)
            //    y - y1 = m (x - x1)
            //    x - x1 = (y - y1)/m
            //    x = (y - y1)/m + x1
            double oneOverSlope = double(polygonPoints[j].x() - polygonPoints[i].x())/(polygonPoints[j].y() - polygonPoints[i].y());
            double xIntersect = oneOverSlope * (y - polygonPoints[i].y()) + polygonPoints[i].x();
            if ((int)xIntersect == x) { // If x falls exactly at intersect point, x is considered inside.
                return true;
            }
            // xIntersect is the x' position (given passed-in y) that intersect the edge
            
            // Check if the line [(0,y), (x,y)] crosses the edge
            // If x is greater than x', it means the line already crosses the edge.
            if (xIntersect < x) {
                pointIsInside = !pointIsInside;
            }
        }
    }
    
    return pointIsInside;
}

int main(int argc, const char * argv[]) {
    // Test the algorithm
    std::vector<Eigen::Vector2i> polygonPoints {
        Eigen::Vector2i(0, 0),
        Eigen::Vector2i(10, 0),
        Eigen::Vector2i(10, 20),
        Eigen::Vector2i(0, 20)
    };
    
    std::vector<Eigen::Vector2i> insidePoint {
        Eigen::Vector2i(10, 10),
        Eigen::Vector2i(0, 0),
        Eigen::Vector2i(10, 0),
        Eigen::Vector2i(10, 20),
        Eigen::Vector2i(0, 20),
        Eigen::Vector2i(5, 0), // on horizontal edge
        Eigen::Vector2i(10, 10), // on vertical edge
        Eigen::Vector2i(0, 10), // on vertical edge
        Eigen::Vector2i(5, 5) // inside
    };
    
    std::cout << "Testing inside groundtruth points" << std::endl;
    for (auto point : insidePoint) {
        if (false == IsPointInsidePolygon(polygonPoints, point.x(), point.y())) {
            std::cout << "FAILED (" << point.x() << ", " << point.y() << ") is incorrectly determined as an outside point" << std::endl;
        } else {
            std::cout << "PASSED (" << point.x() << ", " << point.y() << ") is correctly determined as an inside point" << std::endl;
        }
    }
    
    std::vector<Eigen::Vector2i> outsidePoint {
        Eigen::Vector2i(-1, -1),
        Eigen::Vector2i(11, 0),
        Eigen::Vector2i(0, 21)
    };
    
    std::cout << "Testing outside groundtruth points" << std::endl;
    for (auto point : outsidePoint) {
        if (true == IsPointInsidePolygon(polygonPoints, point.x(), point.y())) {
            std::cout << "FAILED (" << point.x() << ", " << point.y() << ") is incorrectly determined as an inside point" << std::endl;
        }else {
            std::cout << "PASSED (" << point.x() << ", " << point.y() << ") is correctly determined as an outside point" << std::endl;
        }
    }
    
    return 0;
}

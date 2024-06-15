//
//  algorithms.hpp
//  homography
//
//  Created by Supannee Tanathong on 15/07/2024.
//

#pragma once

#include <vector>
#include <Eigen/Core>

// Original code by Randolph Franklin
// Reference: https://www.eecs.umich.edu/courses/eecs380/HANDOUTS/PROJ2/InsidePoly.html
// This is a modified version of the above implementation:
//      * Return true if the test point falls inside the polygon or on edge, otherwise return false
//      * Breaking the original short/complicated code into an easily understandable code
//      * Handle exception case
bool IsPointInsidePolygon(const std::vector<Eigen::Vector2i>& polygonPoints, int x, int y) {
    bool pointIsInside = false;

    // Iterate through all the edges forming the polygon
    for (int i = 0; i < polygonPoints.size(); ++i) {
        int j = (i + 1) % polygonPoints.size();
        // i, j are the two vertices forming an edge

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
            // Check if a line from left to right [(0,y), (x,y)] crosses the edge
            //    m = (y2 - y1)/(x2 - x1)
            //    y - y1 = m (x - x1)
            //    x - x1 = (y - y1)/m
            //    x' = (y - y1)/m + x1
            double oneOverSlope = double(polygonPoints[j].x() - polygonPoints[i].x())/(polygonPoints[j].y() - polygonPoints[i].y());
            double xIntersect = oneOverSlope * (y - polygonPoints[i].y()) + polygonPoints[i].x();
            // xIntersect is the x' position (given the passed-in y) that intersects the edge
            if ((int)xIntersect == x) { // If x falls exactly at the intersect point, x is considered inside.
                return true;
            }
            // If x is greater than x', it means the line already crosses the edge.
            if (xIntersect < x) {
                pointIsInside = !pointIsInside;
            }
        }
    }
    
    return pointIsInside;
}

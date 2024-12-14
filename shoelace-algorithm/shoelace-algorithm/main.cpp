//
//  main.cpp
//  shoelace-algorithm
//
//  Created by Supannee Tanathong on 14/12/2024.
//

// Shoelace formula / Gauss's area formula
//  for computing area of polygon when knowing their vertices

#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

struct Point2d {
    double x, y;
    Point2d(double x_val = 0.0, double y_val = 0.0) : x(x_val), y(y_val) {}
};

/*
 * points are stored in the counter-clockwise order.
 * (x0,y0) --> (x1,y1) --> (x2,y2) --> (x3,y3) --> (x4,y4) --> (x0,y0)
 * The points array only needs to contain points (x0,y0)...(xn,yn).
 * It is assumed that the last point is connected with the first one.
 *
 *           1  |                               |
 *  Area =  --- | Σ (x_i * y_i+1 - x_i+1 * y_i) |
 *           2  |                               |
 */
double computeAreaOfPolgyon(const vector<Point2d>& points) {
    if (points.size() < 3) {
        cerr << "A minimum number of 3 points are required, only " << points.size() << " points passed-in." << endl;
        return 0.0;
    }
    
    double area = 0;
    for (int i = 0; i < points.size(); ++i) {
        int j = (i+1) % points.size();
        // An edge is defined from point i and point j
        area += (points[i].x * points[j].y) - (points[j].x * points[i].y);
    }
    return fabs(area) / 2.0;
}

int main(int argc, const char * argv[]) {
    vector<Point2d> points = {
        {5,0}, {9,0}, {10,3}, {10,6}, {7,8}, {6,6}, {3,3}
    };
    double area = computeAreaOfPolgyon(points); // 37
    cout << "Area of the polygon computed using shoelace algorithm is " << area << endl;
    
    return 0;
}

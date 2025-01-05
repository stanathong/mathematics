//
//  main.cpp
//  convex-hull
//
//  Created by Supannee Tanathong on 05/01/2025.
//

// Check if point P is inside a context hull, defined by a sequence of edges.

#include <iostream>
#include <vector>
#include <cmath>

class Point {
public:
    double x, y;
    friend std::ostream& operator << (std::ostream& os, const Point& p);
};

std::ostream& operator << (std::ostream& os, const Point& p) {
    os << "(" << p.x << ", " << p.y << ")";
    return os;
}

// Cross product of vector AB and AP: AB x AP
// AB: (Bx-Ax, By-Ay)
// AP: (Px-Ax, Py-Ay)
// AB x AP =        i        j
//            |   Bx-Ax    By-Ay |
//            |   Px-Ax    Py-Ay |
//
// = (Bx-Ax) * (Py-Ay) - (Px-Ax) * (By-Ay)

double cross(const Point& A, const Point& B, const Point& P) {
    return (B.x - A.x)*(P.y - A.y) - (B.y - A.y)*(P.x - A.x);
}

// polygon is defined as a sequence of point: A0,A1,..,An.
// It is assumed that A0 and An is connected.
bool isPointInsideConvexHull(const std::vector<Point>& polygon, const Point& P) {
    int numPoints = static_cast<int>(polygon.size());
    if (numPoints < 3) {
        std::cerr << "A minimum number of points is 3, but receive " << numPoints << std::endl;
        return false;
    }
    
    // Test by performing cross product of each edge Ai-->Ai+1 with Ai-->Ap
    bool inside = true;
    for (int e = 0; e < numPoints; ++e) {
        const Point& st = polygon[e];
        const Point& en = polygon[(e + 1) % numPoints]; // Making sure last is connected to first
        int ret = cross(st, en, P);
        /*std::cout << "Cross of edge: " << e << " --> " << ((e + 1) % numPoints)
            << " and " << e << " --> P : " << ret << std::endl;*/
        inside &= (ret >= 0); // Point is inside if all cross product > 0, 0 is on edge
    }
    return inside;
}

int main(int argc, const char * argv[]) {
    std::vector<Point> polygon = {
        {0,0}, {4,0}, {9,3}, {6,8}, {0,6}
    };
    
    Point test1 = {4,4}; // inside
    bool result1 = isPointInsideConvexHull(polygon, test1);
    std::cout << test1 << " is " << (result1 ? "inside" : "outside") << std::endl;
    
    Point test2 = {2,9}; // outside
    bool result2 = isPointInsideConvexHull(polygon, test2);
    std::cout << test2 << " is " << (result2 ? "inside" : "outside") << std::endl;
    
    return 0;
}

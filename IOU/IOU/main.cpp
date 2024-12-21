//
//  main.cpp
//  IOU
//
//  Created by Supannee Tanathong on 21/12/2024.
//
// Compute IOU between two rectangles.

#include <iostream>
#include <cmath>
#include <stdexcept>

class Vector2d {
public:
    double x, y;
    Vector2d() : x(0), y(0) {}
    Vector2d(double x, double y) : x(x), y(y) {}
    // Copy constructor
    Vector2d(const Vector2d& other) {
        x = other.x;
        y = other.y;
    }
    // Assignment operator
    Vector2d& operator=(const Vector2d& other) {
        x = other.x;
        y = other.y;
        return *this;
    }
    // Element access
    double& operator()(size_t index) {
        if (index > 1)
            throw std::out_of_range("Index out of bound");
        return (index == 0) ? x : y;
    }
    double operator()(size_t index) const {
        if (index > 1)
            throw std::out_of_range("Index out of bound");
        return (index == 0) ? x : y;
    }
};

// Coordinate system: x to right, y down
class BBox2D {
public:
    double width = 0;
    double height = 0;
    Vector2d top_left_corner;
    BBox2D() {}
    BBox2D(const double& width, const double& height, const Vector2d& top_left_corner) :
        width(width), height(height) {
        this->top_left_corner = top_left_corner;
    }
    BBox2D(const double& width, const double& height, const double& top_left_x, const double& top_left_y) :
        width(width), height(height), top_left_corner(top_left_x, top_left_y) {}
    BBox2D(const double&& width, const double&& height, const double&& top_left_x, const double&& top_left_y) :
        width(width), height(height), top_left_corner(top_left_x, top_left_y) {}
    
    // Helper funtions
    double left() const { return top_left_corner(0); }
    double top() const { return top_left_corner(1); }
    double right() const { return top_left_corner(0) + width; }
    double bottom() const { return top_left_corner(1) + height; }
    double area() const { return width * height; }
    
    static bool isIntersect(const BBox2D& box1, const BBox2D& box2) {
        bool xIntersect = fmax(box1.left(), box2.left()) < fmin(box1.right(), box2.right());
        bool yIntersect = fmax(box1.top(), box2.top()) < fmin(box1.bottom(), box2.bottom());
        return xIntersect && yIntersect;
    }
};

// IoU = intersect(box1, box2) / union(box1,box2)
double computeIoU(const BBox2D& box1, const BBox2D& box2) {
    if (!BBox2D::isIntersect(box1, box2))
        return 0.0;
    
    // The two boxes in intersect.
    // Compute intersection between 2 boxes
    double xmin = fmax(box1.left(), box2.left());
    double xmax = fmin(box1.right(), box2.right());
    double ymin = fmax(box1.top(), box2.top());
    double ymax = fmin(box1.bottom(), box2.bottom());
    double intersectArea = (xmax - xmin) * (ymax - ymin);
    
    // Compute union between 2 boxes
    double unionArea = box1.area() + box2.area() - intersectArea;
    
    return intersectArea / unionArea;
}

int main(int argc, const char * argv[]) {
    /*
        Box1  --------------------
              |       -----------|------- Box2
              |       |          |      |
              |       -----------|-------
              |                  |
              --------------------
     
        Box1 = top_left (0,2), bottom_right (8,10), width = 8-0 = 8, height = 10-2 = 8
        Box2 = top_left (6,4), bottom right (12,6), width = 12-6 = 6, height = 6-4 = 2
     */
    // BBox2D(width, height, top_left_x, top_left_y)
    BBox2D box1(8, 8, 0, 2);
    BBox2D box2(6, 2, 6, 4);
    std::cout << "IoU = " << computeIoU(box1, box2) << std::endl;
    std::cout << "IoU = " << computeIoU(box2, box1) << std::endl;
    
    return 0;
}

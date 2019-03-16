//
// Created by yangt on 19-3-11.
//
#include "point.hpp"

Point::Point() : x_(0.0), y_(0.0) {
    std::cout << "defaule constructor!\n";
}

Point::Point(double x, double y) : x_(x), y_(y) {
    std::cout << "constructor with input (x, y): " << "(" << x << ", " << y
              << ")\n";
}

Point::Point(const Point &point) : x_(point.x()), y_(point.y()) {
    std::cout << "constructor with input point: " << point;
}

Point Point::operator+(const Point &point) const {
    return Point(x_ + point.x(), y_ + point.y());
}

Point Point::operator-(const Point &point) const {
    return Point(x_ - point.x(), y_ - point.y());
}

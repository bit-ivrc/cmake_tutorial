//
// Created by yangt on 19-3-11.
//

#ifndef SIMPLE_CMAKE_EXAMPLE_TEST_CLASS_HPP
#define SIMPLE_CMAKE_EXAMPLE_TEST_CLASS_HPP
#include <iostream>

class Point {
 public:
  /**
   * default constructor
   */
  Point();

  /**
   * construct from inputing (x, y)
   * @param x : x value
   * @param y : y value
   */
  Point(double x, double y);

  /**
   * construct from another point
   * @param point a point
   */
  Point(const Point &point);

  /**
   * overload '+' to achieve point addtive operation
   * @param point the addend point
   * @return the result of point addtive
   */
  Point operator+(const Point &point) const;

  /**
   * overload '-' to achieve point substraction
   * @param point the minuend point
   * @return the result of point substraction
   */
  Point operator-(const Point &point) const;

  friend std::ostream &operator<<(std::ostream &out, const Point &pt) {
      out << "(" << pt.x() << ", " << pt.y() << ")\n";
      return out;
  }

  const double &x() const {
      return x_;
  }
  const double &y() const {
      return y_;
  }

 private:
  double x_;
  double y_;
};
#endif //SIMPLE_CMAKE_EXAMPLE_TEST_CLASS_HPP

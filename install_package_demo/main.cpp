#include <iostream>
#include "tinysplinecpp.h"
int main() {
	std::cout << "Hello, World!" << std::endl;
	tinyspline::BSpline clamped_spline(10, 2, 5, TS_CLAMPED);
	return 0;
}
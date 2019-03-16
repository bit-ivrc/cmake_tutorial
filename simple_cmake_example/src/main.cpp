#include <iostream>
#include <opencv/cv.hpp>
#include <boost/filesystem.hpp>
#include "point.hpp"

int main() {
    std::cout << "**** Simple Cmake Example ****" << std::endl;
    // here show the different construct method, please note the output
    Point pt1(10, 20);
    Point pt2(1, -1);
    Point pt3(pt1 + pt2);
    Point pt4 = pt1;
    std::cout << "pt1: " << pt1 << "pt2: " << pt2 << "pt3: " << pt3;

    // opencv show the image:
    std::string current_path = boost::filesystem::current_path().string();
    std::string image_path = current_path + "/../image.png";
    std::cout << "image path: " << image_path << "\n";
    cv::Mat src = cv::imread(image_path);
    cv::imshow("image", src);
    cv::waitKey();
    return 0;
}
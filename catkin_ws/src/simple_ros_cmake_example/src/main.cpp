#include <opencv/cv.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include "point.hpp"

void goalCb(const geometry_msgs::PoseStamped &pose) {
    std::cout << "recive pose: " << pose;
}

int main(int argc, char **argv) {
    // init ros
    ros::init(argc, argv, "simple_ros_cmake_example_node");
    ros::NodeHandle nh;
    ros::Subscriber
            subscriber = nh.subscribe("/move_base_simple/goal", 1, goalCb);
    ros::Publisher
            publisher = nh.advertise<nav_msgs::OccupancyGrid>("/grid_map", 1);

    // here show the different construct method, please note the output
    Point pt1(10, 20);
    Point pt2(1, -1);
    Point pt3(pt1 + pt2);
    Point pt4 = pt1;
    std::cout << "pt1: " << pt1 << "pt2: " << pt2 << "pt3: " << pt3;

    // initialize grid map from image:
    std::string base_dir = ros::package::getPath("simple_ros_cmake_example");
    std::string image_path = base_dir + "/image.png";
    std::cout << "image path: " << image_path << "\n";
    cv::Mat src = cv::imread(image_path);
    const double resolution = 0.1;
    grid_map::Position center(0.0, 0.0);
    grid_map::GridMap map;

    grid_map::GridMapCvConverter::initializeFromImage(src,
                                                      resolution,
                                                      map,
                                                      center);
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(src,
                                                                      "obstacle",
                                                                      map,
                                                                      0,
                                                                      255,
                                                                      0.5);
    nav_msgs::OccupancyGrid occupancy_map;
    grid_map::GridMapRosConverter::toOccupancyGrid(map,
                                                   "obstacle",
                                                   255,
                                                   0,
                                                   occupancy_map);
    ros::Rate rate(1);
    while (ros::ok()) {
        ros::spinOnce();
        publisher.publish(occupancy_map);
    }
    return 0;
}
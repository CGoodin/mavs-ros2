/**
* \file mavs_ros_uitls.h
*
* A set of functions for copying data from 
* MAVS data types to ROS2 data types
* and other utilities.
*
* \author Chris Goodin
*
* \date 9/23/2022
*/
#ifndef MAVS_ROS_UTILS_H
#define MAVS_ROS_UTILS_H

// C++ includes
#include <iostream>
// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <sensor_msgs/msg/point_field.hpp>
// mavs includes
#include <mavs_core/messages.h>
#include "CImg.h"
#ifdef Success
  #undef Success
#endif

namespace mavs_ros_utils{

template<typename T>
void CImgToImage(cimg_library::CImg<T> *in_image, sensor_msgs::msg::Image &out_image) {
    out_image.height = in_image->height();
    out_image.width = in_image->width();
    out_image.is_bigendian = false;
    out_image.encoding = "rgb8";
    if (out_image.height == 0 || out_image.width == 0) {
        return;
    }
    out_image.data.resize(3 * out_image.height * out_image.width);
    int n = 0;
    for (int j = 0; j < (int)out_image.height; j++) {
        for (int i = 0; i < out_image.width; i++) {
            for (int k = 0; k < 3; k++) {
                out_image.data[n] = (uint8_t)in_image->operator()(i, j, k);
                n++;
            }
        }
    }
    out_image.step = (uint32_t)(sizeof(uint8_t) * out_image.data.size() / out_image.height);
}

template<typename T>
void ImageToCImg(sensor_msgs::msg::Image &in_image, cimg_library::CImg<T> &out_image) {
    int channels = in_image.step / in_image.width;
    out_image.assign(in_image.width, in_image.height, 1, channels, 0);
    if (out_image.height() == 0 || out_image.width() == 0) {
        return;
    }
    int n = 0;
    for (int j = 0; j < (int)out_image.height(); j++) {
        for (int i = 0; i < out_image.width(); i++) {
            for (int k = 0; k < channels; k++) {
                out_image(i, j, k) = (T)in_image.data[n];
                n++;
            }
        }
    }
}

double GetHeadingFromOrientation(geometry_msgs::msg::Quaternion orientation);

sensor_msgs::msg::PointCloud2 CopyFromMavsPc2(mavs::PointCloud2 mavs_pc);

void CopyFromMavsImage(sensor_msgs::msg::Image &image, mavs::Image &mavs_image);

void CopyFromMavsFix(sensor_msgs::msg::NavSatFix &fix, mavs::NavSatFix &mavs_fix);

nav_msgs::msg::Odometry CopyFromMavsVehicleState(mavs::VehicleState state);

void CopyFromMavsOdometry(nav_msgs::msg::Odometry &odom, mavs::Odometry &mavs_odom);

void CopyFromMavsGrid(nav_msgs::msg::OccupancyGrid &grid, mavs::OccupancyGrid &mavs_grid);

double PointLineDistance(glm::dvec2 x1, glm::dvec2 x2, glm::dvec2 x0);

double PointToSegmentDistance(glm::dvec2 ep1, glm::dvec2 ep2, glm::dvec2 p);

inline std::string ToString(int x, int zero_padding) {
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(zero_padding) << x;
    return ss.str();
}

} //namespace mavs_ros_utils

#endif


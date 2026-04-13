#include "mavs-ros2/mavs_ros_utils.h"

// C++ includes
#include <iostream>
#include <cstring> // for memcpy

// ROS2 includes
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

namespace mavs_ros_utils{

double GetHeadingFromOrientation(geometry_msgs::msg::Quaternion orientation){
    tf2::Quaternion q(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

std_msgs::msg::Float32MultiArray ToFloat32MultiArray(const std::vector<std::vector<float>>& data_in){
    std_msgs::msg::Float32MultiArray msg;

    int nx = (int)data_in.size();
    int ny = 0;
    if (nx > 0)ny = (int)data_in[0].size();

    // Set layout dimensions
    msg.layout.dim.resize(2);

    msg.layout.dim[0].label = "x";
    msg.layout.dim[0].size = nx;
    msg.layout.dim[0].stride = nx * ny;  // total elements from this dim onward

    msg.layout.dim[1].label = "y";
    msg.layout.dim[1].size = ny;
    msg.layout.dim[1].stride = ny;       // elements from this dim onward

    msg.layout.data_offset = 0;

    // Flatten row-major into the data vector
    msg.data.reserve(nx * ny);
    for (int i = 0; i < nx; ++i)
        for (int j = 0; j < ny; ++j)
            msg.data.push_back(data_in[i][j]);

    return msg;
}

sensor_msgs::msg::PointCloud2 CopyFromMavsPc2(mavs::PointCloud2 mavs_pc){
    sensor_msgs::msg::PointCloud2 pc;
    pc.height = mavs_pc.height;
    pc.width = mavs_pc.width;
    pc.is_bigendian = mavs_pc.is_bigendian;
    pc.point_step = mavs_pc.point_step;
    pc.row_step = mavs_pc.row_step;
    pc.is_dense = mavs_pc.is_dense;
    for (int i=0;i<(int)mavs_pc.fields.size();i++){
        sensor_msgs::msg::PointField field;
        field.name = mavs_pc.fields[i].name;
        field.offset = mavs_pc.fields[i].offset;
        field.datatype = mavs_pc.fields[i].datatype;
        field.count = mavs_pc.fields[i].count;
        pc.fields.push_back(field);
    }
    int data_size = pc.row_step*mavs_pc.height;
    pc.data.resize(data_size);
    memcpy(&pc.data[0],&mavs_pc.data[0],data_size);
    return pc;
}

sensor_msgs::msg::PointCloud2 CopyFromMavsPc2FullyAttributed(mavs::PointCloud2FullyAttributed mavs_pc) {
    sensor_msgs::msg::PointCloud2 pc;
    pc.height = mavs_pc.height;
    pc.width = mavs_pc.width;
    pc.is_bigendian = mavs_pc.is_bigendian;
    pc.point_step = mavs_pc.point_step;
    pc.row_step = mavs_pc.row_step;
    pc.is_dense = mavs_pc.is_dense;
    for (int i = 0; i < (int)mavs_pc.fields.size(); i++) {
        sensor_msgs::msg::PointField field;
        field.name = mavs_pc.fields[i].name;
        field.offset = mavs_pc.fields[i].offset;
        field.datatype = mavs_pc.fields[i].datatype;
        field.count = mavs_pc.fields[i].count;
        pc.fields.push_back(field);
    }
    int data_size = pc.row_step * mavs_pc.height;
    pc.data.resize(data_size);
    memcpy(&pc.data[0], &mavs_pc.data[0], data_size);
    return pc;
}

void CopyFromMavsImage(sensor_msgs::msg::Image &image, mavs::Image &mavs_image){
    image.height = mavs_image.height;
    image.width = mavs_image.width;
    image.encoding = mavs_image.encoding;
    image.is_bigendian = mavs_image.is_bigendian;
    image.step = mavs_image.step;
    image.data = mavs_image.data;
}

void CopyFromMavsFix(sensor_msgs::msg::NavSatFix &fix, mavs::NavSatFix &mavs_fix){
    fix.latitude = mavs_fix.latitude;
    fix.longitude = mavs_fix.longitude;
    fix.altitude = mavs_fix.altitude;
    fix.status.status = mavs_fix.status.status;
    fix.status.service = mavs_fix.status.service;
}

nav_msgs::msg::Odometry CopyFromMavsVehicleState(mavs::VehicleState state){
    nav_msgs::msg::Odometry odom;
    odom.pose.pose.position.x = state.pose.position.x;
    odom.pose.pose.position.y = state.pose.position.y;
    odom.pose.pose.position.z = state.pose.position.z;
    odom.pose.pose.orientation.x = state.pose.quaternion.x;
    odom.pose.pose.orientation.y = state.pose.quaternion.y;
    odom.pose.pose.orientation.z = state.pose.quaternion.z;
    odom.pose.pose.orientation.w = state.pose.quaternion.w;
    odom.twist.twist.linear.x = state.twist.linear.x;
    odom.twist.twist.linear.y = state.twist.linear.y;
    odom.twist.twist.linear.z = state.twist.linear.z;
    odom.twist.twist.angular.x = state.twist.angular.x;
    odom.twist.twist.angular.y = state.twist.angular.y;
    odom.twist.twist.angular.z = state.twist.angular.z;
    return odom;
}

void CopyFromMavsOdometry(nav_msgs::msg::Odometry &odom, mavs::Odometry &mavs_odom){
    odom.pose.pose.position.x = mavs_odom.pose.pose.position.x;
    odom.pose.pose.position.y = mavs_odom.pose.pose.position.y;
    odom.pose.pose.position.z = mavs_odom.pose.pose.position.z;
    odom.pose.pose.orientation.w = mavs_odom.pose.pose.quaternion.w;
    odom.pose.pose.orientation.x = mavs_odom.pose.pose.quaternion.x;
    odom.pose.pose.orientation.y = mavs_odom.pose.pose.quaternion.y;
    odom.pose.pose.orientation.z = mavs_odom.pose.pose.quaternion.z;
    odom.twist.twist.linear.x = mavs_odom.twist.twist.linear.x;
    odom.twist.twist.linear.y = mavs_odom.twist.twist.linear.y;
    odom.twist.twist.linear.z = mavs_odom.twist.twist.linear.z;
    odom.twist.twist.angular.x = mavs_odom.twist.twist.angular.x;
    odom.twist.twist.angular.y = mavs_odom.twist.twist.angular.y;
    odom.twist.twist.angular.z = mavs_odom.twist.twist.angular.z;
}

void CopyFromMavsGrid(nav_msgs::msg::OccupancyGrid &grid, mavs::OccupancyGrid &mavs_grid){
    grid.info.width = mavs_grid.info.width;
    grid.info.height = mavs_grid.info.height;
    grid.info.resolution = mavs_grid.info.resolution;
    grid.info.origin.position.x = mavs_grid.info.origin.position.x; 
    grid.info.origin.position.y = mavs_grid.info.origin.position.y;
    grid.info.origin.position.z = mavs_grid.info.origin.position.z;
    grid.info.origin.orientation.w = mavs_grid.info.origin.quaternion.w;
    grid.info.origin.orientation.x = mavs_grid.info.origin.quaternion.x;
    grid.info.origin.orientation.y = mavs_grid.info.origin.quaternion.y;
    grid.info.origin.orientation.z = mavs_grid.info.origin.quaternion.z;
    grid.data.resize(mavs_grid.data.size());
    for (int i=0;i<(int)mavs_grid.data.size();i++){
        grid.data[i] = (uint8_t)mavs_grid.data[i];
    }
    grid.header.frame_id = "map";
}

double PointLineDistance(glm::dvec2 x1, glm::dvec2 x2, glm::dvec2 x0) {
    glm::dvec3 x01(x0.x - x1.x, x0.y - x1.y, 0.0);
    glm::dvec3 x02(x0.x - x2.x, x0.y - x2.y, 0.0);
    glm::dvec2 x21 = x2 - x1;
    double d = glm::length(glm::cross(x01, x02)) / glm::length(x21);
    return d;
}

double PointToSegmentDistance(glm::dvec2 ep1, glm::dvec2 ep2, glm::dvec2 p) {
    glm::dvec2 v21 = ep2 - ep1;
    glm::dvec2 pv1 = p - ep1;
    if (glm::dot(v21, pv1) <= 0.0) {
        double d = glm::length(pv1);
        return d;
    }
    glm::dvec2 v12 = ep1 - ep2;
    glm::dvec2 pv2 = p - ep2;
    if (glm::dot(v12, pv2) <= 0.0) {
        double d = glm::length(pv2);
        return d;
    }
    double d0 = PointLineDistance(ep1, ep2, p);
    return d0;
}

// Cross product
geometry_msgs::msg::Point PointCross(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b ) {
    geometry_msgs::msg::Point c;
    c.x = a.y * b.z - a.z * b.y;
    c.y = a.z * b.x - a.x * b.z;
    c.z = a.x * b.y - a.y * b.x;
    return c;
}

// Normalize a quaternion
geometry_msgs::msg::Quaternion NormalizeQuat(const geometry_msgs::msg::Quaternion& q) {
    double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm < 1e-10) return q;
    geometry_msgs::msg::Quaternion q_out;
    q_out.w = q.w / norm;
    q_out.x = q.x / norm;
    q_out.y = q.y / norm;
    q_out.z = q.z / norm;
    return q_out;
}

// Normalize a Point
geometry_msgs::msg::Point NormalizePoint(const geometry_msgs::msg::Point& p) {
    double norm = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    if (norm < 1e-10) return p;
    geometry_msgs::msg::Point p_out;
    p_out.x = p.x / norm;
    p_out.y = p.y / norm;
    p_out.z = p.z / norm;
    return p_out;
}

// SLERP between two quaternions
geometry_msgs::msg::Quaternion Slerp(const geometry_msgs::msg::Quaternion& q1, const geometry_msgs::msg::Quaternion& q2_raw, double t) {
    geometry_msgs::msg::Quaternion qa = NormalizeQuat(q1);
    geometry_msgs::msg::Quaternion qb = NormalizeQuat(q2_raw);

    double d = qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z; // dot product

    // Ensure shortest path by flipping qb if dot product is negative
    if (d < 0.0) {
        qb.w = -1.0 * qb.w;
        qb.x = -1.0 * qb.x;
        qb.y = -1.0 * qb.y;
        qb.z = -1.0 * qb.z;
        d = -d;
    }

    // If quaternions are very close, fall back to LERP to avoid division by zero
    if (d > 0.9995) {
        geometry_msgs::msg::Quaternion result; // = {
        result.w = qa.w + t * (qb.w - qa.w);
        result.x = qa.x + t * (qb.x - qa.x);
        result.y = qa.y + t * (qb.y - qa.y);
        result.z = qa.z + t * (qb.z - qa.z);
        //};
        return NormalizeQuat(result);
    }

    double theta_0 = std::acos(d);
    double theta = theta_0 * t;
    double sin_t = std::sin(theta);
    double sin_t0 = std::sin(theta_0);

    double s1 = std::cos(theta) - d * sin_t / sin_t0;
    double s2 = sin_t / sin_t0;

    geometry_msgs::msg::Quaternion q_out;
    q_out.w = s1 * qa.w + s2 * qb.w;
    q_out.x = s1 * qa.x + s2 * qb.x;
    q_out.y = s1 * qa.y + s2 * qb.y;
    q_out.z = s1 * qa.z + s2 * qb.z;
    q_out = NormalizeQuat(q_out);
    return q_out;
}

// LERP between two 3D positions
geometry_msgs::msg::Point Lerp(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2, double t) {
    geometry_msgs::msg::Point v;
    v.x = p1.x + t * (p2.x - p1.x);
    v.y = p1.y + t * (p2.y - p1.y);
    v.z = p1.z + t * (p2.z - p1.z); 
    return v;
}

} //namespace mavs_ros_utils

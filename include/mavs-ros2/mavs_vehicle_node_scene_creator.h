#ifndef MAVS_VEHICLE_NODE_SCENE_CREATOR_H
#define MAVS_VEHICLE_NODE_SCENE_CREATOR_H

// package includes
#include "mavs-ros2/mavs_ros_utils.h"
#include "mavs-ros2/mavs_sensor_node.h"

// ros includes
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "geometry_msgs/msg/twist.hpp"

// mavs includes
#include "mavs_core/data_path.h"
#include "vehicles/rp3d_veh/mavs_rp3d_veh.h"
#include "raytracers/embree_tracer/embree_tracer.h"
#include "sensors/mavs_sensors.h"
#include "sensors/camera/rgb_camera.h"
#include "mavs_core/terrain_generator/terrain_elevation_functions.h"

#include <string>
#include <vector>

class MavsVehicleNodeSceneCreator : public MavsNode
{
public:
    MavsVehicleNodeSceneCreator();

private:
    // ROS interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_true_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr anim_poses_pub_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Simulation parameters and state
    float dt_;
    int nsteps_;
    int render_steps_;
    float elapsed_time_;
    float throttle_, steering_, braking_;
    bool render_debug_;
    bool use_human_driver_;
    bool use_sim_time_;

    // Scene creator specific parameters
    std::string mode_;             // "manual" or "automated"
    int iterations_;
    float ditch_depth_start_;
    float ditch_width_start_;
    std::string results_path_;

    std::vector<float> ditch_depths_;
    std::vector<float> top_widths_;
    std::vector<float> selected_depths_;
    std::vector<float> selected_widths_;

    int max_iterations_;
    int current_iteration_;
    int depth_index_;
    int width_index_;

    // MAVS objects
    mavs::environment::Environment env_;
    mavs::raytracer::embree::EmbreeTracer scene_;
    mavs::sensor::camera::RgbCamera camera_;
    mavs::vehicle::Rp3dVehicle mavs_veh_;

    // Member functions
    void TwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void PrepareSimulationParams();
    std::vector<float> GenerateRange(float start, float end, float step);

    void TimerCallback();
    void SetupTerrain(float top_width, float base_width, float depth);
    void ResetVehicleState();
    std::vector<float> RunSimulation();
    void LogResults(float ditch_depth, float ditch_width, const std::vector<float>& speeds);

    void LoadVehicleAndScene();
    void UpdateHumanDrivingCommands();
};

#endif // MAVS_VEHICLE_NODE_SCENE_CREATOR_H


// package includes
#include "mavs-ros2/mavs_ros_utils.h"
#include "mavs-ros2/mavs_sensor_node.h"
// ros includes
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "geometry_msgs/msg/twist.hpp"
// mavs includes
#include "mavs_core/data_path.h"
#include "vehicles/rp3d_veh/mavs_rp3d_veh.h"
#include "raytracers/embree_tracer/embree_tracer.h"
#include "sensors/mavs_sensors.h"

class MavsActorManagerNode : public MavsNode
{
public:
	MavsActorManagerNode() : MavsNode(){
		dt_ = 0.01;
		nsteps_ = 0;
		elapsed_time_ = 0.0f;
		use_sim_time_ = false;

		odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odometry", 1, std::bind(&MavsActorManagerNode::OdomCallback, this, std::placeholders::_1));

		actor_poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("actor_poses", 10);


		LoadActorParams();

		timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / (1.0/dt_))), std::bind(&MavsActorManagerNode::TimerCallback, this));

	}

	~MavsActorManagerNode(){}

private:
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr actor_poses_pub_;
	
	rclcpp::TimerBase::SharedPtr timer_;

	float veh_x_, veh_y_, veh_heading_;

	bool use_sim_time_;
	double dt_;
	int nsteps_;
	float elapsed_time_;

	geometry_msgs::msg::Pose current_actor_pose_;

	void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr rcv_msg){
		veh_x_ = rcv_msg->pose.pose.position.x;
		veh_y_ = rcv_msg->pose.pose.position.y;
		float qw = rcv_msg->pose.pose.orientation.w;
		float qx = rcv_msg->pose.pose.orientation.x;
		float qy = rcv_msg->pose.pose.orientation.y;
		float qz = rcv_msg->pose.pose.orientation.z;
		veh_heading_ = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
	}

	void LoadActorParams(){
		// determine if sim time will be used
		use_sim_time_ = this->get_parameter("use_sim_time").as_bool();
		dt_ = GetFloatParam("dt", 0.01f);

		// load parameters
		float x_init = GetFloatParam("Initial_X_Position", 0.0f);
		float y_init = GetFloatParam("Initial_Y_Position", 0.0f);
		float z_init = GetFloatParam("Initial_Z_Position", 0.0f);
		float qw_init = GetFloatParam("Initial_QW_Position", 0.0f);
		float qx_init = GetFloatParam("Initial_QX_Position", 0.0f);
		float qy_init = GetFloatParam("Initial_QY_Position", 0.0f);
		float qz_init = GetFloatParam("Initial_QZ_Position", 0.0f);

		current_actor_pose_.orientation.w = qw_init;
		current_actor_pose_.orientation.x = qx_init;
		current_actor_pose_.orientation.y = qy_init;
		current_actor_pose_.orientation.z = qz_init;
		current_actor_pose_.position.x = x_init;
		current_actor_pose_.position.y = y_init;
		current_actor_pose_.position.z = z_init;
	}

	void TimerCallback(){
		
		// publish the tire poses as PoseArray Message
		geometry_msgs::msg::PoseArray actor_poses;
		actor_poses.poses.push_back(current_actor_pose_);
		actor_poses.header.frame_id = this->get_namespace();
		actor_poses.header.stamp = this->now();
		actor_poses_pub_->publish(actor_poses);

		elapsed_time_ += dt_;
		nsteps_++;
	}
};

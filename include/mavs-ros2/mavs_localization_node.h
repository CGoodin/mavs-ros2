#ifndef MAVS_LOCALIZATION_NODE_H_
#define MAVS_LOCALIZATION_NODE_H_
#include <time.h>
// package includes
#include "mavs-ros2/mavs_ros_utils.h"
#include "mavs-ros2/mavs_sensor_node.h"
//ros includes
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class MavsLocalizationNode : public MavsSensorNode{
  public:
	MavsLocalizationNode(): MavsSensorNode(){

		LoadLocalizationParams();

		odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("sim_odom", 10);

		timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0/update_rate_hz_)),std::bind(&MavsLocalizationNode::TimerCallback, this));

		dt_ = 1.0/update_rate_hz_;

		srand( (unsigned)time( NULL ) );
		float sx = (float)rand()/(float)RAND_MAX;
		float sy = (float)rand()/(float)RAND_MAX;
		
		sx -= 0.5f;
		sy -= 0.5f;
		sx *= 2.0f;
		sy *= 2.0f;
		float sn = sqrtf(sx*sx + sy*sy);
		ox_ = sx/sn;
		oy_ = sy/sn;
	}

	~MavsLocalizationNode(){}

  private:
	// class member data
	
	float pose_error_;
	float ox_, oy_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

	// class member functions
	void LoadLocalizationParams(){
		pose_error_ = GetFloatParam("pose_error", 0.0f);
	}

	void TimerCallback(){

		MavsSensorNode::TimerCallback();

		nav_msgs::msg::Odometry sim_pose = pose_;

		float err_mag = pose_error_*(((float)rand()/(float)RAND_MAX) - 0.5f);
		sim_pose.pose.pose.position.x += err_mag*ox_;
		sim_pose.pose.pose.position.y += err_mag*oy_;

		odom_pub_->publish(sim_pose);
    	}

};

#endif

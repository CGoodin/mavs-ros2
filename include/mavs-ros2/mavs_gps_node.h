#ifndef MAVS_RADAR_NODE_H_
#define MAVS_RADAR_NODE_H_
// package includes
#include "mavs-ros2/mavs_ros_utils.h"
#include "mavs-ros2/mavs_sensor_node.h"
//ros includes
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
// mavs includes
#include "mavs_core/data_path.h"
#include "vehicles/rp3d_veh/mavs_rp3d_veh.h"
#include "raytracers/embree_tracer/embree_tracer.h"
#include "sensors/mavs_sensors.h"

class MavsGpsNode : public MavsSensorNode{
  public:
	MavsGpsNode(): MavsSensorNode(){
		
		LoadGpsParams();

		fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps_fix", 10);

		timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0/update_rate_hz_)),std::bind(&MavsGpsNode::TimerCallback, this));

		dt_ = 1.0/update_rate_hz_;
	}

	~MavsGpsNode(){}

  private:
	// class member data
	mavs::sensor::gps::Gps gps_;
	rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;

	// class member functions
	void LoadGpsParams(){
		//Options are: "normal", "dual band", and "differential"
		std::string gps_type = GetStringParam("gps_type", "normal");
		gps_.SetType(gps_type);
		gps_.SetRelativePose(glm::vec3(offset_[0], offset_[1], offset_[2]), glm::quat(relor_[0], relor_[1], relor_[2], relor_[3]));
		
	}

	void TimerCallback(){

		MavsSensorNode::TimerCallback();

		glm::vec3 pos(pose_.pose.pose.position.x, pose_.pose.pose.position.y, pose_.pose.pose.position.z);
		glm::quat ori(pose_.pose.pose.orientation.w, pose_.pose.pose.orientation.x, pose_.pose.pose.orientation.y, pose_.pose.pose.orientation.z);

		gps_.SetPose(pos, ori);

		gps_.Update(&env_, dt_);
		
		if (display_)gps_.Display();

		// get the gps returns from mavs and publish them 
		mavs::NavSatFix mavs_sat_fix = gps_.GetRosNavSatFix();
		mavs::NavSatStatus mavs_sat_status = gps_.GetRosNavSatStatus();

		sensor_msgs::msg::NavSatFix sat_fix;
		sat_fix.status.status = mavs_sat_status.status;
		sat_fix.status.service = mavs_sat_status.service;
		sat_fix.latitude = mavs_sat_fix.latitude;
		sat_fix.longitude = mavs_sat_fix.longitude;
		sat_fix.altitude = mavs_sat_fix.altitude;

		sat_fix.header.stamp = this->now();
		sat_fix.header.frame_id = "gps_link";
		fix_pub_->publish(sat_fix);

    }

};

#endif
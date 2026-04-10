#ifndef MAVS_SENSOR_NODE_H_
#define MAVS_SENSOR_NODE_H_
// package includes
#include "mavs-ros2/mavs_node.h"
//ros includes
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
// mavs includes
#include "mavs_core/data_path.h"
#include "vehicles/rp3d_veh/mavs_rp3d_veh.h"
#include "raytracers/embree_tracer/embree_tracer.h"

class MavsSensorNode : public MavsNode {
  public:
	MavsSensorNode(): MavsNode(){
		odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("odometry_true", 10, std::bind(&MavsSensorNode::OdomCallback, this, std::placeholders::_1));
		anim_sub = this->create_subscription<geometry_msgs::msg::PoseArray>("all_poses_pub", 10, std::bind(&MavsSensorNode::PosesCallback, this, std::placeholders::_1));
		
		update_rate_hz_ = 10.0f;
		sensor_position_mode_ = "attached";
		use_full_file_path_ = false;
		LoadParams();

		//timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0/update_rate_hz_)),std::bind(&MavsSensorNode::TimerCallback, this));

		timer_ = this->create_timer(
			std::chrono::milliseconds((int)(1000.0 / update_rate_hz_)),
			std::bind(&MavsSensorNode::TimerCallback, this)
		);
	}

  protected:
  	// class member data
  	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  	rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr anim_sub;
	rclcpp::TimerBase::SharedPtr timer_;
	nav_msgs::msg::Odometry pose_;
	geometry_msgs::msg::PoseArray anim_poses_;
	mavs::environment::Environment env_;
	mavs::raytracer::embree::EmbreeTracer scene_;
	std::vector<float> offset_, fixed_position_;
	std::vector<float> relor_, fixed_orientation_;
	bool display_;
	std::string sensor_position_mode_;
	float update_rate_hz_;
	float dt_;
	bool use_full_file_path_;

	void LoadParams(){
		std::string scene_file = GetStringParam( "scene_file", "cube_scene.json");
		offset_ = GetFloatArrayParam("offset",std::vector<float>(0));
		relor_ = GetFloatArrayParam("orientation",std::vector<float>(0));
		display_ = GetBoolParam("display", true);
		use_full_file_path_ = GetBoolParam("use_full_file_path", false);
		sensor_position_mode_ = GetStringParam("sensor_position_mode", "attached");
		if (sensor_position_mode_ == "fixed" || sensor_position_mode_=="follow") {
			fixed_position_ = offset_;
			fixed_orientation_ = relor_;
			offset_[0] = 0.0f; offset_[1] = 0.0f; offset_[2] = 0.0f;
			relor_[0] = 1.0f; relor_[1] = 0.0f; relor_[2] = 0.0f; relor_[3] = 0.0f;
		}
		update_rate_hz_ = GetFloatParam("update_rate_hz", 10.0f);
		std::vector<std::string> vehicle_files = GetStringArrayParam("vehicle_files", std::vector<std::string>(0));
		std::vector<std::string> actor_files = GetStringArrayParam("actor_files", std::vector<std::string>(0));

		dt_ = 1.0f/update_rate_hz_;

		mavs::MavsDataPath mdp;
		std::string mavs_data_path = mdp.GetPath();
		std::string scene_file_path;
		if (use_full_file_path_) {
			scene_file_path = mavs_data_path + "/scenes/" + scene_file;
		}
		else {
			scene_file_path = scene_file;
		}
		scene_.Load(scene_file_path);

		scene_.TurnOffLabeling();
		env_.SetRaytracer(&scene_);
        float rain_rate = GetFloatParam("env_params.rain_rate", 0.0f);
		float snow_rate = GetFloatParam("env_params.snow_rate", 0.0f);
		int year = GetIntParam("env_params.year", 2026);
		int month = GetIntParam("env_params.month", 1);
		int date = GetIntParam("env_params.date", 22);
		int hour = GetIntParam("env_params.hour", 12);
		int minute = GetIntParam("env_params.minute", 0);
		int second = GetIntParam("env_params.second", 0);
		int time_zone = GetIntParam("env_params.time_zone", 6);
		env_.SetRainRate(rain_rate);
		env_.SetSnowRate(snow_rate);
        env_.SetDateTime(year, month, date, hour, minute, second, time_zone);
		for (int nv =0; nv<(int)vehicle_files.size();nv++){
			mavs::vehicle::Rp3dVehicle mavs_veh;
			std::string veh_file_path;
			if (use_full_file_path_) {
				veh_file_path = mavs_data_path + "/vehicles/rp3d_vehicles/" + vehicle_files[nv];
			}
			else {
				veh_file_path = vehicle_files[nv];
			}
			mavs_veh.Load(veh_file_path);
			//mavs_veh.Load(mavs_data_path+"/vehicles/rp3d_vehicles/"+vehicle_files[nv]);
			mavs_veh.SetPosition(-10000.0f, -10000.0f, -10000.0f);
			mavs_veh.SetOrientation(1.0f, 0.0f, 0.0f, 0.0f);
			mavs_veh.Update(&env_, 0.0, 0.0, 0.0, 0.00001);
		}

		for (int na = 0; na < (int)actor_files.size(); na++) {
			std::string actor_file_path;
			if (use_full_file_path_) {
				actor_file_path = mavs_data_path + "/actors/actors/" + actor_files[na];
			}
			else {
				actor_file_path = actor_files[na];
			}
			std::vector<int> actor_idv = env_.LoadActors(actor_file_path);
			//std::vector<int> actor_idv = env_.LoadActors(mavs_data_path + "/actors/actors/"+actor_files[na]);
			int actor_num = (int)(actor_idv.size() - 1);
			env_.UnsetActorUpdate(actor_num);
		}

	}

	geometry_msgs::msg::Quaternion GetSensorLookTo(geometry_msgs::msg::Point camPos, geometry_msgs::msg::Point targetPos) {
		glm::vec3 c(camPos.x, camPos.y, camPos.z);
		glm::vec3 v(targetPos.x, targetPos.y, targetPos.z);
		glm::vec3 look_to = v - c;
		look_to = glm::normalize(look_to);
		glm::vec3 world_up(0.0f, 0.0f, 1.0f);
		glm::vec3 look_side = glm::cross(world_up, look_to);
		look_side = glm::normalize(look_side);
		glm::vec3 look_up = glm::cross(look_to, look_side);
		look_up = glm::normalize(look_up);
		glm::mat3 R(look_to, look_side, look_up);
		glm::quat q = glm::quat_cast(R);
		geometry_msgs::msg::Quaternion q_out;
		q = glm::normalize(q);
		q_out.w = q.w; q_out.x = q.x; q_out.y = q.y; q_out.z = q.z;
		return q_out;
	}

	void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr rcv_msg){
		if (sensor_position_mode_ == "fixed") {
			pose_.pose.pose.position.x = fixed_position_[0];
			pose_.pose.pose.position.y = fixed_position_[1];
			pose_.pose.pose.position.z = fixed_position_[2];
			pose_.pose.pose.orientation.w = fixed_orientation_[0];
			pose_.pose.pose.orientation.x = fixed_orientation_[1];
			pose_.pose.pose.orientation.y = fixed_orientation_[2];
			pose_.pose.pose.orientation.z = fixed_orientation_[3];
		}
		else if (sensor_position_mode_ == "follow") {
			pose_.pose.pose.position.x = fixed_position_[0];
			pose_.pose.pose.position.y = fixed_position_[1];
			pose_.pose.pose.position.z = fixed_position_[2];
			geometry_msgs::msg::Point cam_pos; 
			cam_pos.x = fixed_position_[0]; cam_pos.y = fixed_position_[1]; cam_pos.z = fixed_position_[2];
			pose_.pose.pose.orientation = GetSensorLookTo(cam_pos, rcv_msg->pose.pose.position);
		}
		else { // assume "attached" is the default
			pose_ = *rcv_msg;
		}
	}

	void PosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr rcv_msg){
		anim_poses_ = *rcv_msg;
	}

	void TimerCallback(){
		if (env_.GetNumberOfActors()>=(int)(anim_poses_.poses.size())){
			for (int i=0;i<(int)anim_poses_.poses.size();i++){
				glm::vec3 tpos(anim_poses_.poses[i].position.x, anim_poses_.poses[i].position.y, anim_poses_.poses[i].position.z);
				glm::quat tori(anim_poses_.poses[i].orientation.w, anim_poses_.poses[i].orientation.x, anim_poses_.poses[i].orientation.y, anim_poses_.poses[i].orientation.z);
				env_.SetActorPosition(i, tpos, tori, dt_, true);
			}
		}
    }

};

#endif

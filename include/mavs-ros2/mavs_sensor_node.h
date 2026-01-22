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

		LoadParams();

		timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0/update_rate_hz_)),std::bind(&MavsSensorNode::TimerCallback, this));
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
	std::vector<float> offset_;
	std::vector<float> relor_;
	bool display_;
	float update_rate_hz_;
	float dt_;

	void LoadParams(){
		std::string scene_file = GetStringParam( "scene_file", "cube_scene.json");
		offset_ = GetFloatArrayParam("offset",std::vector<float>(0));
		relor_ = GetFloatArrayParam("orientation",std::vector<float>(0));
		display_ = GetBoolParam("display", true);
		update_rate_hz_ = GetFloatParam("update_rate_hz", 10.0f);
		std::vector<std::string> vehicle_files = GetStringArrayParam("vehicle_files", std::vector<std::string>(0));
		dt_ = 1.0f/update_rate_hz_;

		mavs::MavsDataPath mdp;
		std::string mavs_data_path = mdp.GetPath();
		scene_.Load(mavs_data_path+"/scenes/"+scene_file);
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
			mavs_veh.Load(mavs_data_path+"/vehicles/rp3d_vehicles/"+vehicle_files[nv]);
			mavs_veh.SetPosition(-10000.0f, -10000.0f, -10000.0f);
			mavs_veh.SetOrientation(1.0f, 0.0f, 0.0f, 0.0f);
			mavs_veh.Update(&env_, 0.0, 0.0, 0.0, 0.00001);
		}
	}

	void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr rcv_msg){
		pose_ = *rcv_msg;
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

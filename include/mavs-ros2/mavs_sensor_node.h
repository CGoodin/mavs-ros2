// c++ includes
//#include <omp.h>
//#include <unistd.h>
//ros includes
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
// package includes
//#include "mavs-ros2/mavs_ros_utils.h"
#include "mavs-ros2/mavs_node.h"
// mavs includes
#include "mavs_core/data_path.h"
#include "vehicles/rp3d_veh/mavs_rp3d_veh.h"
#include "raytracers/embree_tracer/embree_tracer.h"

class MavsSensorNode : public MavsNode {
  public:
	MavsSensorNode(): MavsNode(){
		std::cout<<"Creating base sensor node "<<std::endl;
		odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("odometry_true", 10, std::bind(&MavsSensorNode::OdomCallback, this, std::placeholders::_1));
		anim_sub = this->create_subscription<geometry_msgs::msg::PoseArray>("all_poses_pub", 10, std::bind(&MavsSensorNode::PosesCallback, this, std::placeholders::_1));
		
		update_rate_hz = 10.0f;
		LoadParams();

		LoadMavsData();

		timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0/update_rate_hz)),std::bind(&MavsSensorNode::TimerCallback, this));
	}

  protected:
	void LoadMavsData(){
		mavs::MavsDataPath mdp;
		std::string mavs_data_path = mdp.GetPath();
		scene.Load(mavs_data_path+"/scenes/"+scene_file);
		scene.TurnOffLabeling();
		env.SetRaytracer(&scene);

		for (int nv =0; nv<(int)vehicle_files.size();nv++){
			mavs::vehicle::Rp3dVehicle mavs_veh;
			mavs_veh.Load(mavs_data_path+"/vehicles/rp3d_vehicles/"+vehicle_files[nv]);
			mavs_veh.SetPosition(-10000.0f, -10000.0f, -10000.0f);
			mavs_veh.SetOrientation(1.0f, 0.0f, 0.0f, 0.0f);
			mavs_veh.Update(&env, 0.0, 0.0, 0.0, 0.00001);
		}
	}

	void LoadParams(){
		scene_file = GetStringParam( "scene_file", "cube_scene.json");
		offset = GetFloatArrayParam("offset",std::vector<float>(0));
		relor = GetFloatArrayParam("orientation",std::vector<float>(0));
		display = GetBoolParam("display", true);
		update_rate_hz = GetFloatParam("update_rate_hz", 10.0f);
		vehicle_files = GetStringArrayParam("vehicle_files", std::vector<std::string>(0));
		dt = 1.0f/update_rate_hz;
	}

  	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  	rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr anim_sub;

	rclcpp::TimerBase::SharedPtr timer_;

	nav_msgs::msg::Odometry pose_;
	geometry_msgs::msg::PoseArray anim_poses_;

	mavs::environment::Environment env;
	mavs::raytracer::embree::EmbreeTracer scene;
	std::string scene_file;
	std::vector<float> offset;
	std::vector<float> relor;
	bool display;
	float update_rate_hz;
	float dt;
	std::vector<std::string> vehicle_files;

	void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr rcv_msg){
		pose_ = *rcv_msg;
	}

	void PosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr rcv_msg){
		anim_poses_ = *rcv_msg;
	}

	void TimerCallback(){
        RCLCPP_INFO(this->get_logger(), "MAVS Sensor base class callback");
		if (env.GetNumberOfActors()>=(int)(anim_poses_.poses.size())){
			for (int i=0;i<(int)anim_poses_.poses.size();i++){
				glm::vec3 tpos(anim_poses_.poses[i].position.x, anim_poses_.poses[i].position.y, anim_poses_.poses[i].position.z);
				glm::quat tori(anim_poses_.poses[i].orientation.w, anim_poses_.poses[i].orientation.x, anim_poses_.poses[i].orientation.y, anim_poses_.poses[i].orientation.z);
				env.SetActorPosition(i, tpos, tori, dt, true);
			}
		}
    }

};

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
#include "mavs_core/terrain_generator/terrain_elevation_functions.h"
// c++ includes 
#include <fstream>
#include <cmath>
#include <filesystem>
#include <numeric>      
#include <iomanip>      
#include <ctime> 

class MavsVehicleNodeSceneCreator : public MavsNode
{
public:
	MavsVehicleNodeSceneCreator() : MavsNode(){
		dt_ = 0.01;
		nsteps_ = 0;
		elapsed_time_ = 0.0f;
		current_iteration_ = 0;

		twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&MavsVehicleNodeSceneCreator::TwistCallback, this, std::placeholders::_1));
		odom_true_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry_true", 10);
		anim_poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("anim_poses", 10);

		LoadVehicleParams();

		timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / (1.0/dt_))), std::bind(&MavsVehicleNodeSceneCreator::TimerCallback, this));
		render_steps_ = std::max(1,(int)(0.1f/dt_));
	}

	~MavsVehicleNodeSceneCreator() { LogResults(); }

	void SetCurrentIteration(int curr_iter) { current_iteration_ = curr_iter; }
private:
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_true_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr anim_poses_pub_;
	rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
	
	float throttle_;
	float steering_;
	float braking_;
	rclcpp::TimerBase::SharedPtr timer_;
	mavs::environment::Environment env_;
	mavs::raytracer::embree::EmbreeTracer *scene_ptr_;
	mavs::terraingen::TerrainCreator terrain_creator_;
	bool render_debug_;
	bool use_human_driver_;
	bool use_sim_time_;
	mavs::sensor::camera::RgbCamera camera_;
	mavs::vehicle::Rp3dVehicle mavs_veh_;
	double dt_;
	int nsteps_;
	int render_steps_;
	float elapsed_time_;

	// ditch parameters
	float ditch_depth_;
	float ditch_width_;
	// logging parameters
	int current_iteration_;
	std::string results_path_;
	std::vector<float> logged_speeds_;

	void TwistCallback(const geometry_msgs::msg::Twist::SharedPtr rcv_msg){
		throttle_ = rcv_msg->linear.x;
		braking_ = rcv_msg->linear.y;
		steering_ = rcv_msg->angular.z;
	}

	void LoadVehicleParams(){
		// determine if sim time will be used
		use_sim_time_ = this->get_parameter("use_sim_time").as_bool();
		
		if (use_sim_time_){
			clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("clock", 1);
		}

		// load parameters
		//std::string scene_file = GetStringParam("scene_file", "cube_scene.json");
		std::string rp3d_vehicle_file = GetStringParam("rp3d_vehicle_file", "l200.json");
		float soil_strength = GetFloatParam("soil_strength", 250.0f);
		std::string surface_type = GetStringParam("surface_type", "dry");
		float x_init = GetFloatParam("Initial_X_Position", 0.0f);
		float y_init = GetFloatParam("Initial_Y_Position", 0.0f);
		float heading_init = GetFloatParam("Initial_Heading", 0.0f);
		render_debug_ = GetBoolParam("debug_camera", false);
		use_human_driver_ = GetBoolParam("use_human_driver", false);
		dt_ = GetFloatParam("dt",0.01f);

		glm::vec3 initial_position(x_init, y_init, 1.0f);
		glm::quat initial_orientation(cos(0.5 * heading_init), 0.0f, 0.0f, sin(0.5 * heading_init));

		mavs::MavsDataPath mdp;
		std::string mavs_data_path = mdp.GetPath();
		

		float top_width = GetFloatParam("terrain_feature.top_width", 12.0f);
		ditch_width_ = GetFloatParam("terrain_feature.bottom_width", 6.0f);
		ditch_depth_ = GetFloatParam("terrain_feature.depth", 2.0f);
		float ditch_location = GetFloatParam("terrain_feature.location", 20.0f);
		//terrain_creator_.AddTrapezoid(6.0f, 12.0f, 2.0f, 20.0f);
		terrain_creator_.AddTrapezoid(ditch_width_, top_width, ditch_depth_, ditch_location);
		terrain_creator_.CreateTerrain(-25.0f, -25.0f, 200.0f, 25.0f, 0.5f);
		scene_ptr_ = terrain_creator_.GetScenePointer();
		scene_ptr_->TurnOffLabeling();

		env_.SetRaytracer(scene_ptr_);
		env_.SetGlobalSurfaceProperties(surface_type, 6894.76f * soil_strength);

		mavs_veh_.Load(mavs_data_path + "/vehicles/rp3d_vehicles/" + rp3d_vehicle_file);
		mavs_veh_.SetPosition(initial_position.x, initial_position.y, initial_position.z);
		mavs_veh_.SetOrientation(initial_orientation.w, initial_orientation.x, initial_orientation.y, initial_orientation.z);
		mavs_veh_.Update(&env_, throttle_, steering_, braking_, 0.00001);

		camera_.Initialize(640, 480, 0.0046667, 0.0035, 0.0035);
		camera_.SetRenderShadows(true);
		camera_.SetRelativePose(glm::vec3(-9.0, 0.0, 2.5), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));

		results_path_ = GetStringParam("results_path", "/tmp/mavs_results");
		// Ensure results directory exists
		if (!std::filesystem::is_directory(results_path_)) {
			std::filesystem::create_directories(results_path_);
		}
	}

	void LogResults(){
		float avg_speed = 0.0f;
		if (!logged_speeds_.empty()) {
			avg_speed = std::accumulate(logged_speeds_.begin(), logged_speeds_.end(), 0.0f) / logged_speeds_.size();
		}

		std::string filename = results_path_ + "/run_" + std::to_string(current_iteration_) + ".json";

		std::ofstream file(filename);
		if (file.is_open()) {
			file << "{\n";
			//file << "  \"iteration\": " << current_iteration_ << ",\n";
			file << "  \"ditch_depth\": " << ditch_depth_ << ",\n";
			file << "  \"ditch_width\": " << ditch_width_ << ",\n";
			file << "  \"average_speed\": " << avg_speed << ",\n";
			file << "  \"timestamp\": " << std::time(nullptr) << "\n";
			file << "}\n";

			RCLCPP_INFO(this->get_logger(), "Saved results to %s", filename.c_str());
		}
		else {
			RCLCPP_WARN(this->get_logger(), "Failed to open file for saving results: %s", filename.c_str());
		}
	}

	void UpdateHumanDrivingCommands(){
		throttle_ = 0.0;
		steering_ = 0.0;
		braking_ = 0.0;
		std::vector<bool> driving_commands = camera_.GetKeyCommands();
		if (driving_commands[0]){
			throttle_ = 1.0;
		}
		else if (driving_commands[1]){
			braking_ = -1.0;
		}
		if (driving_commands[2]){
			steering_ = 1.0;
		}
		else if (driving_commands[3]){
			steering_ = -1.0;
		}
	}

	void TimerCallback(){
		// vehicle state update
		if (use_human_driver_) UpdateHumanDrivingCommands();

		mavs_veh_.Update(&env_, throttle_, steering_, -braking_, dt_);
		mavs::VehicleState veh_state = mavs_veh_.GetState();
		
		nav_msgs::msg::Odometry true_odom = mavs_ros_utils::CopyFromMavsVehicleState(veh_state);

		float veh_speed = sqrtf(veh_state.twist.linear.x * veh_state.twist.linear.x + veh_state.twist.linear.y * veh_state.twist.linear.y);
		logged_speeds_.push_back(veh_speed);

		if (render_debug_ && nsteps_ % render_steps_ == 0){
			glm::vec3 pos = veh_state.pose.position;
			glm::quat ori = veh_state.pose.quaternion;
			ori.x = 0.0f;
			ori.y = 0.0f;
			ori = glm::normalize(ori);
			camera_.SetPose(pos, ori);
			camera_.Update(&env_, 0.1);
			camera_.Display();
			nsteps_=0;
		}

		// publish the vehicle state as an odometry message
		true_odom.header.stamp = this->now();
		true_odom.header.frame_id = "odom";
		odom_true_pub_->publish(true_odom);

		// publish the tire poses as PoseArray Message
		geometry_msgs::msg::PoseArray anim_poses;
		geometry_msgs::msg::Pose vpose;
		vpose.position = true_odom.pose.pose.position;
		vpose.orientation = true_odom.pose.pose.orientation;
		anim_poses.poses.push_back(vpose);
		for (int i = 0; i < mavs_veh_.GetNumTires(); i++){
			glm::vec3 tpos = mavs_veh_.GetTirePosition(i);
			glm::quat tori = mavs_veh_.GetTireOrientation(i);
			geometry_msgs::msg::Pose tpose;
			tpose.position.x = tpos.x;
			tpose.position.y = tpos.y;
			tpose.position.z = tpos.z;
			tpose.orientation.w = tori.w;
			tpose.orientation.x = tori.x;
			tpose.orientation.y = tori.y;
			tpose.orientation.z = tori.z;
			anim_poses.poses.push_back(tpose);
		}
		anim_poses.header.frame_id = this->get_namespace();
		anim_poses.header.stamp = this->now();
		anim_poses_pub_->publish(anim_poses);

		// clock update
		if (use_sim_time_){
			int sec = (int)floor(elapsed_time_);
			int nsec = (int)(1.0E9f * (elapsed_time_ - (float)sec));
			rclcpp::Time tnow(sec, nsec);
			rosgraph_msgs::msg::Clock cnow;
			cnow.clock = tnow;
			clock_pub_->publish(cnow);
			elapsed_time_ += dt_;
		}
		nsteps_++;
	}
};

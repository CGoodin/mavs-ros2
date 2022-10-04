// ros includes
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
// package includes
#include "mavs-ros2/mavs_ros_utils.h"
#include "mavs-ros2/mavs_sensor_node.h"
// mavs includes
#include "mavs_core/data_path.h"
#include "vehicles/rp3d_veh/mavs_rp3d_veh.h"
#include "raytracers/embree_tracer/embree_tracer.h"
#include "sensors/mavs_sensors.h"

class MavsVehicleNode : public MavsNode
{
public:
	MavsVehicleNode() : MavsNode(){
		dt = 0.01;
		nsteps = 0;
		elapsed_time = 0.0f;

		twist_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&MavsVehicleNode::TwistCallback, this, std::placeholders::_1));
		odom_true_pub = this->create_publisher<nav_msgs::msg::Odometry>("odometry_true", 10);
		anim_poses_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("anim_poses", 10);

		LoadVehicleParams();

		timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / (1.0/dt))), std::bind(&MavsVehicleNode::TimerCallback, this));
	}

	~MavsVehicleNode(){}

private:
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_true_pub;
	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr anim_poses_pub;
	rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub;
	
	float throttle;
	float steering;
	float braking;
	rclcpp::TimerBase::SharedPtr timer_;
	mavs::environment::Environment env;
	mavs::raytracer::embree::EmbreeTracer scene;
	bool render_debug;
	bool use_human_driver;
	bool use_sim_time;
	mavs::sensor::camera::RgbCamera camera;
	mavs::vehicle::Rp3dVehicle mavs_veh;
	double dt;
	int nsteps;
	float elapsed_time;

	void TwistCallback(const geometry_msgs::msg::Twist::SharedPtr rcv_msg){
		throttle = rcv_msg->linear.x;
		braking = rcv_msg->linear.y;
		steering = rcv_msg->angular.z;
	}

	void LoadVehicleParams(){
		// determine if sim time will be used
		use_sim_time = this->get_parameter("use_sim_time").as_bool();
		
		if (use_sim_time){
			clock_pub = this->create_publisher<rosgraph_msgs::msg::Clock>("clock", 1);
		}

		// load parameters
		std::string scene_file = GetStringParam("scene_file", "cube_scene.json");
		std::string rp3d_vehicle_file = GetStringParam("rp3d_vehicle_file", "l200.json");
		float soil_strength = GetFloatParam("soil_strength", 250.0f);
		std::string surface_type = GetStringParam("surface_type", "dry");
		float x_init = GetFloatParam("Initial_X_Position", 0.0f);
		float y_init = GetFloatParam("Initial_Y_Position", 0.0f);
		float heading_init = GetFloatParam("Initial_Heading", 0.0f);
		render_debug = GetBoolParam("debug_camera", false);
		use_human_driver = GetBoolParam("use_human_driver", false);

		glm::vec3 initial_position(x_init, y_init, 1.0f);
		glm::quat initial_orientation(cos(0.5 * heading_init), 0.0f, 0.0f, sin(0.5 * heading_init));

		mavs::MavsDataPath mdp;
		std::string mavs_data_path = mdp.GetPath();
		
		scene.Load(mavs_data_path + "/scenes/" + scene_file);
		scene.TurnOffLabeling();

		env.SetRaytracer(&scene);
		env.SetGlobalSurfaceProperties(surface_type, 6894.76f * soil_strength);

		mavs_veh.Load(mavs_data_path + "/vehicles/rp3d_vehicles/" + rp3d_vehicle_file);
		mavs_veh.SetPosition(initial_position.x, initial_position.y, initial_position.z);
		mavs_veh.SetOrientation(initial_orientation.w, initial_orientation.x, initial_orientation.y, initial_orientation.z);
		mavs_veh.Update(&env, throttle, steering, braking, 0.00001);

		camera.Initialize(256, 256, 0.0035, 0.0035, 0.0035);
		camera.SetRenderShadows(false);
		camera.SetRelativePose(glm::vec3(-10.0, 0.0, 2.0), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
	}

	void TimerCallback(){
		// vehicle state update
		if (use_human_driver){
			throttle = 0.0;
			steering = 0.0;
			braking = 0.0;
			std::vector<bool> driving_commands = camera.GetKeyCommands();
			if (driving_commands[0]){
				throttle = 1.0;
			}
			else if (driving_commands[1]){
				braking = -1.0;
			}
			if (driving_commands[2]){
				steering = 1.0;
			}
			else if (driving_commands[3]){
				steering = -1.0;
			}
		}
		mavs_veh.Update(&env, throttle, steering, -braking, dt);
		mavs::VehicleState veh_state = mavs_veh.GetState();

		nav_msgs::msg::Odometry true_odom = mavs_ros_utils::CopyFromMavsVehicleState(veh_state);

		if (render_debug && nsteps % 10 == 0){
			camera.SetPose(veh_state);
			camera.Update(&env, 0.1);
			camera.Display();
			nsteps=0;
		}

		// publish the vehicle state as an odometry message
		true_odom.header.stamp = this->now();
		odom_true_pub->publish(true_odom);

		// publish the tire poses as PoseArray Message
		geometry_msgs::msg::PoseArray anim_poses;
		geometry_msgs::msg::Pose vpose;
		vpose.position = true_odom.pose.pose.position;
		vpose.orientation = true_odom.pose.pose.orientation;
		anim_poses.poses.push_back(vpose);
		for (int i = 0; i < mavs_veh.GetNumTires(); i++){
			glm::vec3 tpos = mavs_veh.GetTirePosition(i);
			glm::quat tori = mavs_veh.GetTireOrientation(i);
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
		anim_poses_pub->publish(anim_poses);

		// clock update
		if (use_sim_time){
			int sec = (int)floor(elapsed_time);
			int nsec = (int)(1.0E9f * (elapsed_time - (float)sec));
			rclcpp::Time tnow(sec, nsec);
			rosgraph_msgs::msg::Clock cnow;
			cnow.clock = tnow;
			clock_pub->publish(cnow);
			elapsed_time += dt;
		}
		nsteps++;
	}
};

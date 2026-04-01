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


class TriggerEvent {
public:
	std::string type; // can be "x", "y", or "time"
	std::string op;
	float value;
	bool fired;

	TriggerEvent() { fired = false; type = "none"; op = "none"; value = 0.0f; }

	bool Evaluate(float veh_x, float veh_y, float elapsed_time){
		float lhs;
		if (type == "x") lhs = veh_x;
		if (type == "y")lhs = veh_y;
		if (type == "time")lhs = elapsed_time;
		bool triggered = false;
		if (op == ">") { triggered = lhs > value; }
		else if (op == "<") { triggered = lhs < value; }
		else if (op == ">=") { triggered = lhs >= value; }
		else if (op == "<=") { triggered = lhs <= value; }
		else if (op == "==") { triggered = lhs == value; }
		else { triggered = false; }
		if (triggered) {
			fired = true;
			std::cout << "Triggered at " << type << " " << op << " " << value << std::endl;
		}
		return triggered;
	}
};

class MavsActorManagerNode : public MavsNode
{
public:
	MavsActorManagerNode() : MavsNode(){
		dt_ = 0.01;
		nsteps_ = 0;
		elapsed_time_ = 0.0f;
		use_sim_time_ = false;
		odom_rcvd_ = false;

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
	bool odom_rcvd_;

	geometry_msgs::msg::Pose current_actor_pose_;
	geometry_msgs::msg::Pose init_pose_, final_pose_;
	
	TriggerEvent trigger_;
	double trigger_time_;
	double transition_time_;

	void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr rcv_msg){
		veh_x_ = rcv_msg->pose.pose.position.x;
		veh_y_ = rcv_msg->pose.pose.position.y;
		float qw = rcv_msg->pose.pose.orientation.w;
		float qx = rcv_msg->pose.pose.orientation.x;
		float qy = rcv_msg->pose.pose.orientation.y;
		float qz = rcv_msg->pose.pose.orientation.z;
		veh_heading_ = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
		odom_rcvd_ = true;
	}

	void LoadActorParams(){
		// determine if sim time will be used
		use_sim_time_ = this->get_parameter("use_sim_time").as_bool();
		dt_ = GetFloatParam("dt", 0.01f);

		// load parameters
	    std::vector<float> init_pos = GetFloatArrayParam("initial_position", std::vector<float>(0));
		std::vector<float> init_ori = GetFloatArrayParam("initial_orientation", std::vector<float>(0));

		init_pose_.orientation.w = init_ori[0];
		init_pose_.orientation.x = init_ori[1];
		init_pose_.orientation.y = init_ori[2];
		init_pose_.orientation.z = init_ori[3];
		init_pose_.position.x = init_pos[0];
		init_pose_.position.y = init_pos[1];
		init_pose_.position.z = init_pos[2];

		std::vector<float> final_pos = GetFloatArrayParam("final_position", std::vector<float>(0));
		std::vector<float> final_ori = GetFloatArrayParam("final_orientation", std::vector<float>(0));

		final_pose_.orientation.w = final_ori[0];
		final_pose_.orientation.x = final_ori[1];
		final_pose_.orientation.y = final_ori[2];
		final_pose_.orientation.z = final_ori[3];
		final_pose_.position.x = final_pos[0];
		final_pose_.position.y = final_pos[1];
		final_pose_.position.z = final_pos[2];

		trigger_.type = GetStringParam("trigger.type", "none");
		trigger_.op = GetStringParam("trigger.operator", "none");
		trigger_.value = GetFloatParam("trigger.threshold", 0.0f);

		transition_time_ = GetFloatParam("transition_time", 0.0f);
		
	}

	void SetActorPose() {
		if (!trigger_.fired) {
			current_actor_pose_ = init_pose_;
			return;
		}
		double delta_t = this->get_clock()->now().seconds() - trigger_time_;
		if (delta_t >= transition_time_) {
			current_actor_pose_ = final_pose_;
			return;
		}
		double t = delta_t / transition_time_;
		current_actor_pose_.position = mavs_ros_utils::Lerp(init_pose_.position, final_pose_.position, t);
		current_actor_pose_.orientation = mavs_ros_utils::Slerp(init_pose_.orientation, final_pose_.orientation, t);
		return;
	}

	void TimerCallback(){
		
		// Evaluate event trigger
		if (odom_rcvd_) {
			if (!trigger_.fired) {
				trigger_.Evaluate(veh_x_, veh_y_, elapsed_time_);
				if (trigger_.fired) trigger_time_ = this->get_clock()->now().seconds();
			}
		}

		SetActorPose();

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

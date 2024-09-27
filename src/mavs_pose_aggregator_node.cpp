// c++ includes
#include <omp.h>
#include <unistd.h>
#include <map>
//ros includes
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/image.hpp"
// package includes
#include "mavs_ros2/mavs_ros_utils.h"
// mavs includes
#include "mavs_core/data_path.h"
#include "vehicles/rp3d_veh/mavs_rp3d_veh.h"
#include "raytracers/embree_tracer/embree_tracer.h"
#include "sensors/mavs_sensors.h"

std::map<std::string, geometry_msgs::msg::PoseArray > poses_map;

void PoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr rcv_msg){
	geometry_msgs::msg::PoseArray anim_poses = *rcv_msg;
	rclcpp::Time tnow(anim_poses.header.stamp.sec, anim_poses.header.stamp.nanosec);
	std::string key = anim_poses.header.frame_id;
	if (poses_map.count(key)>0 ){
		poses_map[key] = anim_poses;
	}
	else{
		poses_map.insert({key,anim_poses});	
	}
}

int main(int argc, char **argv){
	//- Create the node and subscribers ---//
	rclcpp::init(argc, argv);
    auto n = std::make_shared<rclcpp::Node>("mavs_pose_aggregator_node");

	int num_veh = 1;
	n->declare_parameter("num_vehicles", 1);
	if (n->has_parameter("num_vehicles")){
		num_veh = n->get_parameter("num_vehicles").as_int();
	}

	std::vector< rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr > anim_subs;
	for (int nv = 0; nv<num_veh;nv++){
		std::string topic_name = "/mavs"+mavs::utils::ToString(nv,3)+"/anim_poses";
		auto anim_sub = n->create_subscription<geometry_msgs::msg::PoseArray>(topic_name, num_veh+25, PoseCallback);
		anim_subs.push_back(anim_sub);
	}

	auto agg_poses_pub = n->create_publisher<geometry_msgs::msg::PoseArray>("all_poses_pub", 10);

	while (rclcpp::ok()){
		if ((int)poses_map.size()==num_veh){
			geometry_msgs::msg::PoseArray anim_poses;
			anim_poses.header.stamp = n->now();
			anim_poses.header.frame_id = "world";
			for (auto const& x : poses_map){
				for (int i=0;i<(int)x.second.poses.size();i++){
					anim_poses.poses.push_back(x.second.poses[i]);
				}
			}
			agg_poses_pub->publish(anim_poses);
		}
		rclcpp::spin_some(n);
	} //while ros OK

	return 0;
}

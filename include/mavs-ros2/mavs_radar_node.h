#ifndef MAVS_RADAR_NODE_H_
#define MAVS_RADAR_NODE_H_
// package includes
#include "mavs-ros2/mavs_ros_utils.h"
#include "mavs-ros2/mavs_sensor_node.h"
//ros includes
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
// mavs includes
#include "mavs_core/data_path.h"
#include "vehicles/rp3d_veh/mavs_rp3d_veh.h"
#include "raytracers/embree_tracer/embree_tracer.h"
#include "sensors/mavs_sensors.h"

class MavsRadarNode : public MavsSensorNode{
  public:
	MavsRadarNode(): MavsSensorNode(){
		lidar_ = NULL;

		LoadLidarParams();

		lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar", 10);

		timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0/update_rate_hz_)),std::bind(&MavsRadarNode::TimerCallback, this));

		dt_ = 1.0/update_rate_hz_;
	}

	~MavsRadarNode(){
		if (lidar_) delete lidar_;
	}

  private:
	// class member data
	mavs::sensor::lidar::Lidar *lidar_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
	bool register_points_;

	// class member functions
	void LoadLidarParams(){
		//Options are: HDL-32E', 'HDL-64E', 'M8','OS1', 'OS1-16', 'OS2', 'LMS-291', 'VLP-16', 'RS32
		std::string lidar_type = GetStringParam("lidar_type", "OS1");
		register_points_ = GetBoolParam("register_points",true);

		if (lidar_type == "HDL-32E"){
			lidar_ = new mavs::sensor::lidar::Hdl32E;
		}
		else if (lidar_type == "HDL-64E"){
			lidar_ = new mavs::sensor::lidar::Hdl64ESimple;
		}
		else if (lidar_type == "M8"){
			lidar_ = new mavs::sensor::lidar::MEight;
		}
		else if (lidar_type == "OS1"){
			lidar_ = new mavs::sensor::lidar::OusterOS1;	
		}
		else if (lidar_type == "OS1-16"){
			lidar_ = new mavs::sensor::lidar::OusterOS1_16;
		}
		else if (lidar_type == "OS2"){
			lidar_ = new mavs::sensor::lidar::OusterOS2;
		}
		else if (lidar_type == "LMS-291"){
			lidar_ = new mavs::sensor::lidar::Lms291_S05;
		}
		else if (lidar_type == "VLP-16"){
			lidar_ = new mavs::sensor::lidar::Vlp16;
		}
		else if (lidar_type == "RS32"){
			lidar_ = new mavs::sensor::lidar::Rs32;
		}
		else {
			std::cerr<<"WARNING: LIDAR TYPE "<<lidar_type<<" NOT RECOGNIZED, USING OS2"<<std::endl;
			lidar_ = new mavs::sensor::lidar::OusterOS2;
		}

		lidar_->SetRelativePose(glm::vec3(offset_[0], offset_[1], offset_[2]), glm::quat(relor_[0], relor_[1], relor_[2], relor_[3]));
	}

	void TimerCallback(){

		MavsSensorNode::TimerCallback();

		glm::vec3 pos(pose_.pose.pose.position.x, pose_.pose.pose.position.y, pose_.pose.pose.position.z);
		glm::quat ori(pose_.pose.pose.orientation.w, pose_.pose.pose.orientation.x, pose_.pose.pose.orientation.y, pose_.pose.pose.orientation.z);

		lidar_->SetPose(pos, ori);

		lidar_->Update(&env_, dt_);
		
		if (display_)lidar_->Display();

		mavs::PointCloud2 mavs_pc2;
		sensor_msgs::msg::PointCloud2 pc2;
		if (register_points_){
			mavs_pc2 = lidar_->GetPointCloud2Registered();
			pc2 = mavs_ros_utils::CopyFromMavsPc2(mavs_pc2); 
			pc2.header.frame_id = "map";
		}
		else{
			mavs_pc2 = lidar_->GetPointCloud2();
			pc2 = mavs_ros_utils::CopyFromMavsPc2(mavs_pc2);
			pc2.header.frame_id = "map";
		}

		lidar_pub_->publish(pc2);
    }

};

#endif
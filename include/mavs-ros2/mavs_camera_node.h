//ros includes
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

class MavsCameraNode : public MavsSensorNode{
  public:
	MavsCameraNode(): MavsSensorNode(){
		//std::cout<<"Creating camera node "<<std::endl;
		cam = NULL;
		LoadCameraParams();

		camera_pub = this->create_publisher<sensor_msgs::msg::Image>("camera", 10);

		timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0/update_rate_hz)),std::bind(&MavsCameraNode::TimerCallback, this));
	}


	~MavsCameraNode(){
		if (cam) delete cam;
	}

  private:

	void LoadCameraParams(){
		std::string camera_type = GetStringParam("camera_type", "rgb");
		int nx = GetIntParam("num_horizontal_pix", 256);
		int ny = GetIntParam("num_vertical_pix", 256);
		float px = GetFloatParam("horizontal_pixel_plane_size", 0.0035f);
		float py = GetFloatParam("vertical_pixel_plane_size", 0.0035f);
		float flen = GetFloatParam("focal_length", 0.0035f);
		bool render_shadows = GetBoolParam("render_shadows", true);
		
		if (camera_type == "rgb"){
			cam = new mavs::sensor::camera::RgbCamera;
		}
		else if (camera_type == "rccb"){
			cam = new mavs::sensor::camera::RccbCamera;
		}
		else if (camera_type == "fisheye"){
			cam = new mavs::sensor::camera::RgbCamera;
		}
		else if (camera_type == "nir"){
			cam = new mavs::sensor::camera::RgbCamera;	
		}
		else if (camera_type == "lwir"){
			cam = new mavs::sensor::camera::RgbCamera;
		}
		else {
			std::cerr<<"WARNING: CAMERA TYPE "<<camera_type<<" NOT RECOGNIZED, USING RGB"<<std::endl;
			cam = new mavs::sensor::camera::RgbCamera;
		}
		cam->Initialize(nx,ny,px,py,flen);
		cam->SetRelativePose(glm::vec3(offset[0], offset[1], offset[2]), glm::quat(relor[0], relor[1], relor[2], relor[3]));
		cam->SetRenderShadows(render_shadows);
	}

	mavs::sensor::camera::Camera *cam;

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_pub; // = n->create_publisher<sensor_msgs::msg::Image>("camera", 10);

	void TimerCallback(){
        //RCLCPP_INFO(this->get_logger(), "MAVS Camera class callback");
		MavsSensorNode::TimerCallback();

		glm::vec3 pos(pose_.pose.pose.position.x, pose_.pose.pose.position.y, pose_.pose.pose.position.z);
		glm::quat ori(pose_.pose.pose.orientation.w, pose_.pose.pose.orientation.x, pose_.pose.pose.orientation.y, pose_.pose.pose.orientation.z);

		cam->SetPose(pos, ori);

		cam->Update(&env, dt);
		
		if (display)cam->Display();

		sensor_msgs::msg::Image img;
		mavs::Image mavs_img = cam->GetRosImage();
		mavs_ros_utils::CopyFromMavsImage(img, mavs_img);
		camera_pub->publish(img);
    }

};

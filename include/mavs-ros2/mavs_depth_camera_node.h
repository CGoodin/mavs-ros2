#ifndef MAVS_DEPTH_CAMERA_NODE_H_
#define MAVS_DEPTH_CAMERA_NODE_H_
// package includes
#include "mavs-ros2/mavs_ros_utils.h"
#include "mavs-ros2/mavs_sensor_node.h"
//ros includes
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
// mavs includes
#include "mavs_core/data_path.h"
#include "vehicles/rp3d_veh/mavs_rp3d_veh.h"
#include "raytracers/embree_tracer/embree_tracer.h"
#include "sensors/mavs_sensors.h"

class MavsDepthCameraNode : public MavsSensorNode{
  public:
	MavsDepthCameraNode(): MavsSensorNode(){

        frame_num_ = 0;
		LoadCameraParams();

		left_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("left/image_raw", 10);
		right_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("right/image_raw", 10);

		left_camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", 10);
		right_camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", 10);

		timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0/update_rate_hz_)),std::bind(&MavsDepthCameraNode::TimerCallback, this));
	}

	~MavsDepthCameraNode(){
		
	}

  private:
	// class member data
	mavs::sensor::camera::RgbCamera left_cam_, right_cam_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_, right_image_pub_;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_camera_info_pub_, right_camera_info_pub_;
	sensor_msgs::msg::CameraInfo left_cam_info_, right_cam_info_;
    int frame_num_;
    bool save_images_;

	// class member functions
	void LoadCameraParams(){
		std::string camera_type = GetStringParam("camera_type", "rgb");
		int nx = GetIntParam("num_horizontal_pix", 256);
		int ny = GetIntParam("num_vertical_pix", 256);
		float px = GetFloatParam("horizontal_pixel_plane_size", 0.0035f);
		float py = GetFloatParam("vertical_pixel_plane_size", 0.0035f);
		float flen = GetFloatParam("focal_length", 0.0035f);
		bool render_shadows = GetBoolParam("render_shadows", true);
        save_images_ = GetBoolParam("save_images", false);
		float baseline = GetFloatParam("baseline", 0.075f);
		
		left_cam_.Initialize(nx,ny,px,py,flen);
		left_cam_.SetRelativePose(glm::vec3(offset_[0], offset_[1]+0.5*baseline, offset_[2]), glm::quat(relor_[0], relor_[1], relor_[2], relor_[3]));
		left_cam_.SetRenderShadows(render_shadows);
		left_cam_.SetName("Left Camera");
		right_cam_.Initialize(nx, ny, px, py, flen);
		right_cam_.SetRelativePose(glm::vec3(offset_[0], offset_[1] - 0.5 * baseline, offset_[2]), glm::quat(relor_[0], relor_[1], relor_[2], relor_[3]));
		right_cam_.SetRenderShadows(render_shadows);
		right_cam_.SetName("Right Camera");

		float pixdim = px / (float)nx;
		left_cam_info_.height = ny;
		left_cam_info_.width = nx;
		float flen_pixels = flen / pixdim;
		float cx = 0.5f * nx;
		float cy = 0.5f * ny;
		float baseline_pixels = baseline / pixdim;
		float fx = flen_pixels;
		float fy = flen_pixels;

		left_cam_info_.header.frame_id = "stereo_right_optical_frame";
		left_cam_info_.d = { 0.0, 0.0, 0.0, 0.0, 0.0 };
		left_cam_info_.r = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
		//left_cam_info_.k = { flen/pixdim, px / pixdim, 0.0, 0.0, flen / pixdim, py / pixdim, 0.0, 0.0, 1.0 };
		left_cam_info_.k = { fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0 };
		//left_cam_info_.p = { flen / pixdim, 0.0, px / pixdim, 0.0, 0.0, flen / pixdim, py / pixdim, 0.5*baseline / pixdim, 0.0, 0.0, 1.0, 0.0};
		left_cam_info_.p = { fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0 };
		left_cam_info_.distortion_model = "plumb_bob";

		right_cam_info_.header.frame_id = "stereo_right_optical_frame";
		right_cam_info_.height = ny;
		right_cam_info_.width = nx;
		right_cam_info_.d = { 0.0, 0.0, 0.0, 0.0, 0.0 };
		right_cam_info_.r = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };
		//right_cam_info_.k = { flen / pixdim, px / pixdim, 0.0, 0.0, flen / pixdim, py / pixdim, 0.0, 0.0, 1.0 };
		right_cam_info_.k = { fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0 };
		//right_cam_info_.p = { flen / pixdim, 0.0, px / pixdim, 0.0, 0.0, flen / pixdim, py / pixdim, -0.5 * baseline / pixdim, 0.0, 0.0, 1.0, 0.0 };
		right_cam_info_.p = { fx, 0.0, cx, -fx * baseline_pixels, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0 };
		right_cam_info_.distortion_model = "plumb_bob";
	}

	void TimerCallback(){

		MavsSensorNode::TimerCallback();

		glm::vec3 pos(pose_.pose.pose.position.x, pose_.pose.pose.position.y, pose_.pose.pose.position.z);
		glm::quat ori(pose_.pose.pose.orientation.w, pose_.pose.pose.orientation.x, pose_.pose.pose.orientation.y, pose_.pose.pose.orientation.z);

		left_cam_.SetPose(pos, ori);
		left_cam_.Update(&env_, dt_);
		right_cam_.SetPose(pos, ori);
		right_cam_.Update(&env_, dt_);
		
		if (display_) {
			left_cam_.Display();
			right_cam_.Display();
		}
        if (save_images_){
            std::string left_fname = mavs::utils::ToString(frame_num_,5)+"_left_image.bmp";
            left_cam_.SaveImage(left_fname);
			std::string right_fname = mavs::utils::ToString(frame_num_, 5) + "_right_image.bmp";
			right_cam_.SaveImage(right_fname);
        }
		
		sensor_msgs::msg::Image left_img;
		mavs::Image left_mavs_img = left_cam_.GetRosImage();
		mavs_ros_utils::CopyFromMavsImage(left_img, left_mavs_img);
		left_image_pub_->publish(left_img);
		sensor_msgs::msg::Image right_img;
		mavs::Image right_mavs_img = right_cam_.GetRosImage();
		mavs_ros_utils::CopyFromMavsImage(right_img, right_mavs_img);
		right_image_pub_->publish(right_img);

		left_camera_info_pub_->publish(left_cam_info_);
		right_camera_info_pub_->publish(right_cam_info_);

        frame_num_++;
    }

};

#endif

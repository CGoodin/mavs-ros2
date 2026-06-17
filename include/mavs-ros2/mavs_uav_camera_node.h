#ifndef MAVS_CAMERA_NODE_H_
#define MAVS_CAMERA_NODE_H_
// package includes
#include "mavs-ros2/mavs_ros_utils.h"
#include "mavs-ros2/mavs_sensor_node.h"
#include "mavs-ros2/mavs_trajectory_smoothing.h"
//ros includes
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
// mavs includes
#include "mavs_core/data_path.h"
#include "vehicles/rp3d_veh/mavs_rp3d_veh.h"
#include "raytracers/embree_tracer/embree_tracer.h"
#include "sensors/mavs_sensors.h"

std::vector<glm::vec2> positions;
std::vector<float> headings;

static float HeadingFromQuaternion(glm::quat q) {
	// Yaw (heading) around the Z-axis
	float siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
	float cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
	return std::atan2(siny_cosp, cosy_cosp);
}

class MavsUavCameraNode : public MavsSensorNode {
public:
	MavsUavCameraNode() : MavsSensorNode() {
		cam_ = NULL;
		frame_num_ = 0;
		LoadCameraParams();

		camera_pub_ = this->create_publisher<sensor_msgs::msg::Image>("uav_camera", 10);

		timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0/update_rate_hz_)),std::bind(&MavsUavCameraNode::TimerCallback, this));

	}

	~MavsUavCameraNode() {
		if (cam_) delete cam_;
	}

private:
	// class member data
	mavs::sensor::camera::Camera* cam_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_pub_;
	int frame_num_;
	bool save_images_;
	float altitude_;

	// class member functions
	void LoadCameraParams() {
		std::string camera_type = GetStringParam("camera_type", "rgb");
		int nx = GetIntParam("num_horizontal_pix", 256);
		int ny = GetIntParam("num_vertical_pix", 256);
		float px = GetFloatParam("horizontal_pixel_plane_size", 0.0035f);
		float py = GetFloatParam("vertical_pixel_plane_size", 0.0035f);
		float flen = GetFloatParam("focal_length", 0.0035f);
		altitude_ = GetFloatParam("altitude", 100.0f);
		bool render_shadows = GetBoolParam("render_shadows", true);
		save_images_ = GetBoolParam("save_images", false);

		if (camera_type == "rgb") {
			cam_ = new mavs::sensor::camera::RgbCamera;
		}
		else if (camera_type == "rccb") {
			cam_ = new mavs::sensor::camera::RccbCamera;
		}
		else if (camera_type == "fisheye") {
			cam_ = new mavs::sensor::camera::RgbCamera;
		}
		else if (camera_type == "nir") {
			cam_ = new mavs::sensor::camera::RgbCamera;
		}
		else if (camera_type == "lwir") {
			cam_ = new mavs::sensor::camera::RgbCamera;
		}
		else {
			std::cerr << "WARNING: CAMERA TYPE " << camera_type << " NOT RECOGNIZED, USING RGB" << std::endl;
			cam_ = new mavs::sensor::camera::RgbCamera;
		}
		cam_->Initialize(nx, ny, px, py, flen);
		cam_->SetRelativePose(glm::vec3(offset_[0], offset_[1], offset_[2]), glm::quat(relor_[0], relor_[1], relor_[2], relor_[3]));
		cam_->SetRenderShadows(render_shadows);
	}

	void TimerCallback() {

		MavsSensorNode::TimerCallback();

		glm::vec3 p(pose_.pose.pose.position.x, pose_.pose.pose.position.y, pose_.pose.pose.position.z);
		glm::quat q(pose_.pose.pose.orientation.w, pose_.pose.pose.orientation.x, pose_.pose.pose.orientation.y, pose_.pose.pose.orientation.z);
		float heading = HeadingFromQuaternion(q);
		positions.push_back(glm::vec2(p.x, p.y));
		headings.push_back(heading);
		if (positions.size() > 100) {
			positions.erase(positions.begin());
			headings.erase(headings.begin());
		}
		trajectory::Prediction pred;
		if (positions.size() >= 50) {
			std::vector<glm::vec2> old_pos(positions.begin(), positions.begin() + 50);
			std::vector<float> old_heading(headings.begin(), headings.begin() + 50);
			pred = trajectory::SmoothAndPredict(old_pos, old_heading, 2.5f, 50);
		}
		else {
			pred.position = p;
			pred.heading = heading;
		}

		glm::vec3 pos(pred.position.x, pred.position.y, altitude_);
		glm::quat yaw(cosf(0.5f * pred.heading), 0.0f, 0.0f, sinf(0.5f * pred.heading));
		//glm::quat tiltDown = glm::angleAxis(glm::half_pi<float>(), glm::vec3(0.0f, 1.0f, 0.0f));
		glm::quat ori(cosf(0.25f * 3.14159f), 0.0f, sinf(0.25f * 3.14159f), 0.0f);
		ori = yaw * ori;
		cam_->SetPose(pos, ori);

		cam_->Update(&env_, dt_);

		if (display_)cam_->Display();
		if (save_images_) {
			std::string fname = mavs::utils::ToString(frame_num_, 5) + "_image.bmp";
			cam_->SaveImage(fname);
		}
		sensor_msgs::msg::Image img;
		mavs::Image mavs_img = cam_->GetRosImage();
		mavs_ros_utils::CopyFromMavsImage(img, mavs_img);

		img.header.stamp = this->get_clock()->now();
		img.header.frame_id = "base_link";

		camera_pub_->publish(img);
		frame_num_++;
	}

};

#endif

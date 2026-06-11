#ifndef MAVS_CAMERA_NODE_H_
#define MAVS_CAMERA_NODE_H_

#include "mavs-ros2/mavs_ros_utils.h"
#include "mavs-ros2/mavs_sensor_node.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "mavs_core/data_path.h"
#include "vehicles/rp3d_veh/mavs_rp3d_veh.h"
#include "raytracers/embree_tracer/embree_tracer.h"
#include "sensors/mavs_sensors.h"

class MavsCameraNode : public MavsSensorNode {
public:
    MavsCameraNode() : MavsSensorNode() {
        cam_ = NULL;
        frame_num_ = 0;
        LoadCameraParams();
        camera_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera", 10);
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
        BuildCameraInfoMessage();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(1000.0 / update_rate_hz_)),
            std::bind(&MavsCameraNode::TimerCallback, this)
        );
    }

    ~MavsCameraNode() {
        if (cam_) delete cam_;
    }

private:
    mavs::sensor::camera::Camera* cam_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;
    int frame_num_;
    bool save_images_;
    // stored for CameraInfo construction
    int nx_, ny_;
    float px_, py_, flen_;

    void LoadCameraParams() {
        std::string camera_type = GetStringParam("camera_type", "rgb");
        nx_ = GetIntParam("num_horizontal_pix", 256);
        ny_ = GetIntParam("num_vertical_pix", 256);
        px_ = GetFloatParam("horizontal_pixel_plane_size", 0.0035f);
        py_ = GetFloatParam("vertical_pixel_plane_size", 0.0035f);
        flen_ = GetFloatParam("focal_length", 0.0035f);
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

        cam_->Initialize(nx_, ny_, px_, py_, flen_);
        cam_->SetRelativePose(
            glm::vec3(offset_[0], offset_[1], offset_[2]),
            glm::quat(relor_[0], relor_[1], relor_[2], relor_[3])
        );
        cam_->SetRenderShadows(render_shadows);
    }

    void BuildCameraInfoMessage() {
        // Image dimensions
        camera_info_msg_.width = static_cast<uint32_t>(nx_);
        camera_info_msg_.height = static_cast<uint32_t>(ny_);

        // Focal lengths in pixels
        // fx = f / px  (focal_length / horizontal pixel size)
        // fy = f / py  (focal_length / vertical pixel size)
        double fx = static_cast<double>(flen_) / static_cast<double>(px_);
        double fy = static_cast<double>(flen_) / static_cast<double>(py_);

        // Principal point at image center
        double cx = nx_ / 2.0;
        double cy = ny_ / 2.0;

        // Distortion model — no distortion coefficients available from MAVS params,
        // so plumb_bob with all zeros is the correct declaration for a pinhole camera.
        camera_info_msg_.distortion_model = "plumb_bob";
        camera_info_msg_.d = { 0.0, 0.0, 0.0, 0.0, 0.0 };

        // Intrinsic matrix K (3x3, row-major):
        //  [ fx   0  cx ]
        //  [  0  fy  cy ]
        //  [  0   0   1 ]
        camera_info_msg_.k = {
            fx,  0.0, cx,
            0.0, fy,  cy,
            0.0, 0.0, 1.0
        };

        // Rectification matrix R — identity for a single, unrectified camera
        camera_info_msg_.r = {
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        };

        // Projection matrix P (3x4, row-major):
        //  [ fx   0  cx  0 ]
        //  [  0  fy  cy  0 ]
        //  [  0   0   1  0 ]
        // Tx/Ty are zero — no stereo baseline.
        camera_info_msg_.p = {
            fx,  0.0, cx,  0.0,
            0.0, fy,  cy,  0.0,
            0.0, 0.0, 1.0, 0.0
        };

        camera_info_msg_.header.frame_id = "camera_optical_frame";
    }

    void TimerCallback() {
        if (!cam_) return;

        MavsSensorNode::TimerCallback();

        glm::vec3 pos(
            pose_.pose.pose.position.x,
            pose_.pose.pose.position.y,
            pose_.pose.pose.position.z
        );
        glm::quat ori(
            pose_.pose.pose.orientation.w,
            pose_.pose.pose.orientation.x,
            pose_.pose.pose.orientation.y,
            pose_.pose.pose.orientation.z
        );

        cam_->SetPose(pos, ori);
        cam_->Update(&env_, dt_);
        if (display_) cam_->Display();

        if (save_images_) {
            std::string fname = mavs::utils::ToString(frame_num_, 5) + "_image.bmp";
            cam_->SaveImage(fname);
        }

        auto stamp = this->now();

        // Publish image
        sensor_msgs::msg::Image img;
        img.header.stamp = stamp;
        img.header.frame_id = "camera_optical_frame";
        mavs::Image mavs_img = cam_->GetRosImage();
        mavs_ros_utils::CopyFromMavsImage(img, mavs_img);
        camera_pub_->publish(img);

        // Publish camera info with matching stamp
        camera_info_msg_.header.stamp = stamp;
        camera_info_pub_->publish(camera_info_msg_);

        frame_num_++;
    }
};

#endif
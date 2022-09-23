// c++ includes
#include <omp.h>
#include <unistd.h>
//ros includes
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/image.hpp"
// package includes
#include "mavs_ros_utils.h"
// mavs includes
#include "mavs_core/data_path.h"
#include "vehicles/rp3d_veh/mavs_rp3d_veh.h"
#include "raytracers/embree_tracer/embree_tracer.h"
#include "sensors/mavs_sensors.h"

nav_msgs::msg::Odometry pose;
geometry_msgs::msg::PoseArray tire_poses;

void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr rcv_msg){
	pose = *rcv_msg;
}

void TireCallback(const geometry_msgs::msg::PoseArray::SharedPtr rcv_msg){
	tire_poses = *rcv_msg;
}

int main(int argc, char **argv){
	//- Create the node and subscribers ---//
	rclcpp::init(argc, argv);
    auto n = std::make_shared<rclcpp::Node>("mavs_camera_node");

	auto odom_sub = n->create_subscription<nav_msgs::msg::Odometry>("odometry_true", 10, OdomCallback);
	auto tire_sub = n->create_subscription<geometry_msgs::msg::PoseArray>("tire_poses", 10, TireCallback);
	auto camera_pub = n->create_publisher<sensor_msgs::msg::Image>("camera", 10);

	// load parameters
	std::string scene_file = mavs_ros_utils::GetStringParam(n, "scene_file", "cube_scene.json");
	std::string rp3d_vehicle_file = mavs_ros_utils::GetStringParam(n, "rp3d_vehicle_file", "l200.json");
	// camera types are "rgb", "rccb", "fisheye", "nir", and "lwir"
	std::string camera_type = mavs_ros_utils::GetStringParam(n, "camera_type", "rgb");
	int nx = mavs_ros_utils::GetIntParam(n, "num_horizontal_pix", 256);
	int ny = mavs_ros_utils::GetIntParam(n, "num_vertical_pix", 256);
	float px = mavs_ros_utils::GetFloatParam(n, "horizontal_pixel_plane_size", 0.0035f);
	float py = mavs_ros_utils::GetFloatParam(n, "vertical_pixel_plane_size", 0.0035f);
	float flen = mavs_ros_utils::GetFloatParam(n, "focal_length", 0.0035f);
	std::vector<float> offset = mavs_ros_utils::GetFloatArrayParam(n,"offset",std::vector<float>(0));
	std::vector<float> relor = mavs_ros_utils::GetFloatArrayParam(n,"orientation",std::vector<float>(0));
	bool render_shadows = mavs_ros_utils::GetBoolParam(n, "render_shadows", true);
	bool display = mavs_ros_utils::GetBoolParam(n, "display", true);
	float update_rate_hz = mavs_ros_utils::GetFloatParam(n, "update_rate_hz", 10.0f);

	mavs::sensor::camera::Camera *cam;
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

	mavs::MavsDataPath mdp;
	std::string mavs_data_path = mdp.GetPath();
	mavs::environment::Environment env;
	mavs::raytracer::embree::EmbreeTracer scene;
	scene.Load(mavs_data_path+"/scenes/"+scene_file);
	scene.TurnOffLabeling();
	env.SetRaytracer(&scene);

	mavs::vehicle::Rp3dVehicle mavs_veh;
	mavs_veh.Load(mavs_data_path+"/vehicles/rp3d_vehicles/"+rp3d_vehicle_file);
	mavs_veh.SetPosition(-10000.0f, -10000.0f, -10000.0f);
	mavs_veh.SetOrientation(1.0f, 0.0f, 0.0f, 0.0f);
	mavs_veh.Update(&env, 0.0, 0.0, 0.0, 0.00001);

	rclcpp::Rate rate(update_rate_hz);
	float dt = 1.0f/update_rate_hz;

	while (rclcpp::ok()){

		glm::vec3 pos(pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z);
		glm::quat ori(pose.pose.pose.orientation.w, pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z);
		env.SetActorPosition(0, pos, ori, dt, true);

		if (env.GetNumberOfActors()>=(int)(tire_poses.poses.size()+1)){
			for (int i=0;i<(int)tire_poses.poses.size();i++){
				glm::vec3 tpos(tire_poses.poses[i].position.x, tire_poses.poses[i].position.y, tire_poses.poses[i].position.z);
				glm::quat tori(tire_poses.poses[i].orientation.w, tire_poses.poses[i].orientation.x, tire_poses.poses[i].orientation.y, tire_poses.poses[i].orientation.z);
				env.SetActorPosition(i+1, tpos, tori, dt, true);
			}
		}

		cam->SetPose(pos, ori);

		cam->Update(&env, dt);
		
		if (display)cam->Display();

		sensor_msgs::msg::Image img;
		mavs::Image mavs_img = cam->GetRosImage();
		mavs_ros_utils::CopyFromMavsImage(img, mavs_img);
		camera_pub->publish(img);

		rate.sleep();

		rclcpp::spin_some(n);
	} //while ros OK
	delete cam;
	return 0;
}

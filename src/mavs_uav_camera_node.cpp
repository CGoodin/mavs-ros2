// package includes
#include "mavs-ros2/mavs_uav_camera_node.h"

int main(int argc, char **argv){

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MavsUavCameraNode>());
  	rclcpp::shutdown();

	return 0;
}

// package includes
#include "mavs-ros2/mavs_gps_node.h"

int main(int argc, char **argv){

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MavsGpsNode>());
  	rclcpp::shutdown();

	return 0;
}

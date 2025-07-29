// package includes
#include "mavs-ros2/mavs_radar_node.h"

int main(int argc, char **argv){

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MavsRadarNode>());
  	rclcpp::shutdown();

	return 0;
}

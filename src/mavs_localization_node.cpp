// package includes
#include "mavs_ros2/mavs_localization_node.h"

int main(int argc, char **argv){

	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MavsLocalizationNode>());
  	rclcpp::shutdown();

	return 0;
}

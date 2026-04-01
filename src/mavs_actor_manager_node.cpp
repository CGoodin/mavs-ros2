// package includes
#include "mavs-ros2/mavs_actor_manager_node.h"

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MavsActorManagerNode>());
  	rclcpp::shutdown();
	return 0;
}

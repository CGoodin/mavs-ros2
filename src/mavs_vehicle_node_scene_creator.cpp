// package includes
#include "mavs-ros2/mavs_vehicle_node_scene_creator.h"

int main(int argc, char **argv){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MavsVehicleNodeSceneCreator>());
  	rclcpp::shutdown();
	return 0;
}

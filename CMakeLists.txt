cmake_minimum_required(VERSION 3.8)
project(mavs-ros2)

add_definitions(-DUSE_OMP)
add_definitions(-DUSE_EMBREE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(MAVS REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(X11 REQUIRED)
find_package(OpenMP)

# Include directories
include_directories(include)

ament_export_include_directories(include)

# define a library target called my_value_converter_library
add_library(mavs-ros2 src/mavs_ros_utils.cpp)
ament_target_dependencies(mavs-ros2 rclcpp std_msgs geometry_msgs nav_msgs tf2 sensor_msgs tf2_ros tf2_msgs tf2_geometry_msgs visualization_msgs rosgraph_msgs)

# this line to exports the library
ament_export_targets(mavs-ros2 HAS_LIBRARY_TARGET)

# install the include/my_cpp_library directory to the install/include/my_cpp_library
install(
  TARGETS mavs-ros2
  EXPORT mavs-ros2
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Add executables
add_executable(mavs_vehicle_node src/mavs_vehicle_node.cpp)
target_link_libraries(mavs_vehicle_node
  X11
  gomp
  MAVS::mavs 
  ${EMBREE_LIBRARY}
  mavs-ros2
)
ament_target_dependencies(mavs_vehicle_node rclcpp std_msgs geometry_msgs nav_msgs tf2 sensor_msgs tf2_ros tf2_msgs tf2_geometry_msgs visualization_msgs rosgraph_msgs)

add_executable(mavs_camera_node src/mavs_camera_node.cpp)
target_link_libraries(mavs_camera_node
  X11
  gomp
  MAVS::mavs 
  ${EMBREE_LIBRARY}
  mavs-ros2
)
ament_target_dependencies(mavs_camera_node rclcpp std_msgs geometry_msgs nav_msgs tf2 sensor_msgs tf2_ros tf2_msgs tf2_geometry_msgs visualization_msgs rosgraph_msgs)

add_executable(mavs_lidar_node src/mavs_lidar_node.cpp)
target_link_libraries(mavs_lidar_node
  X11
  gomp
  MAVS::mavs 
  ${EMBREE_LIBRARY}
  mavs-ros2
)
ament_target_dependencies(mavs_lidar_node rclcpp std_msgs geometry_msgs nav_msgs tf2 sensor_msgs tf2_ros tf2_msgs tf2_geometry_msgs visualization_msgs rosgraph_msgs)

add_executable(mavs_localization_node src/mavs_localization_node.cpp)
target_link_libraries(mavs_localization_node
  X11
  gomp
  MAVS::mavs 
  ${EMBREE_LIBRARY}
  mavs-ros2
)
ament_target_dependencies(mavs_localization_node rclcpp std_msgs geometry_msgs nav_msgs tf2 sensor_msgs tf2_ros tf2_msgs tf2_geometry_msgs visualization_msgs rosgraph_msgs)


add_executable(mavs_pose_aggregator_node src/mavs_pose_aggregator_node.cpp)
target_link_libraries(mavs_pose_aggregator_node
  X11
  gomp
  MAVS::mavs 
  ${EMBREE_LIBRARY}
  mavs-ros2
)
ament_target_dependencies(mavs_pose_aggregator_node rclcpp std_msgs geometry_msgs nav_msgs tf2 sensor_msgs tf2_ros tf2_msgs tf2_geometry_msgs visualization_msgs rosgraph_msgs)


#add_library(mavs-ros2 /include/mavs-ros2/mavs_node.h /include/mavs-ros2/mavs_ros_utils.h)

install(TARGETS
  mavs_vehicle_node
  mavs_camera_node
  mavs_lidar_node
  mavs_localization_node
  mavs_pose_aggregator_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE)

# Install the export file
install(
  EXPORT mavs-ros2
  FILE mavs-ros2Targets.cmake
  NAMESPACE mavs-ros2::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_package()
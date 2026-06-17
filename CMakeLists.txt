cmake_minimum_required(VERSION 3.8)
project(mavs-ros2)

# --- 1. Global Definitions ---
add_definitions(-DUSE_OMP)
add_definitions(-DUSE_EMBREE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 2. Find Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(MAVS REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(radar_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(X11 REQUIRED)
find_package(OpenMP REQUIRED)

# --- 3. Fix RPATH ---
set(CMAKE_INSTALL_RPATH "/workspace/mavs/install/lib64")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# --- 4. Define Utility Library ---
add_library(mavs-ros2 src/mavs_ros_utils.cpp)

# Using PUBLIC ensures all nodes inherit the MAVS headers
target_include_directories(mavs-ros2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /workspace/mavs/install/include
)

ament_target_dependencies(mavs-ros2
  rclcpp std_msgs geometry_msgs nav_msgs tf2 sensor_msgs tf2_ros tf2_msgs
  tf2_geometry_msgs visualization_msgs rosgraph_msgs radar_msgs
)

# --- 5. Define Executables ---
set(MAVS_NODES
  mavs_vehicle_node
  mavs_actor_manager_node
  mavs_camera_node
  mavs_uav_camera_node
  mavs_depth_camera_node
  mavs_lidar_node
  mavs_radar_node
  mavs_gps_node
  mavs_localization_node
  mavs_pose_aggregator_node
  mavs_vehicle_node_scene_creator
)

foreach(node ${MAVS_NODES})
  add_executable(${node} src/${node}.cpp)
  target_link_libraries(${node}
    X11
    OpenMP::OpenMP_CXX
    MAVS::mavs
    mavs-ros2
  )
  ament_target_dependencies(${node}
    rclcpp std_msgs geometry_msgs nav_msgs tf2 sensor_msgs tf2_ros tf2_msgs
    tf2_geometry_msgs visualization_msgs rosgraph_msgs radar_msgs
  )
endforeach()

# --- 6. Install Logic ---
install(TARGETS mavs-ros2 ${MAVS_NODES}
  EXPORT mavs-ros2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE
)

# --- 7. Export and Package ---
ament_export_include_directories(include)
ament_export_targets(mavs-ros2 HAS_LIBRARY_TARGET)

install(
  EXPORT mavs-ros2
  FILE mavs-ros2Targets.cmake
  NAMESPACE mavs-ros2::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_package()
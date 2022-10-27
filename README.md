# mavs-ros2
ROS-2 interfaces to the MSU Autonomous Vehicle Simulator (MAVS).

MAVS is an open, flexible simulation library for autonomous ground vehicles that features tools for off-road environments.

[Request MAVS access](https://www.cavs.msstate.edu/capabilities/mavs_request.php)

MAVS is a stand-alone library that can be build on Windows or Linux/Unix. The MAVS-ROS2 package provides a ROS2 interface to MAVS functions. These include:

* A Lidar simulation node: Simulates one of the following lidar systems:
    * Velodyne HDL-64E
    * Velodyne HDL-32E
    * Velodyne VLP-16
    * Ouster OS1-16
    * Ouster OS2-64
    * Quanergy M8
    * SICK LMS-291
* A camera simulation node: User defines the camera parameters like resolution and focal length.
* A vehicle simulation node: simulates driving and vehicle dynamics. User can manually drive the vehicle with the W-A-S-D keys.

To use the MAVS-ROS2 package, you will need to 
1. Install [MAVS](https://www.cavs.msstate.edu/capabilities/mavs_request.php) 
2. Install [ROS2](https://docs.ros.org/en/rolling/Installation.html) 
3. Have a working [ROS2 workspace](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).

After this, assuming your ROS-2 workspace is located in *~/ros2_ws/*, the MAVS-ROS2 package can be installed with the following steps:
```bash
$cd ~/ros2_ws/src
$git clone https://github.com/CGoodin/mavs-ros2.git
$cd ../
$colcon build
```
Once the build is complete, you will need to source the ROS2 setup file.
```bash
$source ~/ros2_ws/install/setup.bash
```

Finally, you can check the build by launching the driving example.
```bash
$ros2 launch mavs-ros2 launch_vehicle.py
```

A display window will appear with a vehicle in a simple scene. It can be driven with the W-A-S-D keys.

More information about the nodes and functions of the MAVS-ROS2 package can be found on the [wiki](https://github.com/CGoodin/mavs-ros2/wiki).

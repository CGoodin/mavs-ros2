
# -*- coding: utf-8 -*-
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # ----- Frame args (keep these consistent everywhere) -----
    parent = LaunchConfiguration('parent')               # e.g. base_link
    camera_frame = LaunchConfiguration('camera_frame')   # e.g. stereo_left_camera_frame
    optical_frame = LaunchConfiguration('optical_frame') # e.g. stereo_left_optical_frame

    # Optical rotation defaults: RPY = [-pi/2, 0, -pi/2]
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    scene_file = "cube_scene.json"
    veh_file = "mrzr4_tires_low_gear.json"
    env_params = {'env_params': {"rain_rate": 10.0, "snow_rate": 0.0, "year":2026, "month":1, "date":22, "hour":13, "minute":29, "second":12, "time_zone":6}}

    mavs_vehicle = launch_ros.actions.Node(
        package='mavs-ros2',
        executable='mavs_vehicle_node',
        name='mavs_vehicle_node',
        parameters=[
            {'scene_file': scene_file},
            {'rp3d_vehicle_file': veh_file},
            {'soil_strength': 250.0},
            {'surface_type': "dry"},
            {'Initial_X_Position': 0.0},
            {'Initial_Y_Position': 0.0},
            {'Initial_Heading': 0.0},
            {'debug_camera': True},
            {'use_human_driver': True},
            {'use_full_path': False},
            env_params
        ],
        output='screen',
        emulate_tty=True
    )

    mavs_depth_camera = launch_ros.actions.Node(
        package='mavs-ros2',
        executable='mavs_depth_camera_node',
        name='mavs_depth_camera_node',
        parameters=[
            {'scene_file': scene_file},
            {'vehicle_files': [veh_file]},
            {'camera_type': "rgb"},
            {'num_horizontal_pix': 320},
            {'num_vertical_pix': 200},
            {'horizontal_pixel_plane_size': 0.00384},
            {'vertical_pixel_plane_size': 0.0024},
            {'focal_length': 0.00235},
            {'baseline': 0.075},
            {'offset': [1.5, 0.0, 0.5]},
            {'orientation': [1.0, 0.0, 0.0, 0.0]},
            {'render_shadows': True},
            {'display': True},
            {'update_rate_hz': 10.0},
            {'use_full_path': False},
            env_params
        ],
        output='screen',
        emulate_tty=True
    )

    mavs_aggregator = launch_ros.actions.Node(
        package='mavs-ros2',
        executable='mavs_pose_aggregator_node',
        name='mavs_aggregator_node',
        parameters=[{'num_vehicles': 1}],
        remappings=[('/mavs000/anim_poses', '/mavs/anim_poses')],
        output='screen',
        emulate_tty=True
    )

    
    return LaunchDescription([
        # Frame args so you can override at launch
        DeclareLaunchArgument('parent', default_value='base_link'),
        DeclareLaunchArgument('camera_frame', default_value='stereo_left_camera_frame'),
        DeclareLaunchArgument('optical_frame', default_value='stereo_left_optical_frame'),
        DeclareLaunchArgument('roll',  default_value='-1.57079632679'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw',   default_value='-1.57079632679'),

        mavs_vehicle,
        mavs_depth_camera,
        mavs_aggregator,
    ])

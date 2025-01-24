from launch import LaunchDescription
import launch_ros.actions
import launch.event_handlers
import launch.events

def generate_launch_description():
    # some "global" parameters
    scene_file = "cube_scene.json"
    veh_file = "mrzr4_tires_low_gear.json"

    env_params = {'env_params':
	        {"rain_rate": 10.0,
                "snow_rate": 0.0}
            }

    mavs_vehicle = launch_ros.actions.Node(
        package='mavs-ros2',
        namespace='mavs',
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
            env_params
        ],
        output='screen',
        emulate_tty=True
    )

    mavs_camera = launch_ros.actions.Node(
        package='mavs-ros2',
        namespace='mavs',
        executable='mavs_camera_node',
        name='mavs_camera_node',
        parameters=[
            {'scene_file': scene_file},
             {'vehicle_files': [veh_file]},
            {'camera_type': "rgb"},
            {'num_horizontal_pix': 480},
            {'num_vertical_pix': 270},
            {'horizontal_pixel_plane_size': 0.006222},
            {'vertical_pixel_plane_size': 0.0035},
            {'focal_length': 0.0035},
            {'offset': [-10.0,0.0,1.0]},
            {'orientation': [1.0, 0.0, 0.0, 0.0]},
            {'render_shadows': True},
            {'display': True},
            {'update_rate_hz': 5.0},
            env_params
        ],
        output='screen',
        emulate_tty=True
    )

    mavs_aggregator = launch_ros.actions.Node(
        package='mavs-ros2',
        namespace='mavs',
        executable='mavs_pose_aggregator_node',
        name='mavs_aggregator_node',
        parameters=[
                {'num_vehicles': 1}
            ],
        remappings=[
                (('/mavs000/anim_poses'), '/mavs/anim_poses')
            ],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        mavs_vehicle,
        mavs_camera,
         mavs_aggregator
    ])

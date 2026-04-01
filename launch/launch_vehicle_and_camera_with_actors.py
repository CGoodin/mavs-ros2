from launch import LaunchDescription
import launch_ros.actions
import launch.event_handlers
import launch.events
import math

def generate_launch_description():
    # some "global" parameters
    scene_file = "odoa_scene_no_trees_no_obstacle.json"
    veh_file = "mrzr4_tires_low_gear.json"
    actor_files = ["pine_tree_actor.json"]
    num_actors = len(actor_files)
    env_params = {'env_params':
	        {"rain_rate": 0.0,
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
            {'Initial_X_Position': -50.0},
            {'Initial_Y_Position': 0.0},
            {'Initial_Heading': 0.0},
            {'debug_camera': True},
            {'use_human_driver': True},
            {'publish_imu': False},
            env_params
        ],
        output='screen',
        emulate_tty=True
    )
    
    mavs_actor_manager_node = launch_ros.actions.Node(
            package='mavs-ros2',
            namespace='mavs_actor', # namespace is how different 
            executable='mavs_actor_manager_node',
            name='mavs_actor_manager_node',
            parameters=[
                {'initial_position': [0.0, 10.0, 0.0]},
                {'initial_orientation': [1.0, 0.0, 0.0, 0.0]},
                {'final_position': [0.0, 10.0, 0.25]},
                {'final_orientation': [math.cos(0.25*math.pi), math.sin(0.25*math.pi), 0.0, 0.0]},
                {'transition_time': 1.0},
                {'trigger': {
                    "type": "x", # can be "x", "y", or "time"
                    "operator": ">",
                    "threshold": -25.0} 
                 }, # trigger
            ],
            remappings=[
                (('/mavs_actor/odometry'), '/mavs/odometry_true')    
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
             {'actor_files': [actor_files]},
            {'camera_type': "rgb"},
            {'num_horizontal_pix': 480},
            {'num_vertical_pix': 270},
            {'horizontal_pixel_plane_size': 0.006222},
            {'vertical_pixel_plane_size': 0.0035},
            {'focal_length': 0.0035},
            # if "fixed", the offset and orientation will be the absolute position
            # if "follow", the offset will be the position but the viewfinder will look at the vehicle
            # if "attached", the offset and orientation will be relative to the vehicle
            {'sensor_position_mode': "fixed"}, 
            {'offset': [0.0,-20.0,1.0]},
            {'orientation': [math.cos(0.25*math.pi), 0.0, 0.0, math.sin(0.25*math.pi)]},
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
                {'num_vehicles': 1,
                 'num_actors': num_actors,
                 }
            ],
        remappings=[
                (('/mavs000/anim_poses'), '/mavs/anim_poses'),
                (('/mavs000/actor_poses'), '/mavs_actor/actor_poses')    
            ],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        mavs_vehicle,
        mavs_camera,
        mavs_actor_manager_node,
        mavs_aggregator
    ])

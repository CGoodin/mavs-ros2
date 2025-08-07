from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions
import launch.event_handlers
import launch.events

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='mavs-ros2',
#             executable='mavs_vehicle_node_scene_creator',
#             name='scene_creator_node',
#             output='screen',
#             parameters=[{
#                 'mode': 'manual',
#                 'iterations': 100,
#                 'ditch_depth_start': 1.0,
#                 'ditch_width_start': 3.0,
#                 'results_path': '/scratch/cgoodin/results'
#             }]
#         )
#     ])

def generate_launch_description():
    # some "global" parameters
    scene_file = "cube_scene.json"
    veh_file = "mrzr4_tires_low_gear.json"

    env_params = {'env_params':
	        {"rain_rate": 10.0,
                "snow_rate": 0.0}
            }

    mavs_vehicle_scene_creator_node = launch_ros.actions.Node(
        package='mavs-ros2',
        namespace='mavs',
        executable='mavs_vehicle_node_scene_creator',
        name='scene_creator_node',
        parameters=[
            {'mode': 'manual'},
            {'iterations': 100},
            {'ditch_depth_start': 1.0},
            {'ditch_width_start': 3.0},
            {'results_path': '/scratch/cgoodin/results'},
            {'scene_file': scene_file},
            {'rp3d_vehicle_file': veh_file},
            {'soil_strength': 250.0},
            {'surface_type': "dry"},
            {'Initial_X_Position': 0.0},
            {'Initial_Y_Position': 0.0},
            {'Initial_Heading': 0.0},
            {'dt': 0.01},
            {'debug_camera': True},
            {'use_human_driver': True},
            env_params
        ],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        mavs_vehicle_scene_creator_node
    ])
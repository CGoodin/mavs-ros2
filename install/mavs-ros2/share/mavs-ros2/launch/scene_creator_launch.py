from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavs-ros2',
            executable='mavs_vehicle_node_scene_creator',
            name='scene_creator_node',
            output='screen',
            parameters=[{
                'mode': 'manual',
                'iterations': 100,
                'ditch_depth_start': 1.0,
                'ditch_width_start': 3.0,
                'results_path': '/home/jdj688/mavs_ros2/src/mavs-ros2/results'
            }]
        )
    ])


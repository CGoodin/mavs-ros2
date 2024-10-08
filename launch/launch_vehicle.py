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
        package='mavs_ros2',
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
            {'dt': 0.01},
            {'debug_camera': True},
            {'use_human_driver': True},
            env_params
        ],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        mavs_vehicle
    ])

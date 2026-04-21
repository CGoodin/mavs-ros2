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
            {'publish_imu': True},
            env_params
        ],
        output='screen',
        emulate_tty=True
    )

    mavs_gps = launch_ros.actions.Node(
            package='mavs-ros2',
            namespace='mavs',
            executable='mavs_gps_node',
            name='mavs_gps_node',
            parameters=[
                {'scene_file': scene_file},
                {'offset': [2.0,0.0,0.5]},
                {'orientation': [1.0, 0.0, 0.0, 0.0]},
                {'origin_lla': [33.475884, -88.787965, 70.0]},
                {'display': True},
                {'gps_type': "dual band"},
                {'update_rate_hz': 1.0},
                {'vehicle_files': [veh_file]},
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
    
    navsat_transform = launch_ros.actions.Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[{
            'magnetic_declination_radians': 0.0,  # adjust for your location
            'yaw_offset': 1.5708,  # offset between IMU and ENU frame
            #'swap_ne': True,
            'zero_altitude': True, # flatten Z to 0 in output
            'publish_filtered_gps': False,
            'use_odometry_yaw': False,
            'wait_for_datum': False,
            'broadcast_utm_transform': True,
            'broadcast_utm_transform_as_parent_frame': False,
        }],
        remappings=[
            ('imu', '/mavs/imu'),            # from mavs_vehicle_node
            ('gps/fix', '/mavs/gps_fix'),         # from mavs_gps_node
            #('odometry/filtered', '/odometry/filtered'),  # can be empty/unused if not running EKF
            #('odometry/gps', '/odometry/gps'),    # output: ENU odometry
            #('gps/filtered', '/gps/filtered'),    # output: filtered GPS (if publish_filtered_gps=True)
        ]
    )
    
    ekf_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[{
            'frequency': 30.0,
            'sensor_timeout': 0.1,
            'two_d_mode': True,
            'publish_tf': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'imu_link', # should be base link on a real robot
            'world_frame': 'odom',
            'imu0': '/mavs/imu',
            'imu0_config': [
                False, False, False,   # x, y, z position
                False, False, True,    # roll, pitch, yaw
                False, False, False,   # vx, vy, vz
                True,  True, True,   # vroll, vpitch, vyaw
                False,  False, False    # ax, ay, az
            ],
            'imu0_differential': False,
            'imu0_remove_gravitational_acceleration': False,
        }],
        remappings=[
            #('odometry/filtered', '/odometry/filtered'),
        ]
    )
    
    gps_static_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_static_tf',
        arguments=[
            '--x', '2.0',
            '--y', '0.0',
            '--z', '0.5',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'imu_link',
            '--child-frame-id', 'gps_link'
        ]
    )

    return LaunchDescription([
        mavs_vehicle,
        mavs_gps,
        mavs_aggregator,
        navsat_transform,
        ekf_node,
        gps_static_tf
    ])

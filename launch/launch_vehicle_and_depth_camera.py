
# -*- coding: utf-8 -*-
from launch import LaunchDescription
import launch_ros.actions
import launch.event_handlers
import launch.events
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    scene_file = "cube_scene.json"
    veh_file = "mrzr4_tires_low_gear.json"

    env_params = {'env_params': {"rain_rate": 10.0, "snow_rate": 0.0}}

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
            {'debug_camera': False},
            {'use_human_driver': True},
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
            {'num_horizontal_pix': 320},   # 1280
            {'num_vertical_pix': 200},     # 800
            {'horizontal_pixel_plane_size': 0.00384},
            {'vertical_pixel_plane_size': 0.0024},
            {'focal_length': 0.00235},
            {'baseline': 0.075},
            {'offset': [1.5, 0.0, 0.5]},
            {'orientation': [1.0, 0.0, 0.0, 0.0]},
            {'render_shadows': True},
            {'display': True},
            {'update_rate_hz': 10.0},
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
        remappings=[(('/mavs000/anim_poses'), '/mavs/anim_poses')],
        output='screen',
        emulate_tty=True
    )

    # --- Rectify both cameras via composition (unique names per component) ---
    # image_proc components: publish /{ns}/image_rect (mono) and /{ns}/image_rect_color (color).
    rectify_container = ComposableNodeContainer(
        name='image_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_left',      # unique component name
                namespace='left',         # topics under /left/...
                remappings=[
                    ('image', 'image_raw'),          # /left/image_raw
                    ('camera_info', 'camera_info'),  # /left/camera_info
                    # publishes /left/image_rect and /left/image_rect_color
                ],
            ),
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_right',     # unique component name
                namespace='right',        # topics under /right/...
                remappings=[
                    ('image', 'image_raw'),          # /right/image_raw
                    ('camera_info', 'camera_info'),  # /right/camera_info
                ],
            ),
        ],
    )

    stereo_disparity_node = launch_ros.actions.Node(
        package='stereo_image_proc', executable='disparity_node', name='disparity_node',
        output='screen',
        parameters=[{'approximate_sync': True}, {'queue_size': 10}, {'image_transport': 'raw'}],
        remappings=[
            ('left/image_rect', '/left/image_rect'),
            ('left/camera_info', '/left/camera_info'),
            ('right/image_rect', '/right/image_rect'),
            ('right/camera_info', '/right/camera_info'),
            ('disparity', '/stereo/disparity'),
        ],
    )

    stereo_pointcloud_node = launch_ros.actions.Node(
        package='stereo_image_proc', executable='point_cloud_node', name='point_cloud_node',
        output='screen',
        parameters=[{'approximate_sync': True}, {'queue_size': 10}, {'image_transport': 'raw'}, {'use_color': True}],
        remappings=[
            ('disparity', '/stereo/disparity'),
            ('left/image_rect_color', '/left/image_rect'),
            ('left/camera_info', '/left/camera_info'),
            ('right/camera_info', '/right/camera_info'),
            # Optionally: ('points2', '/stereo/points2'),
        ],
    )

    return LaunchDescription([
        mavs_vehicle,
        mavs_depth_camera,
        mavs_aggregator,
        rectify_container,
        stereo_disparity_node,
        stereo_pointcloud_node,
    ])



# # -*- coding: utf-8 -*-
# from launch import LaunchDescription
# import launch_ros.actions
# import launch.event_handlers
# import launch.events

# def generate_launch_description():
#     scene_file = "cube_scene.json"
#     veh_file = "mrzr4_tires_low_gear.json"

#     env_params = {'env_params': {"rain_rate": 10.0, "snow_rate": 0.0}}

#     mavs_vehicle = launch_ros.actions.Node(
#         package='mavs-ros2',
#         executable='mavs_vehicle_node',
#         name='mavs_vehicle_node',
#         parameters=[
#             {'scene_file': scene_file},
#             {'rp3d_vehicle_file': veh_file},
#             {'soil_strength': 250.0},
#             {'surface_type': "dry"},
#             {'Initial_X_Position': 0.0},
#             {'Initial_Y_Position': 0.0},
#             {'Initial_Heading': 0.0},
#             {'debug_camera': False},
#             {'use_human_driver': True},
#             env_params
#         ],
#         output='screen',
#         emulate_tty=True
#     )

#     mavs_depth_camera = launch_ros.actions.Node(
#         package='mavs-ros2',
#         executable='mavs_depth_camera_node',
#         name='mavs_depth_camera_node',
#         parameters=[
#             {'scene_file': scene_file},
#             {'vehicle_files': [veh_file]},
#             {'camera_type': "rgb"},
#             {'num_horizontal_pix': 320},   # 1280
#             {'num_vertical_pix': 200},     # 800
#             {'horizontal_pixel_plane_size': 0.00384},
#             {'vertical_pixel_plane_size': 0.0024},
#             {'focal_length': 0.00235},
#             {'baseline': 0.075},
#             {'offset': [1.5, 0.0, 0.5]},
#             {'orientation': [1.0, 0.0, 0.0, 0.0]},
#             {'render_shadows': True},
#             {'display': True},
#             {'update_rate_hz': 10.0},
#             env_params
#         ],
#         output='screen',
#         emulate_tty=True
#     )

#     mavs_aggregator = launch_ros.actions.Node(
#         package='mavs-ros2',
#         executable='mavs_pose_aggregator_node',
#         name='mavs_aggregator_node',
#         parameters=[{'num_vehicles': 1}],
#         remappings=[(('/mavs000/anim_poses'), '/mavs/anim_poses')],
#         output='screen',
#         emulate_tty=True
#     )

#     # --- Rectify both cameras ---
#     # IMPORTANT: Do NOT remap image_rect -> image_rect_color here.
#     # image_proc will publish image_rect (mono) and image_rect_color (color).
#     # Keep both available so disparity_node can consume mono and point_cloud_node can consume color.
#     left_rect = launch_ros.actions.Node(
#         package='image_proc',
#         executable='image_proc',
#         name='image_proc_left',
#         namespace='left',
#         remappings=[
#             ('image', 'image_raw'),          # expects /left/image_raw
#             ('camera_info', 'camera_info'),  # expects /left/camera_info
#             # ('image_rect', 'image_rect_color')  # <-- REMOVED: breaks mono rect topic
#         ],
#         arguments=['--ros-args', '-r', '__node:=image_proc_left'],
#         output='screen'
#     )

#     right_rect = launch_ros.actions.Node(
#         package='image_proc',
#         executable='image_proc',
#         name='image_proc_right',
#         namespace='right',
#         remappings=[
#             ('image', 'image_raw'),          # expects /right/image_raw
#             ('camera_info', 'camera_info'),  # expects /right/camera_info
#             # ('image_rect', 'image_rect_color')  # <-- REMOVED
#         ],
#         arguments=['--ros-args', '-r', '__node:=image_proc_right'],
#         output='screen'
#     )

    
#     stereo_disparity_node = launch_ros.actions.Node(
#         package='stereo_image_proc', executable='disparity_node', name='disparity_node',
#         output='screen',
#         parameters=[{'approximate_sync': True}, {'queue_size': 10}, {'image_transport': 'raw'}],
#         remappings=[
#             ('left/image_rect', '/left/image_rect'),
#             ('left/camera_info', '/left/camera_info'),
#             ('right/image_rect', '/right/image_rect'),
#             ('right/camera_info', '/right/camera_info'),
#             ('disparity', '/stereo/disparity'),
#         ],
#     )

#     stereo_pointcloud_node = launch_ros.actions.Node(
#         package='stereo_image_proc', executable='point_cloud_node', name='point_cloud_node',
#         output='screen',
#         parameters=[{'approximate_sync': True}, {'queue_size': 10}, {'image_transport': 'raw'}, {'use_color': True}],
#         remappings=[
#             ('disparity', '/stereo/disparity'),
#             ('left/image_rect_color', '/left/image_rect_color'),
#             ('left/camera_info', '/left/camera_info'),
#             ('right/camera_info', '/right/camera_info'),
#             # Optionally: ('points2', '/stereo/points2'),
#         ],
#     )

#     return LaunchDescription([
#         mavs_vehicle,
#         mavs_depth_camera,
#         mavs_aggregator,
#         left_rect,
#         right_rect,
#         stereo_disparity_node,
#         stereo_pointcloud_node,
#     ])




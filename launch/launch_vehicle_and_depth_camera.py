
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
                name='rectify_left',
                namespace='left',
                remappings=[
                    ('image', 'image_raw'),
                    ('camera_info', 'camera_info'),
                ],
            ),
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_right',
                namespace='right',
                remappings=[
                    ('image', 'image_raw'),
                    ('camera_info', 'camera_info'),
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

    # **Fix**: feed color image when use_color=True
    stereo_pointcloud_node = launch_ros.actions.Node(
        package='stereo_image_proc', executable='point_cloud_node', name='point_cloud_node',
        output='screen',
        parameters=[{'approximate_sync': True}, {'queue_size': 10}, {'image_transport': 'raw'}, {'use_color': True}],
        remappings=[
            ('disparity', '/stereo/disparity'),
            ('left/image_rect_color', '/left/image_rect_color'),  # <-- changed
            ('left/camera_info', '/left/camera_info'),
            ('right/camera_info', '/right/camera_info'),
            # ('points2', '/stereo/points2'),  # optional custom topic
        ],
    )

    # ---- Node 1: Mount transform (quaternion form) ----
    stereo_left_mount_tf_pub = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='stereo_left_mount_tf_pub',
        output='screen',
        arguments=[
            '0.0', '0.0', '0.0',        # translation
            '0.0', '0.0', '0.0', '1.0', # quaternion (x,y,z,w)
            parent,                     # parent frame
            camera_frame                # child frame
        ],
    )

    # ---- Node 2: Optical transform (roll-pitch-yaw form; use standard optical rotation) ----
    stereo_left_optical_tf_pub = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='stereo_left_optical_tf_pub',
        output='screen',
        arguments=[
            '0.0', '0.0', '0.0',        # translation
            roll, pitch, yaw,           # RPY (r, p, y)  = [-pi/2, 0, -pi/2] by default
            camera_frame,               # parent frame
            optical_frame               # child frame
        ],
    )
    
    # Shows left/right rectified color images + a color-mapped disparity window
    # stereo_view = launch_ros.actions.Node(
    #     package='image_view',
    #     executable='stereo_view',
    #     name='stereo_view',
    #     output='screen',
    #     parameters=[
    #         #{'stereo': '/'},                 # look under /left/... and /right/...
    #         #{'image': 'image_rect_color'},   # subscribe to .../image_rect_color
    #         {'approximate_sync': True},      # optional; mentioned in your log
    #         {'queue_size': 10},              # optional; increase if needed
    #         {'image_transport': 'raw'},      # 'compressed' if bandwidth is tight
    #     ],
    #     remappings=[
    #         #('disparity', '/stereo/disparity'),  # let the viewer find your disparity
    #         #('left/image_rect_color', '/stereo/left/image'),
    #         #('right/image_rect_color', '/stereo/right/image')
    #         ('/stereo/left/image', 'left/image_rect_color'),
    #         ('/stereo/right/image', 'right/image_rect_color')
    #     ],
    # )


    # (Optional) Disparity-only viewer for /stereo/disparity
    disparity_view = launch_ros.actions.Node(
        package='image_view',
        executable='disparity_view',
        name='disparity_view',
        output='screen',
        remappings=[('image', '/stereo/disparity')],
        parameters=[{'autosize': False}],
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
        rectify_container,
        stereo_disparity_node,
        stereo_pointcloud_node,
        stereo_left_mount_tf_pub,
        stereo_left_optical_tf_pub,
        #stereo_view,
        disparity_view
    ])



# # -*- coding: utf-8 -*-
# from launch import LaunchDescription
# import launch_ros.actions
# import launch.event_handlers
# import launch.events
# from launch_ros.actions import ComposableNodeContainer
# from launch_ros.descriptions import ComposableNode

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
#             {'debug_camera': True},
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

#     # --- Rectify both cameras via composition (unique names per component) ---
#     # image_proc components: publish /{ns}/image_rect (mono) and /{ns}/image_rect_color (color).
#     rectify_container = ComposableNodeContainer(
#         name='image_proc_container',
#         namespace='',
#         package='rclcpp_components',
#         executable='component_container_mt',
#         output='screen',
#         composable_node_descriptions=[
#             ComposableNode(
#                 package='image_proc',
#                 plugin='image_proc::RectifyNode',
#                 name='rectify_left',      # unique component name
#                 namespace='left',         # topics under /left/...
#                 remappings=[
#                     ('image', 'image_raw'),          # /left/image_raw
#                     ('camera_info', 'camera_info'),  # /left/camera_info
#                     # publishes /left/image_rect and /left/image_rect_color
#                 ],
#             ),
#             ComposableNode(
#                 package='image_proc',
#                 plugin='image_proc::RectifyNode',
#                 name='rectify_right',     # unique component name
#                 namespace='right',        # topics under /right/...
#                 remappings=[
#                     ('image', 'image_raw'),          # /right/image_raw
#                     ('camera_info', 'camera_info'),  # /right/camera_info
#                 ],
#             ),
#         ],
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
#             ('left/image_rect_color', '/left/image_rect'),
#             ('left/camera_info', '/left/camera_info'),
#             ('right/camera_info', '/right/camera_info'),
#             # Optionally: ('points2', '/stereo/points2'),
#         ],
#     )

    
#     # ---- Node 1: Mount transform (quaternion form) ----
#     stereo_left_mount_tf_pub = launch_ros.actions.Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         name='stereo_left_mount_tf_pub',
#         output='screen',
#         arguments=[
#             '0.0', '0.0', '0.0',            # translation
#             '0.0', '0.0', '0.0', '1.0',     # quaternion (x,y,z,w)
#             "base_link",             # parent frame
#             "camera_frame"        # child frame
#         ],
#     ),

#     # ---- Node 2: Optical transform (roll-pitch-yaw form) ----
#     stereo_left_optical_tf_pub = launch_ros.actions.Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         name='stereo_left_optical_tf_pub',
#         output='screen',
#         arguments=[
#             '0.0', '0.0', '0.0',  # translation
#             '0.0', '0.0', '0.0',     # RPY (r, p, y)
#             "camera_frame",         # parent frame
#             "optical_frame"         # child frame
#         ],
#     ),



#     return LaunchDescription([
#         mavs_vehicle,
#         mavs_depth_camera,
#         mavs_aggregator,
#         rectify_container,
#         stereo_disparity_node,
#         stereo_pointcloud_node,
#         stereo_left_mount_tf_pub,
#         stereo_left_optical_tf_pub
#     ])



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




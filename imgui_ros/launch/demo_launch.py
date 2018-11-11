# Copyright 2018 Lucas Walter

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    image_manip_dir = get_package_share_directory('image_manip')
    print('image_manip dir ' + image_manip_dir)
    # TODO(lucasw) include roto zoom launch
    # image_pub = launch_ros.actions.Node(
    #         package='image_manip2', node_executable='image_publisher', output='screen',
    #         arguments=[image_manip_dir + "/data/mosaic.jpg"])
    roto_zoom = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
            image_manip_dir + '/launch/roto_zoom_launch.py'))
    imgui_ros = launch_ros.actions.Node(
            package='imgui_ros', node_executable='imgui_ros_node', output='screen',
            # arguments=[image_manip_dir + "/data/mosaic.jpg"])
            remappings=[])
    configure_windows = launch_ros.actions.Node(
            package='imgui_ros', node_executable='demo.py', output='screen')
    # TODO(lucasw) need rclpy node that calls add_window to display the image
    # from the the image publisher

    return launch.LaunchDescription([
        roto_zoom,
        # image_pub,
        imgui_ros,
        configure_windows,
    ])

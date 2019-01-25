# Copyright 2018 Lucas Walter

import launch
import launch_ros.actions
import os
import yaml

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # image_manip_dir = get_package_share_directory('image_manip')
    # print('image_manip dir ' + image_manip_dir)

    launches = []

    node_name = 'imgui_ros'
    params = dict(
        name = 'imgui_ros demo',
        width = 1000,
        height = 800,
        )
    node = launch_ros.actions.Node(
            package='imgui_ros', node_executable='imgui_ros_node', output='screen',
            node_name=node_name,
            # arguments=[image_manip_dir + "/data/mosaic.jpg"])
            # arguments=['__params:=' + param_file],
            parameters=[params],
            remappings=[])
    launches.append(node)

    node = launch_ros.actions.Node(
            package='imgui_ros', node_executable='graph.py', output='screen')
    launches.append(node)

    if False:
        node = launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                 get_package_share_directory('imgui_ros') + '/launch/shaders_launch.py'))
        launches.append(node)

        node = launch_ros.actions.Node(
                package='imgui_ros', node_executable='pub_shape.py', output='screen',
                )
        launches.append(node)
        node = launch_ros.actions.Node(
                package='imgui_ros', node_executable='cameras.py', output='screen',
                )
        launches.append(node)

    return launch.LaunchDescription(launches)

# Copyright 2018 Lucas Walter

import launch
import launch_ros.actions
import os
import yaml

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    launches = []

    # TODO(lucasw) all the nodes come out named node_loader when using this launch file,
    # otherwise they are named as desired.
    node = launch_ros.actions.Node(
            package='internal_pub_sub',
            node_executable='node_loader',
            output='screen',
            node_name='node_loader',
            )
    launches.append(node)

    node = launch_ros.actions.Node(
            package='imgui_ros',
            node_executable='demo_imgui_ros.py',
            node_name='demo_imgui_ros',
            output='screen',
            arguments=['-n'],
            )
    launches.append(node)

    return launch.LaunchDescription(launches)

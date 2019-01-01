# Copyright 2018 Lucas Walter

import launch
import launch_ros.actions
import os
import yaml

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    prefix = '/tmp/ros2/imgui_ros_demo/'
    if not os.path.exists(prefix):
        os.makedirs(prefix)

    shader_dir = get_package_share_directory('imgui_ros') + '/../../lib/imgui_ros/'
    vertex_filename = shader_dir + 'vertex.glsl'
    fragment_filename = shader_dir + 'fragment.glsl'
    add_default_shaders = launch_ros.actions.Node(
            package='imgui_ros', node_executable='add_shaders.py',
            node_name='add_shaders',
            output='screen',
            arguments=['-n', 'default', '-v', vertex_filename, '-f', fragment_filename],
            )
    vertex_filename = shader_dir + 'depth_vertex.glsl'
    fragment_filename = shader_dir + 'depth_fragment.glsl'
    add_depth_shaders = launch_ros.actions.Node(
            package='imgui_ros', node_executable='add_shaders.py',
            node_name='add_depth_shaders',
            output='screen',
            arguments=['-n', 'depth', '-v', vertex_filename, '-f', fragment_filename],
            )
    vertex_filename = shader_dir + 'cube_camera_vertex.glsl'
    fragment_filename = shader_dir + 'cube_camera_fragment.glsl'
    add_cube_camera_shaders = launch_ros.actions.Node(
            package='imgui_ros', node_executable='add_shaders.py',
            node_name='add_cube_camera_shaders',
            output='screen',
            arguments=['-n', 'cube_map', '-v', vertex_filename, '-f', fragment_filename],
            )

    return launch.LaunchDescription([
        add_default_shaders,
        add_depth_shaders,
        # add_cube_camera_shaders,
    ])

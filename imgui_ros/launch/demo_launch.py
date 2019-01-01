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

    image_manip_dir = get_package_share_directory('image_manip')
    print('image_manip dir ' + image_manip_dir)
    # TODO(lucasw) include roto zoom launch
    # image_pub = launch_ros.actions.Node(
    #         package='image_manip2', node_executable='image_publisher', output='screen',
    #         arguments=[image_manip_dir + "/data/mosaic.jpg"])
    # roto_zoom = launch.actions.Node(
    #         launch.launch_description_sources.PythonLaunchDescriptionSource(
    #         image_manip_dir + '/launch/roto_zoom_launch.py'))
    static_tf = launch_ros.actions.Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            node_name='static_foo',
            output='screen',
            arguments=['0.1', '0.2', '0.3', '0.4', '.5', '.6', 'map', 'foo'],
            # this doesn't work- it becomes one arg?
            # arguments=['0.1 0.2 0.3 0.4 .5 .6 map foo'],
            )

    node_name = 'imgui_ros'
    params = dict(
        name = 'imgui_ros demo',
        width = 1440,
        height = 800,
        )
    param_file = prefix + node_name + '.yaml'
    with open(param_file, 'w') as outfile:
        print('opened ' + param_file + ' for yaml writing')
        data = {}
        data[node_name] = dict(ros__parameters = params)
        yaml.dump(data, outfile, default_flow_style=False)
    imgui_ros = launch_ros.actions.Node(
            package='imgui_ros', node_executable='imgui_ros_node', output='screen',
            node_name=node_name,
            # arguments=[image_manip_dir + "/data/mosaic.jpg"])
            arguments=['__params:=' + param_file],
            remappings=[])

    shader_dir = get_package_share_directory('imgui_ros') + '/../../lib/imgui_ros/'
    vertex_filename = shader_dir + 'vertex.glsl'
    fragment_filename = shader_dir + 'fragment.glsl'
    add_shaders = launch_ros.actions.Node(
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

    configure_windows = launch_ros.actions.Node(
            package='imgui_ros', node_executable='demo.py', output='screen')
    add_shapes = launch_ros.actions.Node(
            package='imgui_ros', node_executable='pub_shape.py', output='screen',
            )

    return launch.LaunchDescription([
        # thse are both taking 100% cpu
        # roto_zoom,
        # static_tf,
        # image_pub,
        imgui_ros,
        configure_windows,
        add_shaders,
        add_depth_shaders,
        add_shapes,
    ])

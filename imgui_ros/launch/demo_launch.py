# Copyright 2018 Lucas Walter

import launch
import launch_ros.actions
import os
import yaml

from ament_index_python.packages import get_package_share_directory

def make_param_file(param_file, node_name, params):
    with open(param_file, 'w') as outfile:
        print('opened ' + param_file + ' for yaml writing')
        data = {}
        data[node_name] = dict(ros__parameters = params)
        yaml.dump(data, outfile, default_flow_style=False)

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
    # roto_zoom = launch.actions.IncludeLaunchDescription(
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

    launches = []

    if False:
        node_name = 'imgui_ros'
        params = dict(
            name = 'imgui_ros demo',
            width = 1440,
            height = 800,
            )
        param_file = prefix + node_name + '.yaml'
        make_param_file(param_file, node_name, params)
        imgui_ros = launch_ros.actions.Node(
                package='imgui_ros', node_executable='imgui_ros_node', output='screen',
                node_name=node_name,
                # arguments=[image_manip_dir + "/data/mosaic.jpg"])
                arguments=['__params:=' + param_file],
                remappings=[])
        launches.append(imgui_ros)
    else:
        node = launch_ros.actions.Node(
                package='internal_pub_sub', node_executable='node_loader', output='screen',
                node_name='node_loader',
                )
        launches.append(node)

        node = launch_ros.actions.Node(
                package='imgui_ros', node_executable='demo_start.py', output='screen')
        launches.append(node)

    node = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
             get_package_share_directory('imgui_ros') + '/launch/shaders_launch.py'))
    launches.append(node)

    node = launch_ros.actions.Node(
            package='imgui_ros', node_executable='demo.py', output='screen')
    launches.append(node)

    node = launch_ros.actions.Node(
            package='imgui_ros', node_executable='pub_shape.py', output='screen',
            )
    launches.append(node)

    node = launch_ros.actions.Node(
            package='imgui_ros', node_executable='cameras.py', output='screen',
            )
    launches.append(node)

    # TODO(lucasw) move to node loader
    node_name = 'generate_pointcloud2'
    params = dict(
        frame_id = 'bar2',
        )
    param_file = prefix + node_name + '.yaml'
    make_param_file(param_file, node_name, params)
    node = launch_ros.actions.Node(
            package='imgui_ros', node_executable='generate_pointcloud2', output='screen',
            node_name=node_name,
            arguments=['__params:=' + param_file],
            )
    launches.append(node)

    return launch.LaunchDescription(launches)

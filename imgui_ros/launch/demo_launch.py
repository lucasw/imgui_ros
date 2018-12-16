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
    roto_zoom = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
            image_manip_dir + '/launch/roto_zoom_launch.py'))
    # this requires https://github.com/lucasw/geometry2/tree/static_tf_args_bouncy
    static_tf = launch_ros.actions.Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
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

    add_shaders = launch_ros.actions.Node(
            package='imgui_ros', node_executable='add_shaders.py', output='screen')
    configure_windows = launch_ros.actions.Node(
            package='imgui_ros', node_executable='demo.py', output='screen')
    add_shapes = launch_ros.actions.Node(
            package='imgui_ros', node_executable='pub_shape.py', output='screen')

    return launch.LaunchDescription([
        roto_zoom,
        static_tf,
        # image_pub,
        imgui_ros,
        configure_windows,
        add_shapes,
        add_shaders,
    ])

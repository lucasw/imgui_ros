# Copyright Lucas Walter 2019
# Requires internal_pub_sub branch of usb_cam https://github.com/lucasw/usb_cam/tree/internal_pub_sub
# git clone -b internal_pub_sub https://github.com/lucasw/usb_cam

import argparse
import launch
import launch_ros.actions
import os
import sys
import time
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import ThisLaunchFileDir


def write_params(prefix, ns, node_name, params):
    name = prefix + ns + node_name + '_params.yaml'
    path = os.path.dirname(name)
    if not os.path.exists(path):
        os.makedirs(path)
    with open(name, 'w') as outfile:
        print('opened ' + name + ' for yaml parameter writing')
        data = {}
        data[ns] = {}
        for node_name in params.keys():
            data[ns][node_name] = {}
            data[ns][node_name]['ros__parameters'] = params[node_name]
        yaml.dump(data, outfile, default_flow_style=False)
        return name
    print('error opening file for parameter writing: ' + name)
    return None

def generate_launch_description():
    parser = argparse.ArgumentParser(description='usb_cam_viewer')
    # parser.add_argument('-cmp', '--composed', dest='composed',
    #             help='launch composed nodes', action='store_true')
    args, unknown = parser.parse_known_args(sys.argv[4:])

    # can't set to ''
    namespace = '/'  # '/tmp'

    device = '/dev/video0'
    usb_params = dict(
        video_device = device,
        # io_method
        # pixel_format
        image_width = 640,
        image_height = 480,
        # camera_name = 'usb_camera',
        )
    imgui_params = dict(
        name = device,
        width = 800,
        height = 600,
        )

    launches = []
    if True:
        prefix = "/tmp/ros2/" + str(int(time.time())) + "/"
        print('writing launch parameter files to ' + prefix)

        params = {}
        params['usb_cam'] = usb_params
        params['imgui_ros'] = imgui_params

        # if the node_name is left blank, and all the default node names
        # match the names in params above, then the parameters will
        # get loaded
        composed_exec_name = 'usb_cam_viewer'
        param_file = write_params(prefix, namespace, composed_exec_name, params)
        params_arg = '__params:=' + param_file

        node = launch_ros.actions.Node(
                package='imgui_ros',
                node_executable=composed_exec_name,
                arguments=[params_arg],
                node_namespace=[namespace],
                remappings=[
                            ],
                output='screen')
        launches.append(node)

    node = launch_ros.actions.Node(
            package='imgui_ros',
            node_executable='usb_cam_viewer.py',
            node_namespace=[namespace],
            remappings=[
                        ],
            output='screen')
    launches.append(node)

    return launch.LaunchDescription(launches)

#!/usr/bin/env python3
# Copyright (c) 2019 Lucas Walter
# February 2019
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import argparse
import imgui_ros
import internal_pub_sub
import rclpy
import sys
import time

from ament_index_python.packages import get_package_share_directory
from internal_pub_sub.msg import NodeSettings, Remapping
from internal_pub_sub.srv import AddNode
from rcl_interfaces.msg import Parameter, ParameterType
from rclpy.node import Node


class DemoImguiRos(Node):
    def __init__(self):
        super().__init__('demo_add_node')
        self.node_cli = self.create_client(AddNode, 'add_node')
        while not self.node_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().info(self.get_namespace() + '/add_node service not available')
            time.sleep(1.0)

    def wait_for_response(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                if self.future.result() is not None:
                    response = self.future.result()
                    self.get_logger().info(
                        'Result %s' % (str(response)))
                else:
                    self.get_logger().info(
                        'Service call failed %r' % (self.future.exception(),))
                break

    def run(self, namespace='', load_imgui_node=True):
        add_node = AddNode.Request()
        if load_imgui_node:
            node_settings = NodeSettings()
            node_settings.package_name = 'imgui_ros'
            node_settings.plugin_name = 'ImguiRos'
            node_settings.node_name = 'imgui_ros'
            node_settings.node_namespace = namespace
            node_settings.internal_pub_sub = True

            # TODO(lucasw) need to be able to specify parameters with fewer lines
            # parameters
            node_settings.parameters.append(internal_pub_sub.string_param('name', 'imgui_ros_demo'))
            node_settings.parameters.append(internal_pub_sub.integer_param('width', 1440))
            node_settings.parameters.append(internal_pub_sub.integer_param('height', 800))
            node_settings.parameters.append(internal_pub_sub.double_param('red', 0.5))
            node_settings.parameters.append(internal_pub_sub.double_param('green', 0.5))
            node_settings.parameters.append(internal_pub_sub.double_param('blue', 0.52))

            if False:
                node_settings.remappings.append(internal_pub_sub.make_remapping('image', 'different_image'))

            add_node.node_settings.append(node_settings)

        # generate test pointcloud2
        if False:
            # this is taking up 100% cpu
            node_settings = NodeSettings()
            node_settings.package_name = 'imgui_ros'
            node_settings.plugin_name = 'GeneratePointCloud2'
            node_settings.node_name = 'generate_pointcloud2'
            node_settings.node_namespace = namespace
            node_settings.internal_pub_sub = False
            add_node.node_settings.append(node_settings)

        self.future = self.node_cli.call_async(add_node)
        self.wait_for_response()

        imgui_ros.add_default_shaders(namespace)

        try:
            print("add shapes and textures")
            node = imgui_ros.PubShape()
            node.run(namespace)
        finally:
            node.destroy_node()

        try:
            print("add cameras")
            node = imgui_ros.Cameras()
            node.run(namespace)
        finally:
            node.destroy_node()

        return

        # TODO(lucasw) something is wrong with these controls that is taking 100% cpu
        try:
            print("add gui controls")
            node = imgui_ros.DemoGui()
            node.run(namespace)
        finally:
            node.destroy_node()

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='load_sim_line_nodes')
    parser.add_argument('-n', '--load-imgui-node', dest='load_imgui_node',  # type=bool,
            help='enable node loading', action='store_true')  # , default=True)
    args, unknown = parser.parse_known_args(sys.argv)

    try:
        demo = DemoImguiRos()
        demo.run('', args.load_imgui_node)
    finally:
        demo.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

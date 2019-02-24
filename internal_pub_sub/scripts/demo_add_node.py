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
#POSSIBILITY OF SUCH DAMAGE.

import internal_pub_sub
import rclpy
import time

from internal_pub_sub.msg import NodeSettings, Remapping
from internal_pub_sub.srv import AddNode
from rcl_interfaces.msg import Parameter, ParameterType
from rclpy.node import Node


class DemoAddNode(Node):
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

    def run(self):
        if False:
            add_node = AddNode.Request()
            # rviz2 isn't composable
            node_settings = NodeSettings()
            node_settings.package_name = 'rviz'
            node_settings.plugin_name = 'Rviz2'
            node_settings.node_namespace = 'foo'
            node_settings.node_name = 'bar'
            node_settings.arguments.extend(['-f', 'foo'])
            node_settings.internal_pub_sub = True
            add_node.node_settings.append(node_settings)

        # time.sleep(2.0)
        # demonstrate replacing loading nodes with different node
        # self.add_color_node('foo', 'bar')

        self.add_imgui_node('foo', 'bar')

        time.sleep(2.0)
        self.unload_node('foo', 'bar')

    def unload_node(self, node_namespace, node_name):
        # demonstrate replacing loading nodes with different node
        add_node = AddNode.Request()

        node_settings = NodeSettings()
        node_settings.node_namespace = node_namespace
        node_settings.node_name = node_name
        node_settings.remove= True
        add_node.node_settings.append(node_settings)

        self.future = self.node_cli.call_async(add_node)
        self.wait_for_response()

    def add_imgui_node(self, node_namespace, node_name):
        add_node = AddNode.Request()

        node_settings = NodeSettings()
        node_settings.package_name = 'imgui_ros'
        node_settings.plugin_name = 'ImguiRos'
        # node_settings.package_name = 'image_manip'
        # node_settings.plugin_name = 'Color'
        node_settings.node_namespace = node_namespace
        node_settings.node_name = node_name
        node_settings.internal_pub_sub = True

        node_settings.parameters.append(internal_pub_sub.string_param('name', 'imgui'))
        node_settings.parameters.append(internal_pub_sub.double_param('red', 0.3))
        node_settings.parameters.append(internal_pub_sub.integer_param('width', 1800))
        node_settings.parameters.append(internal_pub_sub.integer_param('height', 800))

        add_node.node_settings.append(node_settings)

        self.future = self.node_cli.call_async(add_node)
        self.wait_for_response()

    def add_color_node(self, node_namespace, node_name):
        add_node = AddNode.Request()

        node_settings = NodeSettings()
        node_settings.package_name = 'image_manip'
        node_settings.plugin_name = 'Color'
        node_settings.node_namespace = node_namespace
        node_settings.node_name = node_name
        node_settings.internal_pub_sub = True

        node_settings.parameters.append(internal_pub_sub.integer_param('red', 250))
        node_settings.parameters.append(internal_pub_sub.integer_param('width', 768))
        node_settings.parameters.append(internal_pub_sub.integer_param('height', 758))
        node_settings.parameters.append(internal_pub_sub.double_param('frame_rate', 13.0))
        node_settings.remappings.append(internal_pub_sub.make_remapping('image', 'different_image'))

        add_node.node_settings.append(node_settings)

        self.future = self.node_cli.call_async(add_node)
        self.wait_for_response()

def main(args=None):
    rclpy.init(args=args)

    try:
        demo = DemoAddNode()
        demo.run()
    finally:
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

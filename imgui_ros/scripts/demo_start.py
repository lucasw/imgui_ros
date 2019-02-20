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

import rclpy
import time

from internal_pub_sub.msg import NodeSettings, Remapping
from internal_pub_sub.srv import AddNode
from rcl_interfaces.msg import Parameter, ParameterType
from rclpy.node import Node


class DemoStart(Node):
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
        if True:
            node_settings = NodeSettings()
            node_settings.package_name = 'imgui_ros'
            node_settings.plugin_name = 'ImguiRos'
            node_settings.node_name = 'imgui_ros'
            node_settings.node_namespace = ''
            node_settings.internal_pub_sub = True

            # TODO(lucasw) need to be able to specify parameters with fewer lines
            # parameters
            param = Parameter()
            param.name = 'name'
            param.value.type = ParameterType.PARAMETER_STRING
            param.value.string_value = 'imgui_ros demo'
            node_settings.parameters.append(param)

            param = Parameter()
            param.name = 'width'
            param.value.type = ParameterType.PARAMETER_INTEGER
            param.value.integer_value = 1440
            node_settings.parameters.append(param)

            param = Parameter()
            param.name = 'height'
            param.value.type = ParameterType.PARAMETER_INTEGER
            param.value.integer_value = 800
            node_settings.parameters.append(param)

            param = Parameter()
            param.name = 'red'
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = 0.5
            node_settings.parameters.append(param)

            param = Parameter()
            param.name = 'green'
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = 0.5
            node_settings.parameters.append(param)

            param = Parameter()
            param.name = 'blue'
            param.value.type = ParameterType.PARAMETER_DOUBLE
            param.value.double_value = 0.52
            node_settings.parameters.append(param)

            if False:
                remapping = Remapping()
                remapping.from_topic = "image"
                remapping.to_topic = "different_image"
                node_settings.remappings.append(remapping)

            add_node = AddNode.Request()
            add_node.node_settings.append(node_settings)
            self.future = self.node_cli.call_async(add_node)
            self.wait_for_response()

def main(args=None):
    rclpy.init(args=args)

    try:
        demo = DemoStart()
        demo.run()
        rclpy.spin(demo)
    finally:
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

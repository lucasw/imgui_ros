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

import imgui_ros
import rclpy
import time

from ament_index_python.packages import get_package_share_directory
from internal_pub_sub.msg import NodeSettings, Remapping
from internal_pub_sub.srv import AddNode
from rcl_interfaces.msg import Parameter, ParameterType
from rclpy.node import Node


# TODO(lucasw) make an importable script in internal_pub_sub with all of these
def double_param(name, value=0.0):
    param = Parameter()
    param.name = name
    param.value.type = ParameterType.PARAMETER_DOUBLE
    param.value.double_value = value
    return param

def integer_param(name, value=0):
    param = Parameter()
    param.name = name
    param.value.type = ParameterType.PARAMETER_INTEGER
    param.value.integer_value = value
    return param

def bool_param(name, value=0):
    param = Parameter()
    param.name = name
    param.value.type = ParameterType.PARAMETER_BOOL
    param.value.bool_value = value
    return param

def string_param(name, value=''):
    param = Parameter()
    param.name = name
    param.value.type = ParameterType.PARAMETER_STRING
    param.value.string_value = value
    return param

def make_remapping(from_topic, to_topic):
    remapping = Remapping()
    remapping.from_topic = "image"
    remapping.to_topic = "different_image"
    return remapping

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

    def run(self):
        add_node = AddNode.Request()
        if False:
            node_settings = NodeSettings()
            node_settings.package_name = 'imgui_ros'
            node_settings.plugin_name = 'ImguiRos'
            node_settings.node_name = 'imgui_ros'
            node_settings.node_namespace = ''
            node_settings.internal_pub_sub = True

            # TODO(lucasw) need to be able to specify parameters with fewer lines
            # parameters
            node_settings.parameters.append(string_param('name', 'imgui_ros_demo'))
            node_settings.parameters.append(integer_param('width', 1440))
            node_settings.parameters.append(integer_param('height', 800))
            node_settings.parameters.append(double_param('red', 0.5))
            node_settings.parameters.append(double_param('green', 0.5))
            node_settings.parameters.append(double_param('blue', 0.52))

            if False:
                node_settings.remappings.append(make_remapping('image', 'different_image'))

            add_node.node_settings.append(node_settings)

        self.future = self.node_cli.call_async(add_node)
        self.wait_for_response()

        shader_dir = get_package_share_directory('imgui_ros') + '/../../lib/imgui_ros/'
        print("loading shaders from " + shader_dir)

        try:
            node = imgui_ros.AddShadersNode()
            node.run('default',
                     shader_dir + 'vertex.glsl',
                     shader_dir + 'fragment.glsl')
            node.run('depth',
                     shader_dir + 'depth_vertex.glsl',
                     shader_dir + 'depth_fragment.glsl')
            node.run('cube_map',
                     shader_dir + 'cube_camera_vertex.glsl',
                     shader_dir + 'cube_camera_fragment.glsl')
        finally:
            node.destroy_node()

        try:
            node = imgui_ros.PubShape()
            node.run()
        finally:
            node.destroy_node()

        try:
            node = imgui_ros.Cameras()
            node.run()
        finally:
            node.destroy_node()

        # TODO(lucasw) generate_pointcloud2

def main(args=None):
    rclpy.init(args=args)

    try:
        demo = DemoImguiRos()
        demo.run()
    finally:
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# Copyright 2018 Lucas Walter
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import cv2
import cv_bridge
import math
# TODO(lucasW) this doesn't exist in python yet?
# import tf2_ros
import rclpy
import sys

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, TransformStamped, Vector3
from imgui_ros.msg import TexturedShape, Widget
from imgui_ros.srv import AddShaders, AddShape, AddTexture, AddWindow
from rclpy.node import Node
from shape_msgs.msg import MeshTriangle, Mesh
from std_msgs.msg import ColorRGBA
from time import sleep
from visualization_msgs.msg import Marker


class AddShadersNode(Node):

    def __init__(self):
        super().__init__('add_shaders')
        # self.marker_pub = self.create_publisher(Marker, 'marker')
        # self.shape_pub = self.create_publisher(TexturedShape, 'shapes')
        self.shaders_cli = self.create_client(AddShaders, 'add_shaders')
        while not self.shaders_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.bridge = cv_bridge.CvBridge()
        sleep(1.0)

    # TODO(lucasw) can't this be a callback instead?
    def wait_for_response(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                if self.future.result() is not None:
                    response = self.future.result()
                    self.get_logger().info(
                        'Result: %s success: %s' % (response.message, str(response.success)))
                    return response
                else:
                    self.get_logger().info(
                        'Service call failed %r' % (self.future.exception(),))
                    return None
                break

    def run(self):
        parser = argparse.ArgumentParser(description='add_shaders')
        parser.add_argument('-n', '--name', dest='name', type=str,
                help='shader set name', default='default')
        parser.add_argument('-v', '--vertex', dest='vertex', type=str,
                help='vertex shader file', default='')
        parser.add_argument('-f', '--fragment', dest='fragment', type=str,
                help='vertex shader file', default='')
        self.args, unknown = parser.parse_known_args(sys.argv)
        self.get_logger().info("unknown args: {}".format(unknown))
        self.get_logger().info("args: {}".format(self.args))
        self.add_shaders(self.args.name, self.args.vertex, self.args.fragment)

    def add_shaders(self, name, vertex_filename, fragment_filename):
        req = AddShaders.Request()
        req.name = name
        req.vertex = ''
        if vertex_filename != '':
            with open(vertex_filename) as vertex_file:
                req.vertex = vertex_file.read()
        if req.vertex == '':
            self.get_logger().error("couldn't load vertex shader from: '{}'".format(
                    vertex_filename))
            return False
        # print(req.vertex)
        # print('####################################################')
        req.fragment = ''
        if fragment_filename != '':
            with open(fragment_filename) as fragment_file:
                req.fragment = fragment_file.read()
        if req.vertex == '':
            self.get_logger().error("couldn't load fragment shader from '{}'".format(
                    fragment_filename))
            return False
        # print(req.fragment)
        self.future = self.shaders_cli.call_async(req)
        rv = self.wait_for_response()
        if rv is not None:
            return rv.success
        return False

def main(args=None):
    rclpy.init(args=args)

    try:
        demo = AddShadersNode()
        demo.run()
        rclpy.spin(demo)
    finally:
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

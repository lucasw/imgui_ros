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

# TODO(lucasW) this doesn't exist in python yet?
# import tf2_ros
import math
import rclpy

from geometry_msgs.msg import Point, TransformStamped
from imgui_ros.msg import TexturedShape, TfWidget, Widget
from imgui_ros.srv import AddTf, AddWindow
from rclpy.node import Node
from shape_msgs.msg import MeshTriangle, Mesh
from transforms3d import _gohlketransforms as tg
from visualization_msgs.msg import Marker


class Demo(Node):

    def __init__(self):
        super().__init__('usb_cam_viewer')
        self.cli = self.create_client(AddWindow, 'add_window')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('add_window service not available, waiting again...')

    # TODO(lucasw) can't this be a callback instead?
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

    def init(self):
        self.add_images()

    def add_images(self):
        req = AddWindow.Request()
        req.name = "usb cam viewer"
        tab_name = 'images'
        widget = Widget()
        widget.name = "roto image"
        widget.tab_name = tab_name
        widget.topic = "image_raw"
        widget.type = Widget.IMAGE
        req.widgets.append(widget)
        self.future = self.cli.call_async(req)
        self.wait_for_response()

def main(args=None):
    rclpy.init(args=args)

    try:
        demo = Demo()
        demo.init()
        rclpy.spin(demo)
    finally:
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

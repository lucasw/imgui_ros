#!/usr/bin/env python3
# Copyright 2016 Open Source Robotics Foundation, Inc.
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

from imgui_ros.msg import Widget
from imgui_ros.srv import AddWindow

import rclpy
from rclpy.node import Node


class Demo(Node):

    def __init__(self):
        super().__init__('demo')
        self.cli = self.create_client(AddWindow, 'add_window')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

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

    def run(self):
        req = AddWindow.Request()
        req.name = "input image"
        widget = Widget()
        widget.name = "roto image"
        widget.topic = "/image_raw"
        widget.type = Widget.IMAGE
        req.widgets.append(widget)
        self.future = self.cli.call_async(req)

        req = AddWindow.Request()
        req.name = "rotated image"
        widget = Widget()
        widget.name = "image_out viewer"
        widget.topic = "/image_out"
        widget.type = Widget.IMAGE
        req.widgets.append(widget)
        self.future = self.cli.call_async(req)

        req = AddWindow.Request()
        req.name = "rotozoom controls"

        for ctrl in ["psi", "theta", "phi"]:
            widget = Widget()
            widget.name = ctrl + " pub"
            widget.topic = ctrl
            widget.type = Widget.PUB
            widget.sub_type = Widget.FLOAT32
            widget.value = 0.0
            widget.min = -3.2
            widget.max = 3.2
            req.widgets.append(widget)

            widget = Widget()
            widget.name = ctrl + " sub"
            widget.topic = ctrl
            widget.type = Widget.PLOT
            widget.sub_type = Widget.FLOAT32
            req.widgets.append(widget)

        widget = Widget()
        widget.name = "z pub"
        widget.topic = "z"
        widget.type = Widget.PUB
        widget.sub_type = Widget.FLOAT32
        widget.value = 1.0
        widget.min = 0.01
        widget.max = 5.0
        req.widgets.append(widget)

        widget = Widget()
        widget.name = "z_scale pub"
        widget.topic = "z_scale"
        widget.type = Widget.PUB
        widget.sub_type = Widget.FLOAT32
        widget.value = 0.005
        widget.min = 0.0
        widget.max = 0.05
        req.widgets.append(widget)

        for ctrl in ["width", "height"]:
            widget = Widget()
            widget.name = ctrl + " pub"
            widget.topic = ctrl
            widget.type = Widget.PUB
            widget.sub_type = Widget.INT32
            # has to be float even though type above is int
            widget.value = 0.0
            widget.min = 0.0
            widget.max = 2048.0
            req.widgets.append(widget)

        self.future = self.cli.call_async(req)

def main(args=None):
    rclpy.init(args=args)

    demo = Demo()
    demo.run()

    demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

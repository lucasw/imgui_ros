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

from imgui_ros2.srv import AddWindow

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
        req.name = "image_raw viewer"
        req.topic = "/image_raw"
        req.type = AddWindow.Request.IMAGE
        self.future = self.cli.call_async(req)

        req = AddWindow.Request()
        req.name = "image_out viewer"
        req.topic = "/image_out"
        req.type = AddWindow.Request.IMAGE
        self.future = self.cli.call_async(req)


        req = AddWindow.Request()
        req.name = "theta pub"
        req.topic = "/theta"
        req.type = AddWindow.Request.PUB
        req.sub_type = AddWindow.Request.FLOAT32
        req.value = 0.0
        req.min = -3.2
        req.max = 3.2
        self.future = self.cli.call_async(req)

        req = AddWindow.Request()
        req.name = "phi pub"
        req.topic = "/phi"
        req.type = AddWindow.Request.PUB
        req.sub_type = AddWindow.Request.FLOAT32
        req.value = 0.0
        req.min = -3.2
        req.max = 3.2
        self.future = self.cli.call_async(req)

def main(args=None):
    rclpy.init(args=args)

    demo = Demo()
    demo.run()

    demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

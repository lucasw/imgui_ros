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
import rclpy

from geometry_msgs.msg import TransformStamped
from imgui_ros.msg import Widget
from imgui_ros.srv import AddWindow
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
        self.wait_for_response()

        req = AddWindow.Request()
        req.name = "rotated image"
        widget = Widget()
        widget.name = "image_out viewer"
        widget.topic = "/image_out"
        widget.type = Widget.IMAGE
        req.widgets.append(widget)
        self.future = self.cli.call_async(req)
        self.wait_for_response()

        req = AddWindow.Request()
        req.name = "rotozoom controls"

        widget = Widget()
        widget.name = "frame rate"
        widget.topic = "rotozoom"
        widget.items.append('frame_rate')
        widget.type = Widget.PARAM
        widget.sub_type = Widget.FLOAT32
        widget.value = 0.0
        widget.min = 0.0
        widget.max = 30.0
        req.widgets.append(widget)

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
            widget.type = Widget.SUB
            widget.sub_type = Widget.FLOAT32
            req.widgets.append(widget)

            widget = Widget()
            widget.name = ctrl + " plot"
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
            widget_type = Widget.INT32
            widget = Widget()
            widget.name = ctrl + " pub"
            widget.topic = ctrl
            widget.type = Widget.PUB
            widget.sub_type = widget_type
            # has to be float even though type above is int
            widget.value = 0.0
            widget.min = 0.0
            widget.max = 2048.0
            req.widgets.append(widget)

            widget = Widget()
            widget.name = ctrl + " sub"
            widget.topic = ctrl
            widget.type = Widget.SUB
            widget.sub_type = widget_type
            req.widgets.append(widget)

            widget = Widget()
            widget.name = ctrl + " plot"
            widget.topic = ctrl
            widget.type = Widget.PLOT
            widget.sub_type = widget_type
            req.widgets.append(widget)

        # string pub sub test
        string_topic = "string"
        widget = Widget()
        widget.name = "string pub"
        widget.topic = string_topic
        widget.type = Widget.PUB
        widget.sub_type = Widget.STRING
        # has to be float even though type above is int
        req.widgets.append(widget)

        widget = Widget()
        widget.name = "string menu pub"
        widget.topic = string_topic
        widget.type = Widget.PUB
        widget.sub_type = Widget.STRING
        widget.items = ['item1 foo', 'item2 bar', 'item final']
        # has to be float even though type above is int
        widget.value = 0.0
        req.widgets.append(widget)

        widget = Widget()
        widget.name = "string sub"
        widget.topic = string_topic
        widget.type = Widget.SUB
        widget.sub_type = Widget.STRING
        req.widgets.append(widget)

        self.future = self.cli.call_async(req)
        self.wait_for_response()

        #######################################
        req = AddWindow.Request()
        req.name = "tf viz"

        widget = Widget()
        widget.name = "map foo tf"
        widget.type = Widget.SUB
        widget.sub_type = Widget.TF
        widget.items.append("map")
        widget.items.append("foo")
        req.widgets.append(widget)

        widget = Widget()
        widget.name = "map pub tf"
        widget.type = Widget.PUB
        widget.sub_type = Widget.TF
        widget.min = -2.0
        widget.max = 2.0
        widget.items.append("map")
        widget.items.append("bar")
        req.widgets.append(widget)

        widget = Widget()
        widget.name = "viz2d"
        widget.type = Widget.SUB
        widget.sub_type = Widget.VIZ2D
        widget.topic = "map"
        widget.max = 100.0
        widget.items.append("foo")
        widget.items.append("bar")
        req.widgets.append(widget)

        self.future = self.cli.call_async(req)
        self.wait_for_response()

        # TODO(lucasw) move the tf broadcasting into standalone node
        self.elapsed = 0.0
        self.period = 0.05
        # self.br = tf2_ros.TransformBroadcaster()
        # self.timer = self.create_timer(self.period, self.update)

    def update(self):
        ts = TransformStamped()
        ts.header.stamp = self.now()
        ts.header.frame_id = "map"
        ts.child_frame_id = "bar"
        ts.transform.translation.x = math.cos(self.elapsed * 0.1)
        ts.transform.translation.y = math.sin(self.elapsed * 0.2)
        ts.transform.rotation.w = 1.0
        self.br.sendTransform(ts)
        self.elapsed += 0.05

def main(args=None):
    rclpy.init(args=args)

    demo = Demo()
    demo.run()

    demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

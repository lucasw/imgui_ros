#!/usr/bin/env python
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
import rospy

from imgui_ros_msgs.msg import Widget
from imgui_ros_msgs.srv import AddWindow, AddWindowRequest


class Console:
    def __init__(self):
        rospy.wait_for_service('add_window')
        self.cli = rospy.ServiceProxy('add_window', AddWindow)

        self.add_console()

    def add_console(self):
        # TF control widgets
        req = AddWindowRequest()
        req.name = "console"
        tab_name = "console2"

        widget = Widget()
        widget.name = "console3"
        widget.tab_name = tab_name
        widget.type = Widget.SUB
        widget.sub_type = Widget.ROSOUT
        widget.topic = "/rosout_agg"
        req.widgets.append(widget)

        # All the above take up about 10% cpu
        self.cli(req)

if __name__ == '__main__':
    rospy.init_node('setup_console')
    console = Console()

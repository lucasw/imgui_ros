#!/usr/bin/env python
# Copyright 2018 Lucas Walter
# BSD3 license
import math
import rospy

from imgui_ros_msgs.msg import Widget
from imgui_ros_msgs.srv import *
# from time import sleep


class DemoGui:
    def __init__(self):
        pass

    def run(self, namespace=''):
        self.cli = rospy.ServiceProxy(namespace + '/add_window', AddWindow)

        self.add_misc()

    def add_misc(self):
        req = AddWindowRequest()
        req.name = 'misc controls'
        req.position.x = 300
        req.position.y = 0
        req.size.x = 300
        req.size.y = 300
        tab_name = 'misc'

        # string pub sub test
        string_topic = "string"
        widget = Widget()
        widget.name = "string pub"
        widget.tab_name = tab_name
        widget.topic = string_topic
        widget.type = Widget.PUB
        widget.sub_type = Widget.STRING
        # has to be float even though type above is int
        req.widgets.append(widget)

        widget = Widget()
        widget.name = "string menu pub"
        widget.tab_name = tab_name
        widget.topic = string_topic
        widget.type = Widget.PUB
        widget.sub_type = Widget.STRING
        widget.items = ['item1 foo', 'item2 bar', 'item final']
        # has to be float even though type above is int
        widget.value = 0.0
        req.widgets.append(widget)

        widget = Widget()
        widget.name = "string sub"
        widget.tab_name = tab_name
        widget.topic = string_topic
        widget.topic = string_topic
        widget.type = Widget.SUB
        widget.sub_type = Widget.STRING
        req.widgets.append(widget)

        try:
            resp = self.cli(req)
            rospy.loginfo(resp)
        except rospy.service.ServiceException as e:
            rospy.logerr(e)

def main(args=None):
    rospy.init_node("imgui_ros_demo2")

    try:
        demo = DemoGui()
        demo.run()
    finally:
        pass

if __name__ == '__main__':
    main()

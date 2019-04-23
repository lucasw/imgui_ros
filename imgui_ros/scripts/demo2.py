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
        rospy.wait_for_service(namespace + '/add_window')
        self.cli = rospy.ServiceProxy(namespace + '/add_window', AddWindow)

        self.add_misc()
        self.add_dr()

    def add_dr(self):
        req = AddWindowRequest()
        req.name = 'misc controls'
        req.init = True
        req.fractional = False
        if req.fractional:
            # TODO(lucasw) fractional doesn't allow dragging of window around
            req.position.x = 0.0
            req.position.y = 0.0
            req.size.x = 0.5
            req.size.y = 0.5
        else:
            req.position.x = 200.0
            req.position.y = 0.0
            req.size.x = 300.0
            req.size.y = 400.0

        tab_name = 'dr'

        if True:
            widget = Widget()
            widget.name = "dr"
            widget.tab_name = tab_name
            widget.topic = '/example_server_manual_py'
            widget.type = Widget.DYNREC
            # widget.sub_type = Widget.IMAGE
            req.widgets.append(widget)

        try:
            resp = self.cli(req)
            rospy.loginfo(resp)
        except rospy.service.ServiceException as e:
            rospy.logerr(e)


    def add_misc(self):
        req = AddWindowRequest()
        req.name = 'misc controls'
        req.init = True
        req.fractional = False
        if req.fractional:
            # TODO(lucasw) fractional doesn't allow dragging of window around
            req.position.x = 0.0
            req.position.y = 0.0
            req.size.x = 0.5
            req.size.y = 0.5
        else:
            req.position.x = 100.0
            req.position.y = 0.0
            req.size.x = 300.0
            req.size.y = 400.0
        tab_name = 'misc'

        if True:
            widget = Widget()
            widget.name = "image pub"
            widget.tab_name = tab_name
            widget.topic = "test_image1/image_raw"
            widget.type = Widget.PUB
            widget.sub_type = Widget.IMAGE
            req.widgets.append(widget)

        if True:
            widget = Widget()
            widget.name = "image sub"
            widget.tab_name = tab_name
            widget.topic = "test_image2/image_raw"
            widget.type = Widget.SUB
            widget.sub_type = Widget.IMAGE
            req.widgets.append(widget)

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

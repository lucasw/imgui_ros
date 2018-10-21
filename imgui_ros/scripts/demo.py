#!/usr/bin/env python

import rospy

from imgui_ros.srv import *


class ImguiDemo:
    def __init__(self):
        rospy.wait_for_service('add_window')
        self.add_window = rospy.ServiceProxy('add_window', AddWindow)

        remove = rospy.get_param("~remove", False)

        try:
            req = AddWindowRequest()
            req.name = "source"
            req.topic = "/image_source/image_raw"
            req.remove = remove
            req.type = AddWindowRequest.IMAGE
            self.add_window(req)

            req = AddWindowRequest()
            req.name = "rotated"
            req.topic = "/image_source/rotated"
            req.remove = remove
            req.type = AddWindowRequest.IMAGE
            self.add_window(req)

            req = AddWindowRequest()
            req.name = "roto_zoom0"
            req.topic = "/image_source/roto_zoom0"
            req.remove = remove
            req.type = AddWindowRequest.DYNREC
            self.add_window(req)

        except rospy.service.ServiceException as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('imgui_demo')
    imgui_demo = ImguiDemo()
    # rospy.spin()

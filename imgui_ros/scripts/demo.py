#!/usr/bin/env python

import rospy

from imgui_ros.srv import *


class ImguiDemo:
    def __init__(self):
        self.add_image = rospy.ServiceProxy('add_image', Image)

        try:
            req = ImageRequest()
            req.name = "foo2"
            req.topic = "/image_source/image_raw"
            req.remove = rospy.get_param("~remove", False)
            self.add_image(req)
        except rospy.service.ServiceException as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('imgui_demo')
    imgui_demo = ImguiDemo()
    # rospy.spin()

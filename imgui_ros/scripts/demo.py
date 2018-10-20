#!/usr/bin/env python

import rospy

from imgui_ros.srv import *


class ImguiDemo:
    def __init__(self):
        rospy.wait_for_service('add_image')
        self.add_image = rospy.ServiceProxy('add_image', Image)

        remove = rospy.get_param("~remove", False)

        try:
            req = ImageRequest()
            req.name = "source"
            req.topic = "/image_source/image_raw"
            req.remove = remove
            self.add_image(req)

            req = ImageRequest()
            req.name = "rotated"
            req.topic = "/image_source/rotated"
            req.remove = remove
            self.add_image(req)

        except rospy.service.ServiceException as e:
            rospy.logerr(e)

if __name__ == '__main__':
    rospy.init_node('imgui_demo')
    imgui_demo = ImguiDemo()
    # rospy.spin()

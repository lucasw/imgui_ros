#!/usr/bin/env python
#

import rospy


class TestRosout:
    def __init__(self):
        i = 0
        while not rospy.is_shutdown():
            rospy.loginfo("test {}".format(i))
            if i % 10 == 0:
                rospy.loginfo("foo {}".format(i))
            rospy.sleep(0.05)
            i += 1

if __name__ == '__main__':
    rospy.init_node('test_rosout')
    test_rosout = TestRosout()

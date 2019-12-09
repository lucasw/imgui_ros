#!/usr/bin/env python
# Copyright 2018 Lucas Walter
# BSD3 license
import math
import rospy

from geometry_msgs.msg import TransformStamped
from imgui_ros_msgs.msg import TexturedShape, TfWidget, Vertex, Widget
from imgui_ros_msgs.srv import AddCamera, AddCubeCamera, AddProjector, AddShaders
from imgui_ros_msgs.srv import AddShape, AddTexture, AddTf, AddTfRequest, AddWindow, AddWindowRequest
from shape_msgs.msg import MeshTriangle, Mesh
from transforms3d import _gohlketransforms as tg


class DemoViz3DGui:
    def __init__(self):
        pass

    def run(self, namespace=''):
        rospy.wait_for_service(namespace + '/add_window')
        self.window_cli = rospy.ServiceProxy(namespace + '/add_window', AddWindow)
        rospy.wait_for_service(namespace + '/add_tf')
        self.tf_cli = rospy.ServiceProxy(namespace + '/add_tf', AddTf)

        self.add_tf()

    def add_tf(self):
        # TODO(lucasw) what if this window doesn't exist yet?
        req = AddWindowRequest()
        req.name = "misc controls"
        tab_name = 'tf'

        tf_widget = TfWidget()
        tf_widget.name = "cube camera tf"
        tf_widget.window = req.name
        tf_widget.tab_name = tab_name
        tf_widget.min = -2.0
        tf_widget.max = 2.0
        ts = TransformStamped()
        ts.header.frame_id = "viz3d_main_window_camera"
        ts.child_frame_id = "cube_camera"
        ts.transform.translation.x = 0.0
        ts.transform.translation.y = 0.0
        ts.transform.translation.z = 0.0
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        rot = tg.quaternion_from_euler(roll, pitch, yaw, 'sxyz')
        ts.transform.rotation.w = rot[0]
        ts.transform.rotation.x = rot[1]
        ts.transform.rotation.y = rot[2]
        ts.transform.rotation.z = rot[3]
        tf_widget.transform_stamped = ts
        tf_req = AddTfRequest()
        tf_req.tf = tf_widget

        try:
            resp = self.tf_cli(tf_req)
            rospy.loginfo(resp)
        except rospy.service.ServiceException as e:
            rospy.logerr(e)

        tf_widget = TfWidget()
        tf_widget.name = "cube camera lens pub"
        tf_widget.tab_name = tab_name
        tf_widget.window = req.name
        tf_widget.min = -2.0
        tf_widget.max = 2.0
        ts = TransformStamped()
        ts.header.frame_id = "cube_camera"
        ts.child_frame_id = "cube_camera_lens"
        ts.transform.translation.x = 0.0
        ts.transform.translation.y = 0.0
        ts.transform.translation.z = 0.0
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        rot = tg.quaternion_from_euler(roll, pitch, yaw, 'sxyz')
        ts.transform.rotation.w = rot[0]
        ts.transform.rotation.x = rot[1]
        ts.transform.rotation.y = rot[2]
        ts.transform.rotation.z = rot[3]
        tf_widget.transform_stamped = ts
        tf_req = AddTfRequest()
        tf_req.tf = tf_widget

        try:
            resp = self.tf_cli(tf_req)
            rospy.loginfo(resp)
        except rospy.service.ServiceException as e:
            rospy.logerr(e)

def main(args=None):
    rospy.init_node("imgui_ros_demo_viz3d")

    try:
        demo = DemoViz3DGui()
        demo.run()
    finally:
        pass

if __name__ == '__main__':
    main()

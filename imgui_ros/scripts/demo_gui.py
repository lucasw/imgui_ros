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
import math
import rospy

from geometry_msgs.msg import Point, TransformStamped
from imgui_ros_msgs.msg import TexturedShape, TfWidget, Widget
from imgui_ros_msgs.srv import AddTf, AddTfRequest, AddWindow, AddWindowRequest
from shape_msgs.msg import MeshTriangle, Mesh
from transforms3d import _gohlketransforms as tg
from visualization_msgs.msg import Marker


class DemoGui:
    def __init__(self):
        rospy.wait_for_service('add_window')
        self.cli = rospy.ServiceProxy('add_window', AddWindow)

        rospy.wait_for_service('add_tf')
        self.tf_cli = rospy.ServiceProxy('add_tf', AddTf)

        self.add_images()
        self.add_roto_controls()
        self.add_misc()
        self.add_viz()
        self.add_markers()
        # self.add_shapes()

    def add_images(self):
        req = AddWindowRequest()
        req.name = "misc controls"
        tab_name = 'images'
        widget = Widget()
        widget.name = "roto image"
        widget.tab_name = tab_name
        widget.topic = "camera1"
        widget.type = Widget.IMAGE
        req.widgets.append(widget)

        widget = Widget()
        widget.name = "image_out viewer"
        widget.tab_name = tab_name
        widget.topic = "cube_camera1"
        widget.type = Widget.IMAGE
        req.widgets.append(widget)
        self.cli(req)

    def add_roto_controls(self):
        req = AddWindowRequest()
        req.name = "misc controls"
        tab_name = 'roto'

        widget = Widget()
        widget.name = "frame rate"
        widget.tab_name = tab_name
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
            widget.tab_name = tab_name
            widget.topic = ctrl
            widget.type = Widget.PUB
            widget.sub_type = Widget.FLOAT32
            widget.value = 0.0
            widget.min = -3.2
            widget.max = 3.2
            req.widgets.append(widget)

            widget = Widget()
            widget.name = ctrl + " sub"
            widget.tab_name = tab_name
            widget.topic = ctrl
            widget.type = Widget.SUB
            widget.sub_type = Widget.FLOAT32
            req.widgets.append(widget)

            widget = Widget()
            widget.name = ctrl + " plot"
            widget.tab_name = tab_name
            widget.topic = ctrl
            widget.type = Widget.PLOT
            widget.sub_type = Widget.FLOAT32
            req.widgets.append(widget)

        widget = Widget()
        widget.name = "z pub"
        widget.tab_name = tab_name
        widget.topic = "z"
        widget.type = Widget.PUB
        widget.sub_type = Widget.FLOAT32
        widget.value = 1.0
        widget.min = 0.01
        widget.max = 5.0
        req.widgets.append(widget)

        widget = Widget()
        widget.name = "z_scale pub"
        widget.tab_name = tab_name
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
            widget.tab_name = tab_name
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
            widget.tab_name = tab_name
            widget.topic = ctrl
            widget.type = Widget.SUB
            widget.sub_type = widget_type
            req.widgets.append(widget)

            widget = Widget()
            widget.name = ctrl + " plot"
            widget.tab_name = tab_name
            widget.topic = ctrl
            widget.type = Widget.PLOT
            widget.sub_type = widget_type
            req.widgets.append(widget)

        self.cli(req)

    def add_misc(self):
        req = AddWindowRequest()
        req.name = 'misc controls'

        tab_name = 'misc'
        # point cloud2
        widget = Widget()
        widget.name = 'point cloud sub'
        widget.tab_name = tab_name
        widget.topic = '/point_cloud'
        widget.type = Widget.SUB
        widget.sub_type = Widget.POINTCLOUD
        # has to be float even though type above is int
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
        widget.topic = string_topic
        widget.type = Widget.SUB
        widget.sub_type = Widget.STRING
        req.widgets.append(widget)

        self.cli(req)

    def add_viz(self):
        # dedicate 'radar' for tf
        req = AddWindowRequest()
        req.name = "tf viz"

        tab_name = 'viz2d'

        widget = Widget()
        widget.name = "viz2d"
        widget.tab_name = tab_name
        widget.type = Widget.SUB
        widget.sub_type = Widget.VIZ2D
        widget.topic = 'marker'
        widget.max = 100.0
        widget.items.append("map")
        widget.items.append("projector1")
        widget.items.append("bar2")
        widget.items.append("camera1")
        widget.items.append("cube_camera1")
        req.widgets.append(widget)

        self.cli(req)

        # TF control widgets
        req = AddWindowRequest()
        req.name = "misc controls"
        tab_name = 'tf'

        widget = Widget()
        widget.name = "map foo tf"
        widget.tab_name = tab_name
        widget.type = Widget.SUB
        widget.sub_type = Widget.TF
        widget.items.append("map")
        widget.items.append("bar")
        req.widgets.append(widget)

        widget = Widget()
        widget.name = "camera1 pub tf"
        widget.tab_name = tab_name
        widget.type = Widget.PUB
        widget.sub_type = Widget.TF
        widget.min = -4.0
        widget.max = 4.0
        widget.items.append("viz3d_main_window_camera")
        widget.items.append("camera1")
        req.widgets.append(widget)

        widget = Widget()
        widget.name = "camera2 pub tf"
        widget.tab_name = tab_name
        widget.type = Widget.PUB
        widget.sub_type = Widget.TF
        widget.min = -4.0
        widget.max = 4.0
        widget.items.append("map")
        widget.items.append("camera2")
        req.widgets.append(widget)

        if True:  # False:
            widget = Widget()
            widget.name = "map pub tf"
            widget.tab_name = tab_name
            widget.remove = True
            widget.type = Widget.PUB
            widget.sub_type = Widget.TF
            widget.min = -2.0
            widget.max = 2.0
            widget.items.append("map")
            widget.items.append("projector1")
            req.widgets.append(widget)

        # All the above take up about 10% cpu
        self.cli(req)

        # dedicated tf add service with more configurability

        tf_widget = TfWidget()
        tf_widget.name = "floor tf"
        tf_widget.tab_name = tab_name
        tf_widget.window = req.name
        tf_widget.min = -3.0
        tf_widget.max = 3.0
        ts = TransformStamped()
        ts.header.frame_id = "map"
        ts.child_frame_id = "floor"
        ts.transform.translation.x = 0.0
        ts.transform.translation.y = 0.0
        ts.transform.translation.z = 0.0
        roll = -math.pi * 0.5
        pitch = 0
        yaw = 0
        rot = tg.quaternion_from_euler(roll, pitch, yaw, 'sxyz')
        ts.transform.rotation.w = rot[0]
        ts.transform.rotation.x = rot[1]
        ts.transform.rotation.y = rot[2]
        ts.transform.rotation.z = rot[3]
        tf_widget.transform_stamped = ts
        tf_req = AddTfRequest()
        tf_req.tf = tf_widget
        self.tf_cli(tf_req)

        tf_widget = TfWidget()
        tf_widget.name = "sky tf"
        tf_widget.tab_name = tab_name
        tf_widget.window = req.name
        tf_widget.min = -3.0
        tf_widget.max = 3.0
        ts = TransformStamped()
        ts.header.frame_id = "floor"
        ts.child_frame_id = "sky"
        ts.transform.translation.x = 0.0
        ts.transform.translation.y = 0.0
        ts.transform.translation.z = 0.0
        roll = math.pi
        pitch = 0
        yaw = 0
        rot = tg.quaternion_from_euler(roll, pitch, yaw, 'sxyz')
        ts.transform.rotation.w = rot[0]
        ts.transform.rotation.x = rot[1]
        ts.transform.rotation.y = rot[2]
        ts.transform.rotation.z = rot[3]
        tf_widget.transform_stamped = ts
        tf_req = AddTfRequest()
        tf_req.tf = tf_widget
        self.tf_cli(tf_req)

        tf_widget = TfWidget()
        tf_widget.name = "projector1 pub"
        tf_widget.tab_name = tab_name
        tf_widget.window = req.name
        tf_widget.min = -3.0
        tf_widget.max = 3.0
        ts = TransformStamped()
        ts.header.frame_id = "map"
        ts.child_frame_id = "projector1"
        ts.transform.translation.x = 0.5
        ts.transform.translation.y = -0.43
        ts.transform.translation.z = -1.0
        roll = -3.08
        pitch = -0.02
        yaw = 0.0
        rot = tg.quaternion_from_euler(roll, pitch, yaw, 'sxyz')
        ts.transform.rotation.w = rot[0]
        ts.transform.rotation.x = rot[1]
        ts.transform.rotation.y = rot[2]
        ts.transform.rotation.z = rot[3]
        tf_widget.transform_stamped = ts
        tf_req = AddTfRequest()
        tf_req.tf = tf_widget
        self.tf_cli(tf_req)

        tf_widget = TfWidget()
        tf_widget.name = "map pub tf 2"
        tf_widget.tab_name = tab_name
        tf_widget.window = req.name
        tf_widget.min = -3.0
        tf_widget.max = 3.0
        ts = TransformStamped()
        ts.header.frame_id = "floor"
        ts.child_frame_id = "bar2"
        ts.transform.translation.x = 1.0
        ts.transform.translation.y = -0.0
        ts.transform.translation.z = -1.0
        roll = 0.0
        pitch = 0
        yaw = 0
        rot = tg.quaternion_from_euler(roll, pitch, yaw, 'sxyz')
        ts.transform.rotation.w = rot[0]
        ts.transform.rotation.x = rot[1]
        ts.transform.rotation.y = rot[2]
        ts.transform.rotation.z = rot[3]
        tf_widget.transform_stamped = ts
        tf_req = AddTfRequest()
        tf_req.tf = tf_widget
        self.tf_cli(tf_req)

        # TODO(lucasw) move the tf broadcasting into standalone node
        if False:
            self.elapsed = 0.0
            self.period = 0.05
            self.br = tf2_ros.TransformBroadcaster()
            self.timer = self.create_timer(self.period, self.update)

    def add_markers(self):
        self.marker_pub = rospy.Publisher('marker', Marker, queue_size=3)
        rospy.sleep(1.0)

        # now publish some markers for the Viz2D
        marker = Marker()
        marker.ns = "test"
        marker.id = 0
        marker.header.frame_id = "bar"
        marker.scale.x = 1.0
        marker.scale.y = 0.5
        marker.color.r = 0.8
        marker.color.b = 0.6
        marker.color.a = 1.0
        marker.pose.position.x = 0.5
        marker.pose.position.y = 0.25
        marker.text = "bar marker"
        self.marker_pub.publish(marker)
        rospy.sleep(1.0)

        marker = Marker()
        marker.ns = "test"
        marker.id = 1
        marker.header.frame_id = "bar2"
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.color.r = 0.3
        marker.color.b = 0.6
        marker.color.g = 0.3
        marker.color.a = 1.0
        marker.text = "bar2 marker"
        self.marker_pub.publish(marker)
        rospy.sleep(1.0)

    def add_shapes(self):
        self.shape_pub = rospy.Publisher('shapes', TexturedShape, queue_size=3)
        rospy.sleep(1.0)

        shape = TexturedShape()
        shape.name = "foo"
        shape.header.frame_id = 'map'
        # shape.header.stamp = self.now()

        sc = 0.2
        off_y = 0.0
        num = 8
        off_x = -sc * num / 2
        num_rows = 5
        for j in range(num_rows):
            for i in range(num * 2 - 3):
                ind = i + len(shape.mesh.vertices)
                triangle = MeshTriangle()
                triangle.vertex_indices[0] = ind
                triangle.vertex_indices[1] = ind + 3
                triangle.vertex_indices[2] = ind + 2
                shape.mesh.triangles.append(triangle)
                triangle.vertex_indices[0] = ind
                triangle.vertex_indices[1] = ind + 1
                triangle.vertex_indices[2] = ind + 3
                shape.mesh.triangles.append(triangle)
            for i in range(num):
                pt = Point()
                pt.x = i * sc + off_x
                pt.y = off_y
                pt.z = 1.0 + j * sc * 2.0
                shape.mesh.vertices.append(pt)

        print("shape {} {} {}".format(shape.name, len(shape.mesh.vertices),
              len(shape.mesh.triangles) * 3))
        self.shape_pub.publish(shape)
        rospy.sleep(1.0)

    # TODO(lucasw) this isn't used
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


if __name__ == '__main__':
    rospy.init_node('demo_gui')
    demo = DemoGui()

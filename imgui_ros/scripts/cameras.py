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

import math
# TODO(lucasw) this doesn't exist in python yet?
# import tf2_ros
import rospy

from geometry_msgs.msg import Point, TransformStamped, Vector3
from imgui_ros_msgs.msg import TexturedShape, TfWidget, Vertex
from imgui_ros_msgs.srv import AddCamera, AddCameraRequest, AddCubeCamera, AddCubeCameraRequest
from imgui_ros_msgs.srv import AddShape, AddShapeRequest, AddTf, AddTfRequest
from shape_msgs.msg import MeshTriangle, Mesh
from std_msgs.msg import ColorRGBA
from time import sleep
from transforms3d import _gohlketransforms as tg
from visualization_msgs.msg import Marker


def vector3_len(vec):
    return math.sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z)

class Cameras:
    def __init__(self):
        self.add_cameras()
        # self.add_cube_cameras()
        self.add_gui()

    def add_gui(self):
        rospy.wait_for_service('add_tf')
        self.tf_cli = rospy.ServiceProxy('add_tf', AddTf)

        # TODO(lucasw) what if this window doesn't exist yet?
        # Should be added automatically.
        req_name = "misc controls"
        tab_name = 'tf'

        tf_widget = TfWidget()
        tf_widget.name = "cube camera tf"
        tf_widget.window = req_name
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
        tf_widget.window = req_name
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

    def add_cameras(self):
        rospy.wait_for_service('add_camera')
        self.camera_cli = rospy.ServiceProxy('add_camera', AddCamera)

        if True:
            req = AddCameraRequest()
            req.camera.add = True
            req.camera.header.frame_id = 'camera1'
            # req.camera.header_frame_id = 'camera1'
            req.camera.name = 'camera1'
            req.camera.texture_name = 'camera1'
            req.camera.topic = 'camera1'
            req.camera.width = 256
            req.camera.height = 256
            req.camera.aov_y = 120.0
            req.camera.near = 0.02
            req.camera.far = 100.0
            req.camera.skip = 0
            req.camera.ros_pub = True

            try:
                resp = self.camera_cli(req)
                rospy.loginfo(resp)
            except rospy.service.ServiceException as e:
                rospy.logerr(e)

    def add_cube_cameras(self):
        rospy.wait_for_service('add_cube_camera')
        self.cube_camera_cli = rospy.ServiceProxy('add_cube_camera', AddCubeCamera)

        aspect = 1.0  # 4.0 / 2.0
        height = 768

        if True:
            req = AddCubeCameraRequest()
            req.face_width = 512
            req.camera.add = True
            req.camera.header.frame_id = 'cube_camera'
            req.camera.name = 'cube_camera1'
            req.camera.texture_name = 'cube_camera1'
            req.camera.topic = 'cube_camera1'
            req.camera.width = int(height * aspect)
            req.camera.height = height
            req.camera.aov_y = 90.0
            req.camera.near = 0.02
            req.camera.far = 100.0

            resp = self.cube_camera_cli(req)
            rospy.loginfo(resp)

        self.make_lenses(aspect=aspect)

    def make_lenses(self, aspect=1.0):
        rospy.wait_for_service('add_shape')
        self.shape_cli = rospy.ServiceProxy('add_shape', AddShape)

        req = AddShapeRequest()
        if False:
            shape = self.make_360_lens(name='cube_camera_lens',
                                       cols=32, rows=32, aspect=aspect)
        else:
            shape = self.make_polar_omnidirectional_lens(name='cube_camera_lens',
                                                         # segs_lat=64, segs_long=64, aspect=aspect)
                                                         segs_lat=12, segs_long=12, aspect=aspect)
        shape.header.frame_id = 'cube_camera_lens'
        req.shapes.append(shape)

        resp = self.shape_cli(req)
        rospy.loginfo(resp)
        sleep(1.0)

    def make_polar_omnidirectional_lens(self, name,
            segs_lat=16, segs_long=16,
            flip_normals=False, aspect=1.0):
        shape = TexturedShape()
        shape.add = True
        shape.enable = True
        shape.name = name
        # TODO(lucasw) laster support normal maps
        shape.header.frame_id = 'cube_camera_lens'
        shape.texture = 'default'
        shape.shininess_texture = 'default'
        shape.enable = False

        max_j = int(segs_lat * 2)  # int(segs_lat * 3 / 2)
        max_i = segs_long
        for j in range(max_j):
            fr_j = float(j) / float(segs_lat - 1)
            latitude = (fr_j - 0.5) * math.pi
            for i in range(max_i):
                fr_i = float(i) / float(segs_long - 1)
                longitude = fr_i * 2.0 * math.pi
                vertex = Vertex()
                clong = math.cos(longitude)
                slong = math.sin(longitude)
                vertex.vertex.x = aspect * fr_j * clong
                vertex.vertex.y = fr_j * slong
                vertex.vertex.z = -1.0

                clat = math.cos(latitude)
                slat = math.sin(latitude)
                z = clong * clat
                x = slong * clat
                y = slat

                vertex.normal.x = z
                vertex.normal.y = x
                vertex.normal.z = y

                vertex.uv.x = fr_i
                vertex.uv.y = fr_j

                val = 1.0
                vertex.color.r = val
                vertex.color.g = val
                vertex.color.b = val
                vertex.color.a = 1.0

                shape.vertices.append(vertex)
                shape = self.add_triangles(shape, i, j, max_i, max_j)

        return shape

    def make_360_lens(self, name, cols, rows, aspect=1.0, flip_normals=False):
        shape = TexturedShape()
        shape.add = True
        shape.enable = True
        shape.name = name
        # TODO(lucasw) laster support normal maps
        shape.header.frame_id = 'cube_camera_lens'
        shape.texture = 'default'
        shape.shininess_texture = 'default'
        shape.enable = False

        for i in range(cols):
            fr_x = float(i) / float(cols - 1)
            for j in range(rows):
                fr_y = float(j) / float(rows - 1)
                vertex = Vertex()
                vertex.vertex.x = (-1.0 + fr_x * 2.0) * aspect
                vertex.vertex.y = -1.0 + fr_y * 2.0
                vertex.vertex.z = -1.0

                longitude = fr_x * 2.0 * math.pi
                latitude = (fr_y - 0.5) * math.pi

                clong = math.cos(longitude)
                slong = math.sin(longitude)
                clat = math.cos(latitude)
                slat = math.sin(latitude)
                z = clong * clat
                x = slong * clat
                y = slat

                vertex.normal.x = x
                vertex.normal.y = y
                vertex.normal.z = z
                nrm_len = vector3_len(vertex.normal)
                vertex.normal.x /= nrm_len
                vertex.normal.y /= nrm_len
                vertex.normal.z /= nrm_len

                vertex.uv.x = fr_x  # % 1.0
                vertex.uv.y = fr_y  # % 1.0

                val = 1.0
                vertex.color.r = val
                vertex.color.g = val
                vertex.color.b = val
                vertex.color.a = 1.0

                shape.vertices.append(vertex)
                shape = self.add_triangles(shape, i, j, cols, rows, flip_normals)

        return shape

    def add_triangles(self, shape, i, j, cols, rows, flip_normals=False):
        if i >= cols - 1:
            return shape
        if j >= rows - 1:
            return shape
        # if len(shape.vertices) == 0:
        #     return shape
        ind0 = len(shape.vertices) - 1
        ind1 = ind0 + 1
        ind2 = ind0 + cols
        ind3 = ind0 + cols + 1

        # print("inds {} {} {} {}".format(ind0, ind1, ind2, ind3))
        triangle = MeshTriangle()
        triangle.vertex_indices[0] = ind0
        if flip_normals:
            triangle.vertex_indices[1] = ind3
            triangle.vertex_indices[2] = ind1
        else:
            triangle.vertex_indices[1] = ind1
            triangle.vertex_indices[2] = ind3
        shape.triangles.append(triangle)

        triangle = MeshTriangle()
        triangle.vertex_indices[0] = ind0
        if flip_normals:
            triangle.vertex_indices[1] = ind2
            triangle.vertex_indices[2] = ind3
        else:
            triangle.vertex_indices[1] = ind3
            triangle.vertex_indices[2] = ind2
        shape.triangles.append(triangle)
        return shape

if __name__ == '__main__':
    rospy.init_node('imgui_cameras')
    camera = Cameras()

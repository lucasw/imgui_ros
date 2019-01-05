#!/usr/bin/env python3
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

import argparse
import cv2
import cv_bridge
import math
# TODO(lucasW) this doesn't exist in python yet?
# import tf2_ros
import rclpy
import sys

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, TransformStamped, Vector3
from imgui_ros.msg import TexturedShape, Vertex, Widget
from imgui_ros.srv import AddCamera, AddCubeCamera, AddProjector, AddShaders
from imgui_ros.srv import AddShape, AddTexture, AddWindow
from rclpy.node import Node
from shape_msgs.msg import MeshTriangle, Mesh
from std_msgs.msg import ColorRGBA
from time import sleep
from visualization_msgs.msg import Marker


def vector3_len(vec):
    return math.sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z)

class Cameras(Node):

    def __init__(self):
        super().__init__('add_shape')
        # self.marker_pub = self.create_publisher(Marker, 'marker')
        # self.shape_pub = self.create_publisher(TexturedShape, 'shapes')
        sleep(1.0)

        self.cli = self.create_client(AddShape, 'add_shape')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('shape service not available, waiting again...')

        self.bridge = cv_bridge.CvBridge()

        parser = argparse.ArgumentParser(description='imgui_ros demo')
        # parser.add_argument('-nt', '--no-textures', dest='no_textures',  # type=bool,
        #         help='enable textures', action='store_true')  # , default=True)
        # parser.add_argument('-ns', '--no-shapes', dest='no_shapes',  # type=bool,
        #         help='enable shapes', action='store_true')  # , default=True)
        self.args, unknown = parser.parse_known_args(sys.argv)

    # TODO(lucasw) can't this be a callback instead?
    def wait_for_response(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                if self.future.result() is not None:
                    response = self.future.result()
                    self.get_logger().info(
                        'Result %s' % (str(response)))
                else:
                    self.get_logger().info(
                        'Service call failed %r' % (self.future.exception(),))
                break

    def run(self):
        self.add_cameras()
        self.add_cube_cameras()

    def add_cameras(self):
        self.camera_cli = self.create_client(AddCamera, 'add_camera')
        while not self.camera_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('camera service not available, waiting again...')

        if True:
            req = AddCamera.Request()
            req.camera.header.frame_id = 'camera1'
            req.camera.name = 'camera1'
            req.camera.texture_name = 'camera1'
            req.camera.topic = 'camera1'
            req.camera.width = 256
            req.camera.height = 256
            req.camera.aov_y = 120.0
            self.future = self.camera_cli.call_async(req)
            self.wait_for_response()

    def add_cube_cameras(self):
        self.cube_camera_cli = self.create_client(AddCubeCamera, 'add_cube_camera')
        while not self.cube_camera_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('cube camera service not available, waiting again...')

        aspect = 4.0 / 2.0
        height = 400

        if True:
            req = AddCubeCamera.Request()
            req.camera.header.frame_id = 'cube_camera'
            req.camera.name = 'cube_camera1'
            req.camera.texture_name = 'cube_camera1'
            req.camera.topic = 'cube_camera1'
            req.camera.width = int(height * aspect)
            req.camera.height = height
            req.camera.aov_y = 90.0
            self.future = self.cube_camera_cli.call_async(req)
            self.wait_for_response()
        self.make_lenses(aspect=aspect)

    def make_lenses(self, aspect=1.0):
        req = AddShape.Request()
        self.shape_cli = self.create_client(AddShape, 'add_shape')
        while not self.shape_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('shape service not available, waiting again...')

        shape = self.make_360_lens(name='cube_camera_lens',
                                   cols=32, rows=32, aspect=aspect)
        shape.header.frame_id = 'cube_camera_lens'
        req.shapes.append(shape)

        self.future = self.shape_cli.call_async(req)
        self.wait_for_response()
        sleep(1.0)

    def make_360_lens(self, name, cols, rows, aspect=1.0, flip_normals=False):
        shape = TexturedShape()
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

                vertex.uv.x = fr_x
                vertex.uv.y = fr_y

                val = 1.0
                vertex.color.r = val
                vertex.color.g = val
                vertex.color.b = val
                vertex.color.a = 1.0

                shape.vertices.append(vertex)

                ind0 = len(shape.vertices) - 1
                ind1 = ind0 + 1
                ind2 = ind0 + cols
                ind3 = ind0 + cols + 1
                if (i < cols - 1) and (j < rows - 1):
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

def main(args=None):
    rclpy.init(args=args)

    try:
        demo = Cameras()
        demo.run()
        rclpy.spin(demo)
    finally:
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

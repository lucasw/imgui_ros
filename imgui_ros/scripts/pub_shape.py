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
from imgui_ros.srv import AddCamera, AddProjector, AddShaders, AddShape, AddTexture, AddWindow
from rclpy.node import Node
from shape_msgs.msg import MeshTriangle, Mesh
from std_msgs.msg import ColorRGBA
from time import sleep
from visualization_msgs.msg import Marker


def vector3_len(vec):
    return math.sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z)

class Demo(Node):

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
        parser.add_argument('-nt', '--no-textures', dest='no_textures',  # type=bool,
                help='enable textures', action='store_true')  # , default=True)
        parser.add_argument('-ns', '--no-shapes', dest='no_shapes',  # type=bool,
                help='enable shapes', action='store_true')  # , default=True)
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
        # print(self.args.no_textures)
        # print(self.args.no_shapes)
        if not self.args.no_textures:
            self.add_texture('diffract', 'image_manip', 'diffract1.png')
            self.add_texture('chess', 'image_manip', 'chess.png')
            self.add_texture('gradient_radial', 'image_manip', 'gradient_radial.png')
            # self.add_texture('projected_texture', 'image_manip', 'plasma.png')
        if not self.args.no_shapes:
            # TODO(lucasw) there is something wrong with storing new shapes
            # on top of old ones, all the shapes disappear until
            # add_shaders is run again.
            self.add_shapes()
        self.add_cameras()
        self.add_projectors()

    def add_projectors(self):
        self.projector_cli = self.create_client(AddProjector, 'add_projector')
        while not self.projector_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('projector service not available, waiting again...')

        req = AddProjector.Request()
        req.projector.camera.header.frame_id = 'projector1'
        req.projector.camera.name = 'projector1'
        req.projector.camera.texture_name = 'gradient_radial'
        req.projector.camera.aov_y = 100.0
        req.projector.camera.aov_x = 140.0
        req.projector.constant_attenuation = 0.2
        self.future = self.projector_cli.call_async(req)
        self.wait_for_response()

        if False:
            req = AddProjector.Request()
            req.projector.camera.header.frame_id = 'bar2'
            req.projector.camera.name = 'projector2'
            req.projector.camera.texture_name = 'chess'
            req.projector.camera.aov_y = 30.0
            req.projector.camera.aov_x = 30.0
            self.future = self.projector_cli.call_async(req)
            self.wait_for_response()

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

        # this is getting framebuffer not complete errors
        if True:
            req = AddCamera.Request()
            req.camera.header.frame_id = 'camera2'
            req.camera.name = 'camera2'
            req.camera.texture_name = 'camera2'
            # req.camera.topic = 'camera2'
            req.camera.width = 256
            req.camera.height = 256
            req.camera.aov_y = 20.0
            self.future = self.camera_cli.call_async(req)
            self.wait_for_response()

    def add_texture(self, name='diffract', pkg_name='image_manip', image_name='diffract1.png'):
        self.texture_cli = self.create_client(AddTexture, 'add_texture')
        while not self.texture_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # imread an image
        image_dir = get_package_share_directory(pkg_name)
        # print('image dir ' + image_dir)
        image = cv2.imread(image_dir + "/data/" + image_name, 1)
        # cv2.imshow("image", image)
        # cv2.waitKey(0)
        image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        req = AddTexture.Request()
        req.name = name
        req.image = image

        self.future = self.texture_cli.call_async(req)
        self.wait_for_response()

    # TODO(lucasw) later bring in icosphere code from bullet_server
    def make_sphere(self, name='sphere',
            radius_x=1.0, radius_y=1.0, radius_z=1.0,
            segs_long=16, segs_lat=8,
            off_x=0.0, off_y=0.0, off_z=0.0,
            repeat=1.0,
            flip_normals=False):
        shape = TexturedShape()
        shape.name = name
        shape.header.frame_id = 'bar2'
        shape.texture = 'diffract'

        for j in range(segs_lat):
            fr_y = float(j) / float(segs_lat - 1)
            latitude = (fr_y - 0.5) * math.pi
            for i in range(segs_long):
                fr_x = float(i) / float(segs_long - 1)
                longitude = fr_x * 2.0 * math.pi

                # print("{} {}".format(i, theta))
                clong = math.cos(longitude)
                slong = math.sin(longitude)
                clat = math.cos(latitude)
                slat = math.sin(latitude)
                x = radius_x * clong * clat
                y = radius_y * slong * clat
                z = radius_z * slat

                ind0 = len(shape.vertices)
                ind1 = ind0 + 1
                ind2 = ind0 + segs_long
                ind3 = ind0 + segs_long + 1
                # connect to start, but can't reuse vertices because of uv
                if i < segs_long - 1 and j < segs_lat - 1:
                    if j != 0:
                        triangle = MeshTriangle()
                        triangle.vertex_indices[0] = ind0
                        triangle.vertex_indices[1] = ind1
                        triangle.vertex_indices[2] = ind3
                        shape.triangles.append(triangle)

                    if j != segs_lat - 2:
                        triangle = MeshTriangle()
                        triangle.vertex_indices[0] = ind0
                        triangle.vertex_indices[1] = ind3
                        triangle.vertex_indices[2] = ind2
                        shape.triangles.append(triangle)

                vertex = Vertex()
                vertex.vertex.x = x + off_x
                vertex.vertex.y = y + off_y
                vertex.vertex.z = z + off_z

                vertex.normal.x = x
                vertex.normal.y = y
                vertex.normal.z = z
                nrm_len = vector3_len(vertex.normal)
                vertex.normal.x /= nrm_len
                vertex.normal.y /= nrm_len
                vertex.normal.z /= nrm_len
                if flip_normals:
                    vertex.normal.x *= -1.0
                    vertex.normal.y *= -1.0
                    vertex.normal.z *= -1.0

                vertex.uv.x = repeat * fr_x
                vertex.uv.y = repeat * fr_y

                val = 0.95
                vertex.color.r = val
                vertex.color.g = val
                vertex.color.b = val
                vertex.color.a = 1.0
                shape.vertices.append(vertex)

        return shape

    def make_cylinder(self, name='cylinder', radius=0.5, length=2.5, segs=16,
            off_x=0.0, off_y=0.0):
        shape = TexturedShape()
        shape.name = name
        shape.header.frame_id = 'bar2'
        shape.texture = 'diffract'

        repeat = 4.0
        for i in range(segs):
            fr = float(i) / float(segs - 1)
            theta = fr * 2.0 * math.pi
            # print("{} {}".format(i, theta))
            x = radius * math.cos(theta)
            y = radius * math.sin(theta)

            ind0 = len(shape.vertices)
            ind1 = ind0 + 1
            ind2 = ind0 + 2
            ind3 = ind0 + 3
            # connect to start, but can't reuse vertices because of uv
            if i < segs - 1:
                triangle = MeshTriangle()
                triangle.vertex_indices[0] = ind0
                triangle.vertex_indices[1] = ind3
                triangle.vertex_indices[2] = ind1
                shape.triangles.append(triangle)

                if True:
                    triangle = MeshTriangle()
                    triangle.vertex_indices[0] = ind0
                    triangle.vertex_indices[1] = ind2
                    triangle.vertex_indices[2] = ind3
                    shape.triangles.append(triangle)

            vertex = Vertex()
            vertex.vertex.x = x + off_x
            vertex.vertex.y = y + off_y
            vertex.vertex.z = -length * 0.5

            vertex.normal.x = x
            vertex.normal.y = y
            vertex.normal.z = 0.0
            nrm_len = vector3_len(vertex.normal)
            vertex.normal.x /= nrm_len
            vertex.normal.y /= nrm_len
            vertex.normal.z /= nrm_len

            vertex.uv.x = repeat * fr
            vertex.uv.y = 0.0

            val = 0.95
            vertex.color.r = val
            vertex.color.g = val
            vertex.color.b = val
            vertex.color.a = 1.0
            shape.vertices.append(vertex)

            vertex = Vertex()
            vertex.vertex.x = x + off_x
            vertex.vertex.y = y + off_y
            vertex.vertex.z = length * 0.5

            vertex.normal.x = x
            vertex.normal.y = y
            vertex.normal.z = 0.0
            nrm_len = vector3_len(vertex.normal)
            vertex.normal.x /= nrm_len
            vertex.normal.y /= nrm_len
            vertex.normal.z /= nrm_len

            vertex.uv.x = repeat * fr
            vertex.uv.y = repeat

            vertex.color.r = val
            vertex.color.g = val
            vertex.color.b = val
            vertex.color.a = 1.0
            shape.vertices.append(vertex)

        return shape

    def make_planes(self):
        shape = TexturedShape()
        shape.name = "foo"
        shape.header.frame_id = 'bar2'
        shape.texture = 'diffract'
        # shape.header.stamp = self.now()

        sc = 0.2
        num = 7
        off_x = -sc * num / 2
        num_rows = 4
        off_y = -sc
        off_z = -sc * num_rows
        for j in range(num_rows):
            for i in range(0, num * 2 - 3, 2):
                ind = i + len(shape.vertices)
                triangle = MeshTriangle()
                triangle.vertex_indices[0] = ind
                triangle.vertex_indices[1] = ind + 1
                triangle.vertex_indices[2] = ind + 3
                shape.triangles.append(triangle)

                if True:
                    triangle = MeshTriangle()
                    triangle.vertex_indices[0] = ind
                    triangle.vertex_indices[1] = ind + 3
                    triangle.vertex_indices[2] = ind + 2
                    shape.triangles.append(triangle)

            for i in range(num):
                x = i * sc + off_x
                z = j * sc * 2.0 + off_z

                vertex = Vertex()
                vertex.vertex.x = x
                vertex.vertex.y = off_y
                vertex.vertex.z = z

                fr = float(i) / float(num)
                vertex.uv.x = fr
                vertex.uv.y = 0.0

                vertex.color.r = float(j) / float(num_rows)
                vertex.color.g = 1.0
                vertex.color.b = float(i) / float(num)
                vertex.color.a = 1.0
                shape.vertices.append(vertex)

                vertex = Vertex()
                vertex.vertex.x = x
                vertex.vertex.y = off_y + sc * 2.0
                vertex.vertex.z = z

                vertex.uv.x = fr
                vertex.uv.y = 1.0

                vertex.color.r = float(j) / float(num_rows)
                vertex.color.g = 0.3
                vertex.color.b = 1.0 - float(i) / float(num)
                vertex.color.a = 1.0
                shape.vertices.append(vertex)

        self.get_logger().info("shape {} {} {}".format(shape.name, len(shape.vertices),
              len(shape.triangles) * 3))
        # self.shape_pub.publish(shape)
        shape.add = True
        return shape

    def add_shapes(self):
        req = AddShape.Request()
        if False:
            shape = self.make_planes()
            shape.add = True
            req.shapes.append(shape)
        if True:
            shape = self.make_sphere(name='big_sphere',
                radius_x=10.0, radius_y=10.0, radius_z=10.0,
                segs_long=16, segs_lat=8,
                flip_normals=True,
                )
            shape.add = True
            shape.texture = 'diffract'
            shape.header.frame_id = 'map'
            req.shapes.append(shape)
        if True:
            shape = self.make_cylinder(name='cylinder2', radius=0.03, length=0.1,
                segs=20,
                off_y=0.0)
            shape.add = True
            shape.texture = 'gradient_radial'
            shape.header.frame_id = 'projector1'
            req.shapes.append(shape)
        if True:
            shape = self.make_cylinder(name='cylinder3', segs=25)
            shape.add = True
            shape.texture = 'diffract'  # 'camera1'
            shape.header.frame_id = 'bar2'
            req.shapes.append(shape)
        self.future = self.cli.call_async(req)
        self.wait_for_response()
        sleep(1.0)

def main(args=None):
    rclpy.init(args=args)

    try:
        demo = Demo()
        demo.run()
        rclpy.spin(demo)
    finally:
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

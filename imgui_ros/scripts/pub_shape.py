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

import cv2
import cv_bridge
import math
# TODO(lucasW) this doesn't exist in python yet?
# import tf2_ros
import rclpy

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, TransformStamped, Vector3
from imgui_ros.msg import TexturedShape, Widget
from imgui_ros.srv import AddShape, AddTexture, AddWindow
from rclpy.node import Node
from shape_msgs.msg import MeshTriangle, Mesh
from std_msgs.msg import ColorRGBA
from time import sleep
from visualization_msgs.msg import Marker


class Demo(Node):

    def __init__(self):
        super().__init__('add_shape')
        # self.marker_pub = self.create_publisher(Marker, 'marker')
        # self.shape_pub = self.create_publisher(TexturedShape, 'shapes')
        sleep(1.0)
        self.cli = self.create_client(AddShape, 'add_shape')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.bridge = cv_bridge.CvBridge()

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
        self.add_texture('diffract', 'image_manip', 'diffract1.png')
        self.add_texture('projected_texture', 'image_manip', 'maze1.png')
        self.add_shapes()

    def add_texture(self, name='diffract', pkg_name='image_manip', image_name='diffract1.png'):
        self.texture_cli = self.create_client(AddTexture, 'add_texture')
        while not self.texture_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # imread an image
        image_dir = get_package_share_directory(pkg_name)
        print('image_manip dir ' + image_dir)
        image = cv2.imread(image_dir + "/data/" + image_name, 1)
        # cv2.imshow("image", image)
        # cv2.waitKey(0)
        image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        req = AddTexture.Request()
        req.name = name
        req.image = image

        self.future = self.texture_cli.call_async(req)
        self.wait_for_response()

    def make_cylinder(self, name='cylinder', radius=0.5, length=2.5, segs=16):
        shape = TexturedShape()
        shape.name = name
        shape.header.frame_id = 'bar2'
        shape.texture = 'diffract'

        for i in range(segs):
            fr = float(i) / float(segs - 1)
            theta = fr * 2.0 * math.pi
            # print("{} {}".format(i, theta))
            x = radius * math.cos(theta)
            y = radius * math.sin(theta)

            ind0 = len(shape.mesh.vertices)
            ind1 = ind0 + 1
            ind2 = ind0 + 2
            ind3 = ind0 + 3
            # connect to start, but can't reuse vertices because of uv
            if i < segs - 1:
                triangle = MeshTriangle()
                triangle.vertex_indices[0] = ind0
                triangle.vertex_indices[1] = ind3
                triangle.vertex_indices[2] = ind1
                shape.mesh.triangles.append(triangle)

                if True:
                    triangle = MeshTriangle()
                    triangle.vertex_indices[0] = ind0
                    triangle.vertex_indices[1] = ind2
                    triangle.vertex_indices[2] = ind3
                    shape.mesh.triangles.append(triangle)

            pt = Point()
            pt.x = x
            pt.y = y
            pt.z = -length * 0.5
            shape.mesh.vertices.append(pt)

            uv = Vector3()
            uv.x = fr
            uv.y = 0.0
            shape.uv.append(uv)

            col = ColorRGBA()
            col.r = 0.0
            col.g = 1.0
            col.b = 0.0
            col.a = 1.0
            shape.colors.append(col);

            pt = Point()
            pt.x = x
            pt.y = y
            pt.z = length * 0.5
            shape.mesh.vertices.append(pt)

            uv = Vector3()
            uv.x = fr
            uv.y = 1.0
            shape.uv.append(uv)

            col = ColorRGBA()
            col.r = 0.0
            col.g = 1.0
            col.b = 0.0
            col.a = 1.0
            shape.colors.append(col);

        return shape

    def make_planes(self):
        shape = TexturedShape()
        shape.name = "foo"
        shape.header.frame_id = 'bar'
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
                ind = i + len(shape.mesh.vertices)
                triangle = MeshTriangle()
                triangle.vertex_indices[0] = ind
                triangle.vertex_indices[1] = ind + 1
                triangle.vertex_indices[2] = ind + 3
                shape.mesh.triangles.append(triangle)

                if True:
                    triangle = MeshTriangle()
                    triangle.vertex_indices[0] = ind
                    triangle.vertex_indices[1] = ind + 3
                    triangle.vertex_indices[2] = ind + 2
                    shape.mesh.triangles.append(triangle)

            for i in range(num):
                x = i * sc + off_x
                z = j * sc * 2.0 + off_z
                pt = Point()
                pt.x = x
                pt.y = off_y
                pt.z = z
                shape.mesh.vertices.append(pt)

                fr = float(i) / float(num)
                uv = Vector3()
                uv.x = fr
                uv.y = 0.0
                shape.uv.append(uv)

                col = ColorRGBA()
                col.r = float(j) / float(num_rows)
                col.g = 1.0
                col.b = float(i) / float(num)
                col.a = 1.0
                shape.colors.append(col);

                pt = Point()
                pt.x = x
                pt.y = off_y + sc * 2.0
                pt.z = z
                shape.mesh.vertices.append(pt)

                uv = Vector3()
                uv.x = fr
                uv.y = 1.0
                shape.uv.append(uv)

                col = ColorRGBA()
                col.r = float(j) / float(num_rows)
                col.g = 0.3
                col.b = 1.0 - float(i) / float(num)
                col.a = 1.0
                shape.colors.append(col);

        print("shape {} {} {}".format(shape.name, len(shape.mesh.vertices),
              len(shape.mesh.triangles) * 3))
        # self.shape_pub.publish(shape)
        shape.add = True
        return shape

    def add_shapes(self):
        req = AddShape.Request()
        shape = self.make_planes()
        shape.add = False
        req.shapes.append(shape)
        shape = self.make_cylinder()
        req.shapes.append(shape)
        self.future = self.cli.call_async(req)
        self.wait_for_response()
        sleep(1.0)

def main(args=None):
    rclpy.init(args=args)

    demo = Demo()
    demo.run()

    demo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

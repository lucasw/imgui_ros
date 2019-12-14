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

import cv2
import cv_bridge
import math
import rospkg
import rospy

from geometry_msgs.msg import Point, TransformStamped, Vector3
from imgui_ros_msgs.msg import TexturedShape, Vertex, Widget
from imgui_ros_msgs.srv import AddProjector, AddProjectorRequest
from imgui_ros_msgs.srv import AddShape, AddShapeRequest, AddTexture, AddTextureRequest
from shape_msgs.msg import MeshTriangle, Mesh
from visualization_msgs.msg import Marker


def vector3_len(vec):
    return math.sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z)

class PubShape:
    def __init__(self):
        no_textures = rospy.get_param('~no_textures', False)
        no_shapes = rospy.get_param('~no_shapes', False)
        self.rospack = rospkg.RosPack()
        self.run(no_textures, no_shapes)

    def run(self, no_textures=False, no_shapes=False):
        rospy.wait_for_service('add_shape')
        self.cli = rospy.ServiceProxy('add_shape', AddShape)

        rospy.wait_for_service('add_texture')
        self.texture_cli = rospy.ServiceProxy('add_texture', AddTexture)

        self.bridge = cv_bridge.CvBridge()

        # print(no_textures)
        # print(no_shapes)
        if not no_textures:
            self.add_texture('default', 'imgui_ros', 'black.png')
            self.add_texture('white', 'imgui_ros', 'white.png')
            self.add_texture('gradient', 'imgui_ros', 'gradient.png')
            self.add_texture('square', 'imgui_ros', 'square.png')
            self.add_texture('diffract', 'vimjay', 'diffract1.png')
            self.add_texture('chess', 'vimjay', 'chess.png')
            self.add_texture('gradient_radial', 'vimjay', 'gradient_radial.png')
            # self.add_texture('projected_texture', 'vimjay', 'plasma.png')
        if not no_shapes:
            # TODO(lucasw) there is something wrong with storing new shapes
            # on top of old ones, all the shapes disappear until
            # add_shaders is run again.
            self.add_shapes()
        self.add_projectors()

    def add_projectors(self):
        rospy.wait_for_service('add_projector')
        self.projector_cli = rospy.ServiceProxy('add_projector', AddProjector)

        req = AddProjectorRequest()
        req.projector.camera.header.frame_id = 'projector1'
        req.projector.camera.name = 'projector1'
        req.projector.camera.texture_name = 'gradient_radial'
        req.projector.camera.aov_y = 130.0
        req.projector.camera.aov_x = 130.0
        req.projector.constant_attenuation = 0.2
        req.projector.quadratic_attenuation = 0.004
        self.projector_cli(req)

        if True:
            req = AddProjectorRequest()
            # req.projector.remove = False
            req.projector.camera.header.frame_id = 'bar2'
            req.projector.camera.name = 'projector2'
            req.projector.camera.texture_name = 'chess'
            req.projector.camera.aov_y = 130.0
            req.projector.camera.aov_x = 100.0
            self.projector_cli(req)

    def add_texture(self, name='diffract', pkg_name='image_manip', image_name='diffract1.png'):
        # imread an image
        image_dir = self.rospack.get_path(pkg_name)
        # print('image dir ' + image_dir)
        full_path = image_dir + "/data/" + image_name
        image = cv2.imread(full_path, 1)
        if image is None:
            rospy.logerr("Couldn't read image '{}'".format(full_path))
            return
        rospy.loginfo(type(image))
        # cv2.imshow("image", image)
        # cv2.waitKey(0)
        image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        req = AddTextureRequest()
        req.name = name
        req.image = image
        req.wrap_s = AddTextureRequest.REPEAT
        req.wrap_t = AddTextureRequest.REPEAT

        self.texture_cli(req)

    # TODO(lucasw) later bring in icosphere code from bullet_server
    def make_sphere(self, name='sphere',
            radius_x=1.0, radius_y=1.0, radius_z=1.0,
            segs_long=16, segs_lat=8,
            segs_long_stop=0, segs_lat_stop=0,
            off_x=0.0, off_y=0.0, off_z=0.0,
            color_r=1.0, color_g=1.0, color_b=1.0,
            repeat_x=1.0,
            repeat_y=1.0,
            flip_normals=False):
        if segs_long_stop <= 0:
            segs_long_stop = segs_long
        if segs_lat_stop <= 0:
            segs_lat_stop = segs_lat

        shape = TexturedShape()
        shape.name = name
        shape.header.frame_id = 'bar2'
        shape.texture = 'gradient'
        shape.shininess_texture = 'default'
        shape.emission_texture = 'default'

        for j in range(segs_lat_stop):
            fr_y = float(j) / float(segs_lat - 1)
            latitude = (fr_y - 0.5) * math.pi
            for i in range(segs_long_stop):
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
                ind2 = ind0 + segs_long_stop
                ind3 = ind0 + segs_long_stop + 1
                # connect to start, but can't reuse vertices because of uv
                if i < segs_long_stop - 1 and j < segs_lat_stop - 1:
                    if j != 0:
                        triangle = MeshTriangle()
                        triangle.vertex_indices[0] = ind0
                        if flip_normals:
                            triangle.vertex_indices[1] = ind3
                            triangle.vertex_indices[2] = ind1
                        else:
                            triangle.vertex_indices[1] = ind1
                            triangle.vertex_indices[2] = ind3
                        shape.triangles.append(triangle)

                    if j != segs_lat - 2:
                        triangle = MeshTriangle()
                        triangle.vertex_indices[0] = ind0
                        if flip_normals:
                            triangle.vertex_indices[1] = ind2
                            triangle.vertex_indices[2] = ind3
                        else:
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

                vertex.uv.x = repeat_x * fr_x
                vertex.uv.y = repeat_y * fr_y

                vertex.color.r = color_r
                vertex.color.g = color_g
                vertex.color.b = color_b
                vertex.color.a = 1.0
                shape.vertices.append(vertex)

        return shape

    def make_cylinder(self, name='cylinder', radius=0.5, length=2.5, segs=16,
            off_x=0.0, off_y=0.0, flip_normals=False):
        shape = TexturedShape()
        shape.name = name
        shape.header.frame_id = 'bar2'
        shape.texture = 'diffract'
        shape.shininess_texture = 'diffract'
        shape.emission_texture = 'default'

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
                # the triangle index order is what opengl uses for face culling
                if flip_normals:
                    triangle.vertex_indices[1] = ind1
                    triangle.vertex_indices[2] = ind3
                else:
                    triangle.vertex_indices[1] = ind3
                    triangle.vertex_indices[2] = ind1
                shape.triangles.append(triangle)

                if True:
                    triangle = MeshTriangle()
                    triangle.vertex_indices[0] = ind0
                    if flip_normals:
                        triangle.vertex_indices[1] = ind3
                        triangle.vertex_indices[2] = ind2
                    else:
                        triangle.vertex_indices[1] = ind2
                        triangle.vertex_indices[2] = ind3
                    shape.triangles.append(triangle)

            vertex = Vertex()
            vertex.vertex.x = x + off_x
            vertex.vertex.y = y + off_y
            vertex.vertex.z = -length * 0.5

            sc = 1.0
            if flip_normals:
                sc = -1.0
            vertex.normal.x = sc * x
            vertex.normal.y = sc * y
            vertex.normal.z = sc * 0.0
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

            vertex.normal.x = sc * x
            vertex.normal.y = sc * y
            vertex.normal.z = sc * 0.0
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

    def make_plane(self, off_x=0.0, off_y=0.0, off_z=0.0):
        shape = TexturedShape()
        shape.name = 'big_plane'
        shape.header.frame_id = 'map'
        shape.texture = 'square'
        shape.shininess_texture = 'square'
        shape.emission_texture = 'default'
        # shape.header.stamp = self.now()

        texture_sc = 64.0
        sc = 30.0
        num_rows = 8
        num_cols = 8
        for j in range(num_rows):
            for i in range(num_cols):
                ind = len(shape.vertices)
                if (j < num_rows - 1) and (i < num_cols - 1):
                    triangle = MeshTriangle()
                    triangle.vertex_indices[0] = ind
                    triangle.vertex_indices[1] = ind + num_cols + 1
                    triangle.vertex_indices[2] = ind + 1
                    shape.triangles.append(triangle)

                    if True:
                        triangle = MeshTriangle()
                        triangle.vertex_indices[0] = ind
                        triangle.vertex_indices[1] = ind + num_cols
                        triangle.vertex_indices[2] = ind + num_cols + 1
                        shape.triangles.append(triangle)

                x = i * sc + off_x - sc * num_cols * 0.5
                y = off_y
                z = j * sc + off_z - sc * num_rows * 0.5

                vertex = Vertex()
                vertex.vertex.x = x
                vertex.vertex.y = y
                vertex.vertex.z = z

                vertex.normal.x = 0.0
                vertex.normal.y = 1.0
                vertex.normal.z = 0.0

                fr_x = float(i) / float(num_cols - 1)
                fr_y = float(j) / float(num_rows - 1)
                vertex.uv.x = fr_x * texture_sc
                vertex.uv.y = fr_y * texture_sc

                vertex.color.r = 1.0
                vertex.color.g = 1.0
                vertex.color.b = 1.0
                vertex.color.a = 1.0
                shape.vertices.append(vertex)

        rospy.loginfo("shape {} {} {}".format(shape.name, len(shape.vertices),
                                              len(shape.triangles) * 3))
        # self.shape_pub.publish(shape)
        shape.add = True
        return shape

    def add_shapes(self):
        req = AddShapeRequest()
        if True:
            shape = self.make_plane(off_y=-3.0)
            shape.add = True
            req.shapes.append(shape)
        if True:
            shape = self.make_sphere(name='big_sphere',
                radius_x=60.0, radius_y=60.0, radius_z=60.0,
                segs_long=24,
                segs_long_stop=0,
                segs_lat=16,
                segs_lat_stop=9,
                repeat_y=1.0,
                color_r=0.6,
                color_g=0.6,
                color_b=1.0,
                flip_normals=True,
                )
            shape.add = True
            shape.texture = 'gradient'
            shape.shininess_texture = 'gradient'
            shape.emission_texture = 'default'
            shape.header.frame_id = 'sky'
            req.shapes.append(shape)
        if True:
            shape = self.make_cylinder(name='cylinder2', radius=0.1, length=0.1,
                segs=20,
                off_y=0.0)
            shape.add = True
            shape.texture = 'gradient_radial'
            shape.shininess_texture = 'gradient_radial'
            shape.header.frame_id = 'projector1'
            req.shapes.append(shape)
        if True:
            radius = 0.5
            shape = self.make_cylinder(name='cylinder3', length=2.0,
                                       radius=radius, segs=24)
            shape.add = True
            shape.texture = 'diffract'  # 'camera1'
            shape.shininess_texture = 'gradient_radial'
            shape.header.frame_id = 'bar2'
            req.shapes.append(shape)
            # inside of cylinder
            shape = self.make_cylinder(name='cylinder3_inside', length=2.0,
                                       radius=radius * 0.95, segs=12,
                                       flip_normals=True)
            shape.add = True
            shape.texture = 'diffract'  # 'camera1'
            shape.shininess_texture = 'gradient_radial'
            shape.emission_texture = 'square'
            shape.header.frame_id = 'bar2'
            req.shapes.append(shape)

        self.cli(req)

if __name__ == '__main__':
    rospy.init_node('pub_shape')
    pub_shape = PubShape()

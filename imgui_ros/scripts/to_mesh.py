#!/usr/bin/env python
# sudo apt install python-pyassimp

import cv2
import cv_bridge
import pyassimp
import rospy

from imgui_ros_msgs.msg import TexturedShape, Vertex
from imgui_ros_msgs.srv import AddShape, AddShapeRequest, AddTexture, AddTextureRequest
from shape_msgs.msg import MeshTriangle


class ToMesh:
    def __init__(self):
        filename = rospy.get_param("~filename", None)
        name = rospy.get_param("~name", "mesh")
        frame = rospy.get_param("~frame", "map")
        texture_file = rospy.get_param("~texture", None)
        texture_name = rospy.get_param("~texture_name", "tex1")

        self.bridge = cv_bridge.CvBridge()

        rospy.wait_for_service('add_texture')
        self.texture_cli = rospy.ServiceProxy('add_texture', AddTexture)
        self.add_texture(texture_name, texture_file)

        scene = pyassimp.load(name)
        if len(scene.meshes) < 1:
            rospy.logerr("no meshes in {}".format(name))

        texture_req = AddTextureRequest()

        req = AddShapeRequest()
        # TODO(lucasw) need to be able to have multiple frames for same shape
        # then re-render instances at all of those frames.
        for ind, mesh in enumerate(scene.meshes):
            shape = TexturedShape()
            shape.header.frame_id = frame
            shape.add = True
            shape.enable = True
            # TODO(lucasw) does the assimp mesh have a name?
            shape.name = name + str(ind)
            # TODO(lucasw) what does this mean?  Is it for the textures?
            shape.is_topic = False
            shape.texture = texture_name
            print(mesh)
            print(dir(mesh))
            print(dir(mesh.material.contents))
            print(dir(mesh.material.properties))
            print(len(mesh.texturecoords[0]))
            # print(mesh.texturecoords)
            print(len(mesh.vertices))
            print(len(mesh.normals))
            print(len(mesh.colors))
            print(len(mesh.faces))
            for ind, vertex in enumerate(mesh.vertices):
                vtx = Vertex()
                vtx.vertex.x = vertex[0]
                vtx.vertex.y = vertex[1]
                vtx.vertex.z = vertex[2]
                vtx.normal.x = mesh.normals[ind][0]
                vtx.normal.y = mesh.normals[ind][1]
                vtx.normal.z = mesh.normals[ind][2]
                vtx.color.r = 1.0
                vtx.color.g = 1.0
                vtx.color.b = 1.0
                vtx.color.a = 1.0
                vtx.uv.x = mesh.texturecoords[0][ind][0]
                vtx.uv.y = mesh.texturecoords[0][ind][1]
                shape.vertices.append(vtx)
            for ind, face in enumerate(mesh.faces):
                tri = MeshTriangle()
                # print('{} {}'.format(ind, face))
                for i in range(3):
                    # print(type(face[i]))
                    tri.vertex_indices[i] = int(face[i])
                shape.triangles.append(tri)
            req.shapes.append(shape)

        rospy.wait_for_service('add_shape')
        self.cli = rospy.ServiceProxy('add_shape', AddShape)
        rv = self.cli(req)
        rospy.loginfo(rv)

    def add_texture(self, name, image_name):
        image = cv2.imread(image_name, 1)
        if image is None:
            rospy.logerr("Couldn't read image '{}'".format(image_name))
            return
        image = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        req = AddTextureRequest()
        req.name = name
        req.image = image
        req.wrap_s = AddTextureRequest.REPEAT
        req.wrap_t = AddTextureRequest.REPEAT

        rv = self.texture_cli(req)
        rospy.loginfo(rv)

if __name__ == '__main__':
    rospy.init_node('to_mesh')
    to_mesh = ToMesh()

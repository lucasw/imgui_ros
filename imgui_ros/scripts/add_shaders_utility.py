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
import math
import time
import rospkg
import rospy
import sys

from geometry_msgs.msg import Point, TransformStamped, Vector3
from imgui_ros_msgs.msg import TexturedShape, Widget
from imgui_ros_msgs.srv import AddShaders, AddShadersRequest
from shape_msgs.msg import MeshTriangle, Mesh
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


class AddShadersUtility():
    def __init__(self):
        rospy.wait_for_service('add_shaders')
        self.shaders_cli = rospy.ServiceProxy('add_shaders', AddShaders)

    def add(self, name, vertex_filename, fragment_filename):
        req = AddShadersRequest()
        req.name = name
        req.vertex = ''
        if vertex_filename != '':
            with open(vertex_filename) as vertex_file:
                req.vertex = vertex_file.read()
        if req.vertex == '':
            rospy.logerr("couldn't load vertex shader from: '{}'".format(vertex_filename))
            return False
        # print(req.vertex)
        # print('####################################################')
        req.fragment = ''
        if fragment_filename != '':
            with open(fragment_filename) as fragment_file:
                req.fragment = fragment_file.read()
        if req.vertex == '':
            rospy.logerr("couldn't load fragment shader from '{}'".format(fragment_filename))
            return False
        # print(req.fragment)
        resp = self.shaders_cli(req)
        rospy.loginfo(resp)

def add_default_shaders():
    # TODO(lucasw) put the shaders into a config/data dir
    rospack = rospkg.RosPack()
    shader_dir = rospack.get_path('imgui_ros') + '/config/'
    rospy.loginfo("loading shaders '{}'".format(shader_dir))
    if True:
        add_shaders = AddShadersUtility()
        add_shaders.add('default',
                        shader_dir + 'vertex.glsl',
                        shader_dir + 'fragment.glsl')
        add_shaders.add('depth',
                        shader_dir + 'depth_vertex.glsl',
                        shader_dir + 'depth_fragment.glsl')
        add_shaders.add('cube_map',
                        shader_dir + 'cube_camera_vertex.glsl',
                        shader_dir + 'cube_camera_fragment.glsl')

if __name__ == '__main__':
    rospy.init_node("add_shaders")
    add_default_shaders()

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
from imgui_ros.srv import AddShaders, AddShape, AddTexture, AddWindow
from rclpy.node import Node
from shape_msgs.msg import MeshTriangle, Mesh
from std_msgs.msg import ColorRGBA
from time import sleep
from visualization_msgs.msg import Marker


class AddShadersNode(Node):

    def __init__(self):
        super().__init__('add_shaders')
        # self.marker_pub = self.create_publisher(Marker, 'marker')
        # self.shape_pub = self.create_publisher(TexturedShape, 'shapes')
        self.shaders_cli = self.create_client(AddShaders, 'add_shaders')
        while not self.shaders_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.bridge = cv_bridge.CvBridge()
        sleep(1.0)

    # TODO(lucasw) can't this be a callback instead?
    def wait_for_response(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                if self.future.result() is not None:
                    response = self.future.result()
                    self.get_logger().info(
                        'Result: %s success: %s' % (response.message, str(response.success)))
                else:
                    self.get_logger().info(
                        'Service call failed %r' % (self.future.exception(),))
                break

    def run(self):
        self.add_shaders()

    def add_shaders(self):
        req = AddShaders.Request()
        req.name = 'default'
        # TODO(lucasw) load from disk later
        req.vertex = '''
uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;
// TODO(lucasw) need multiples of these for each projector
// this is the model matrix of the object being drawn
// uniform mat4 projector_model_matrix;
uniform mat4 projector_view_matrix;
uniform mat4 projector_projection_matrix;

in vec3 Position;
in vec3 Normal;
in vec2 UV;
in vec4 Color;

out vec2 FraUV;
smooth out vec3 FraNormal;
out vec4 FraColor;
out vec4 ProjectedTexturePosition;
// The coordinate frame of this position needs to be the same as the normal
out vec3 projector_dir;

void main()
{
  FraUV = UV;
  FraColor = Color;
  // put normal into world frame
  FraNormal = (model_matrix * vec4(Normal, 1.0)).xyz -
        (model_matrix * vec4(0.0, 0.0, 0.0, 1.0)).xyz;
  // FraProjectorPosition =

  mat4 mvp = projection_matrix * view_matrix * model_matrix;
  gl_Position = mvp * vec4(Position.xyz, 1.0);

  mat4 projector_mvp = projector_projection_matrix * projector_view_matrix * model_matrix;
  ProjectedTexturePosition = projector_mvp * vec4(Position.xyz, 1.0);

  // put projector dir into world frame
  projector_dir = (transpose(projector_view_matrix) * vec4(0.0, 0.0, 1.0, 1.0) -
        transpose(projector_view_matrix) * vec4(0.0, 0.0, 0.0, 1.0)).xyz;
}

'''

        req.fragment = '''
// OpenGL makes the first vec4 `out` the fragment color by default
// but should be explicit.
// 130
uniform sampler2D Texture;
uniform sampler2D ProjectedTexture;
uniform float projected_texture_scale;
in vec2 FraUV;
in vec4 FraColor;
smooth in vec3 FraNormal;
in vec3 projector_dir;
in vec4 ProjectedTexturePosition;
out vec4 Out_Color;
void main()
{
    vec4 projected_texture_position = ProjectedTexturePosition;
    // transform to clip space
    projected_texture_position.xyz /= projected_texture_position.w;
    projected_texture_position.xy += 0.5;
    projected_texture_position.z -= 1.0;
    float enable_proj = step(0.0, projected_texture_position.z);
    // TODO(lucasw) this is a manual clamp, can specify this elsewhere and
    // later make it changeable live.
    enable_proj = enable_proj * step(0.0, projected_texture_position.z); /* *
        (1.0 - step(1.0, projected_texture_position.x)) *
        step(0.0, projected_texture_position.x) *
        (1.0 - step(1.0, projected_texture_position.y)) *
        step(0.0, projected_texture_position.y);
    */

    // if normal is facing away from projector disable projection

    // TODO
    float projector_intensity = -dot(FraNormal, projector_dir);
    enable_proj = enable_proj * projector_intensity * step(0.0, projector_intensity);

    // vec3 in_proj_vec = step(0.0, ProjectedTexturePosition.xyz) * (1.0 - step(1.0, ProjectedTexturePosition.xyz));
    // TODO(lwalter) can skip this if always border textures with alpha 0.0
    // float in_proj_bounds = normalize(dot(in_proj_vec, in_proj_vec));
    // Out_Color = FraColor * texture(Texture, FraUV.st) + in_proj_bounds * texture(ProjectedTexture, ProjectedTexturePosition.xy);
    vec2 uv = projected_texture_position.xy;
    // uv.t = 1.0 - uv.t;
    Out_Color = FraColor * texture(Texture, FraUV.st) + enable_proj * projected_texture_scale * texture(ProjectedTexture, uv.st);
}

'''
        self.future = self.shaders_cli.call_async(req)
        self.wait_for_response()

def main(args=None):
    rclpy.init(args=args)

    try:
        demo = AddShadersNode()
        demo.run()
        rclpy.spin(demo)
    finally:
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

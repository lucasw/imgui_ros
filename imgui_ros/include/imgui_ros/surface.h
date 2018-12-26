/*
 * Copyright (c) 2017 Lucas Walter
 * June 2017
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef IMGUI_ROS_SURFACE_H
#define IMGUI_ROS_SURFACE_H

#include <imgui.h>
#include <imgui_ros/camera.h>
#include <imgui_ros/imgui_impl_opengl3.h>
#include <imgui_ros/image.h>
#include <imgui_ros/msg/textured_shape.hpp>
#include <imgui_ros/srv/add_shape.hpp>
// #include <imgui_ros/window.h>
#include <mutex>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
// About OpenGL function loaders: modern OpenGL doesn't have a standard header
// file and requires individual function pointers to be loaded manually. Helper
// libraries are often used for this purpose! Here we are supporting a few
// common ones: gl3w, glew, glad. You may use another loader/header of your
// choice (glext, glLoadGen, etc.), or chose to manually implement your own.
#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
#include <GL/gl3w.h> // Initialize with gl3wInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
#include <GL/glew.h> // Initialize with glewInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
#include <glad/glad.h> // Initialize with gladLoadGL()
#else
#include IMGUI_IMPL_OPENGL_LOADER_CUSTOM
#endif
#pragma GCC diagnostic pop

struct DrawVert {
  glm::vec3 pos;
  glm::vec2 uv;
  glm::vec4 col;
};

struct Shape {
  Shape(const std::string name, const std::string frame_id,
    const std::string texture_name,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer);
  ~Shape();

  // transfer to gpu
  void init();

  void draw();

  std::string print() {
    std::stringstream ss;
    ss << "shape " << name_ << " " << texture_ << " "
        << vertices_.Size << " " << indices_.Size << "\n";
    #if 0
    for (int i = 0; (i < 10) && (i < vertices_.Size); ++i)
      std::cout << i << "  "
        << " " << vertices_[i].pos.x
        << " " << vertices_[i].pos.y
        << " " << vertices_[i].pos.z << ", ";
    std::cout << "\n";
    for (int i = 0; (i < 12) && (i < indices_.Size); ++i)
      std::cout << " " << indices_[i];
    std::cout << "\n";
    #endif
    return ss.str();
  }

  std::string name_ = "unset";
  std::string frame_id_ = "map";
  ImVector<DrawVert> vertices_;
  ImVector<ImDrawIdx> indices_;
  std::string texture_ = 0;

  // ROS
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // OpenGL
  GLuint vao_handle_ = 0;
  GLuint vbo_handle_ = 0;
  GLuint elements_handle_ = 0;

  bool enable_ = true;
};

#endif  // IMGUI_ROS_SURFACE_H

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

#ifndef IMGUI_ROS_VIZ3D_H
#define IMGUI_ROS_VIZ3D_H

#include <glm/glm.hpp>
#include <imgui.h>
#include <imgui_ros/imgui_impl_opengl3.h>
// #include <imgui_ros/window.h>
#include <mutex>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

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

// TODO(lucasw) need to support covering the entire background,
// and being within a widget, possibly with subclassing.
// For now just do entire background.
struct Viz3D {
  Viz3D(const std::string name,
    std::shared_ptr<ImGuiImplOpenGL3> renderer
    );
  ~Viz3D();
  void render(const int fb_width, const int fb_height,
      const int display_pos_x, const int display_pos_y,
      const int display_size_x, const int display_size_y);

  void draw();
  //    const int pos_x, const int pos_y,
  //    const int size_x, const int size_y);
protected:
  glm::vec3 translation_;

  // temp texture test
  GLuint texture_id_ = 0;
  cv::Mat test_;
#if 0
  // TODO(lucasw) or NULL or -1?
  GLuint texture_id_ = 0;
  size_t width_ = 0;
  size_t height_ = 0;

  static const GLfloat* getVertexBuffer() {
    static const GLfloat g_vertex_buffer_data_[] = {
      -1.0f, -1.0f, 0.0f,
      1.0f, -1.0f, 0.0f,
      0.0f,  1.0f, 0.0f,
    };
    return &g_vertex_buffer_data_[0];
  }
  GLuint vertex_buffer_;
#endif

  GLuint shader_handle_ = 0;
  GLuint vert_handle_ = 0;
  GLuint frag_handle_ = 0;
  int attrib_location_tex_ = 0;
  int attrib_location_proj_mtx_ = 0;
  int attrib_location_position_ = 0;
  int attrib_location_uv_ = 0;
  int attrib_location_color_ = 0;
  unsigned int vbo_handle_ = 0;
  unsigned int elements_handle_ = 0;

  std::string name_;
  std::weak_ptr<ImGuiImplOpenGL3> renderer_;
  std::weak_ptr<rclcpp::Node> node_;
};

#endif  // IMGUI_ROS_VIZ3D_H

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

#ifndef IMGUI_ROS_PROJECTOR_H
#define IMGUI_ROS_PROJECTOR_H

#include <glm/glm.hpp>
#include <imgui.h>
#include <imgui_ros/imgui_impl_opengl3.h>
#include <imgui_ros/image.h>
// #include <imgui_ros/AddProjector.h>
// #include <imgui_ros/window.h>
#include <mutex>
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>

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

namespace imgui_ros
{
struct Projector {
  // TODO(lucas) just pass in the Projector.msg
  Projector(const std::string name,
      const std::string texture_name,
      const std::string frame_id,
      const double aov_y,
      const double aov_x,
      const double max_range,
      const double constant_attenuation,
      const double linear_attenuation,
      const double quadratic_attenuation,
      ros::NodeHandle& nh);
  ~Projector();

  std::string print();
  void draw(const std::vector<std::string>& texture_names,
      const std::string& texture_items);
  // void render();

  std::string name_;
  std::string texture_name_;
  std::string frame_id_;
  tf2::Stamped<tf2::Transform> stamped_transform_;

  // TODO(lucasw) later need to use this and resolution to make a CameraInfo
  double aov_y_;
  double aov_x_ = 0.0;

  double near_ = 1.0;
  // TODO(lucasw) can far and max range be the same?
  double far_ = 20.0;
  // set to > 0.0 to be used
  double max_range_ = 0.0;
  double constant_attenuation_ = 0.0;
  double linear_attenuation_ = 0.0;
  double quadratic_attenuation_ = 0.0;

  // TODO(lucasw) put in own class later
  bool enable_ = true;

  int shadow_width_ = 512;
  int shadow_height_ = 512;
  GLuint shadow_framebuffer_ = 0;
  GLuint shadow_depth_texture_ = 0;

  // is a NodeHandle needed?
  // std::weak_ptr<ros::Node> node_;
};

}  // namespace imgui_ros
#endif  // IMGUI_ROS_PROJECTOR_H

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

#ifndef IMGUI_ROS_CUBE_CAMERA_H
#define IMGUI_ROS_CUBE_CAMERA_H

#include <glm/glm.hpp>
#include <imgui.h>
#include <imgui_ros/camera.h>
#include <imgui_ros/imgui_impl_opengl3.h>
#include <imgui_ros/image.h>
#include <imgui_ros_msgs/AddCamera.h>
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
struct CubeFace
{
  CubeFace()
  {
    transform_.setIdentity();
  }

  ~CubeFace()
  {
    glDeleteRenderbuffers(1, &depth_buffer_);
    glDeleteFramebuffers(1, &frame_buffer_);
  }

  GLenum dir_;
  std::shared_ptr<RosImage> image_;
  GLuint depth_buffer_;
  GLuint frame_buffer_;

  tf2::Transform transform_;
};

struct CubeCamera : public Camera {
  CubeCamera(const std::string& name,
      const std::string& frame_id,
      const std::string& header_frame_id,
      const float aox_y, const float aov_x,
      ros::NodeHandle* nh);
  ~CubeCamera();
  void init(
      const size_t width, const size_t height,
      const size_t face_width,
      const std::string& texture_name, const std::string& topic,
      const bool ros_pub,
      ros::NodeHandle* nh,
      std::shared_ptr<ImageTransfer> image_transfer);
  virtual void draw();
  // void render();

  const std::string lens_name_ = "cube_camera_lens";
  GLuint cube_texture_id_ = 0;
  std::array<std::shared_ptr<CubeFace>, 6> faces_;
};

}  // namespace imgui_ros
#endif  // IMGUI_ROS_CUBE_CAMERA_H

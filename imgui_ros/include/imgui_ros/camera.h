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

#ifndef IMGUI_ROS_CAMERA_H
#define IMGUI_ROS_CAMERA_H

#include <glm/glm.hpp>
#include <imgui.h>
#include <imgui_ros/imgui_impl_opengl3.h>
#include <imgui_ros/image.h>
#include <imgui_ros/srv/add_camera.hpp>
// #include <imgui_ros/window.h>
#include <mutex>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
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

struct Camera {
  Camera(const std::string name,
      const std::string frame_id,
      const std::string header_frame_id,
      const double aov_y,
      const double aov_x,
      std::shared_ptr<rclcpp::Node> node);
  ~Camera();

  void init(const size_t width, const size_t height,
      const std::string& texture_name, const std::string& topic,
      std::shared_ptr<rclcpp::Node> node,
      std::shared_ptr<ImageTransfer> image_transfer);
  virtual void draw();
  // void render();

  std::string name_;
  // the frame used in the simulation- needs to be in opengl frame
  // z back, x right, y up
  std::string frame_id_;
  // the frame reported on published messages, should be in optical frame
  // z forward, x right, y down
  std::string header_frame_id_;
  tf2::Stamped<tf2::Transform> stamped_transform_;
  std::shared_ptr<RosImage> image_;

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  void publishCameraInfo(const rclcpp::Time& stamp);

  ImVec4 clear_color_ = ImVec4(0.5, 0.5, 0.5, 1.0);

  // TODO(lucasw) later need to use this and resolution to make a CameraInfo
  double aov_y_ = 90.0;
  double aov_x_ = 0.0;

  double near_ = 0.01;
  double far_ = 100.0;

  bool isReadyToRender();
  // <= 0.0 means render as fast as main loop, otherwise
  // try to get close to the desired framerate by skipping some frames
  float frame_rate = 0.0;
  // the easiest frame rates to do will be those that involving skipping n
  // frames for every rendered, or rendering n frames for every skipped,
  // so for 30Hz update rate this means:
  // skip_max output frame rate (30 / (skip_max)
  // 1        30
  // 2        15
  // 3        10
  // 4        7.5
  // 5        6
  // 6        5
  // 7        4.28
  // 8        3.75
  // ...
  // pub_max  output frame rate 30 * pub_max / (pub_max + 1)
  // 1        15
  // 2        20
  // 3        22.5
  // 4        24.0
  // 5        25
  // 6        25.7
  // 7        26.25
  // 8        26.7
  // ...
  // 19       28.5
  // ...
  // TODO make a map of pairs of pub/skip maxes
  size_t pub_count_ = 0;
  size_t pub_max_ = 0;
  size_t skip_count_ = 0;
  size_t skip_max_ = 0;
  void setFrameRate(const float target_frame_rate, const float update_rate);

  // TODO(lucasw) put in own class later
  bool enable_ = true;
  GLuint frame_buffer_;
  GLuint depth_buffer_;
  // TODO(lucasw) not sure about this
  GLenum DrawBuffers[1] = {GL_COLOR_ATTACHMENT0};
};

#endif  // IMGUI_ROS_CAMERA_H

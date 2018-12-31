/*
 * Copyright (c) 2018 Lucas Walter
 * October 2018
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
 * SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <imgui.h>
// #include "imgui_impl_sdl.h"
#include <imgui_ros/camera.h>
#include <iomanip>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
using std::placeholders::_1;
using std::placeholders::_2;

// TODO(lucasw) 'Camera' -> 'TextureCamera'
Camera::Camera(const std::string name,
    const std::string texture_name,
    const std::string frame_id,
    const std::string topic,
    const size_t width,
    const size_t height,
    const double aov_y,
    const double aov_x,
    std::shared_ptr<rclcpp::Node> node) :
    name_(name),
    frame_id_(frame_id),
    aov_y_(aov_y),
    aov_x_(aov_x)
{
  std::cout << "creating camera " << name << " " << width << " " << height
      << " " << aov_y << " " << aov_x << "\n";
  init(width, height, texture_name, topic, node);
}

void Camera::init(const size_t width, const size_t height,
    const std::string& texture_name, const std::string& topic,
    std::shared_ptr<rclcpp::Node> node)
{
  const bool sub_not_pub = false;
  image_ = std::make_shared<RosImage>(texture_name, topic, sub_not_pub, node);
  {
    // node is bad
    // RCLCPP_INFO(node->get_logger(), "creating camera %s %d %d", name, width, height);
    image_->width_ = width;
    image_->height_ = height;

    image_->image_ = std::make_shared<sensor_msgs::msg::Image>();
    image_->image_->header.frame_id = frame_id_;
    image_->image_->width = width;
    image_->image_->height = height;
    image_->image_->encoding = "bgr8";
    image_->image_->step = width * 3;
    image_->image_->data.resize(width * height * 3);
  }

  {
    cv::Mat tmp(cv::Size(image_->width_, image_->height_), CV_8UC4, cv::Scalar(100, 50, 20, 255));
    glGenTextures(1, &image_->texture_id_);
    glBindTexture(GL_TEXTURE_2D, image_->texture_id_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image_->width_, image_->height_, 0, GL_RGBA,
        GL_UNSIGNED_BYTE, &tmp.data[0]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    // unbind - TODO(lucasw) needed?
    glBindTexture(GL_TEXTURE_2D, 0);
  }

  {
    glGenRenderbuffers(1, &depth_buffer_);
    glBindRenderbuffer(GL_RENDERBUFFER, depth_buffer_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT,
        image_->width_, image_->height_);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
  }

  {
    glGenFramebuffers(1, &frame_buffer_);
    glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
        GL_TEXTURE_2D, image_->texture_id_, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
        GL_RENDERBUFFER, depth_buffer_);

    glDrawBuffers(1, DrawBuffers);
    // OpenGL 4?
    // glNamedFramebufferDrawBuffers(frame_buffer_, 1, DrawBuffers);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
      std::stringstream ss;
      ss << name_ << " framebuffer is not complete " << glGetError();
      throw std::runtime_error(ss.str());
    } else {
      RCLCPP_INFO(node->get_logger(), "camera '%s' framebuffer setup complete, fb %d, depth %d, tex id %d",
          name_.c_str(), frame_buffer_, depth_buffer_, image_->texture_id_);
    }

    // restore default frame buffer
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
  }

  std::string msg;
  if (checkGLError2(msg)) {
    std::cerr << msg << std::endl;
    throw std::runtime_error(msg);
  }
}

Camera::~Camera()
{
  glDeleteRenderbuffers(1, &depth_buffer_);
  glDeleteFramebuffers(1, &frame_buffer_);
}

void Camera::draw()
{
  std::string name = name_ + " camera";
  ImGui::Begin(name.c_str());
  // TODO(lucasw) later re-use code in RosImage
  ImGui::Checkbox(("render to texture##" + name).c_str(), &enable_);

  double min = 1.0;
  double max = 170.0;
  ImGui::SliderScalar(("aov y##" + name).c_str(), ImGuiDataType_Double,
      &aov_y_, &min, &max, "%lf", 2);

  min = 0.0;
  ImGui::SliderScalar(("aov x##" + name).c_str(), ImGuiDataType_Double,
      &aov_x_, &min, &max, "%lf", 2);

  if (enable_) {
    image_->draw();
  }
  ImGui::ColorEdit4(("clear color##" + name).c_str(), (float*)&clear_color_);
  ImGui::End();

  // this does nothing if the image doesn't have a publisher set up
  image_->publish();
}

// Camera::render()
// {
// }

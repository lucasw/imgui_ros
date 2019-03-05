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
#include <imgui_ros/projector.h>
#include <iomanip>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <imgui_ros/utility.h>
using std::placeholders::_1;
using std::placeholders::_2;

namespace imgui_ros
{
Projector::Projector(
    const std::string name,
    const std::string texture_name,
    const std::string frame_id,
    const double aov_y,
    const double aov_x,
    const double max_range,
    const double constant_attenuation,
    const double linear_attenuation,
    const double quadratic_attenuation,
    std::shared_ptr<rclcpp::Node> node) :
    name_(name),
    texture_name_(texture_name),
    frame_id_(frame_id),
    aov_y_(aov_y),
    aov_x_(aov_x),
    max_range_(max_range),
    constant_attenuation_(constant_attenuation),
    linear_attenuation_(linear_attenuation),
    quadratic_attenuation_(quadratic_attenuation),
    node_(node)
{
  std::cout << "creating projector " << print() << std::endl;

  glGenFramebuffers(1, &shadow_framebuffer_);
  glBindFramebuffer(GL_FRAMEBUFFER, shadow_framebuffer_);

  {
    glGenTextures(1, &shadow_depth_texture_);
    glBindTexture(GL_TEXTURE_2D, shadow_depth_texture_);
    // glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT16, 512, 512, 0,
    GLint level = 0;
    // This will produce a 0,1 clamped float
    GLint internal_format = GL_DEPTH_COMPONENT;
    GLint border = 0;
    GLint format = GL_DEPTH_COMPONENT;
    GLvoid* data = nullptr;
    glTexImage2D(GL_TEXTURE_2D, level,
        internal_format,
        shadow_width_, shadow_height_, border,
        format, GL_FLOAT, data);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  }

  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
      GL_TEXTURE_2D, shadow_depth_texture_, 0);
  glDrawBuffer(GL_NONE);
  glReadBuffer(GL_NONE);

  const auto fb_status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  if (fb_status != GL_FRAMEBUFFER_COMPLETE) {
    std::stringstream ss;
    ss << name_ << " framebuffer is not complete " << shadow_framebuffer_
        << " " << shadow_depth_texture_ << ", fb status: " << fb_status  << " " << glGetError();
    throw std::runtime_error(ss.str());
  } else {
    RCLCPP_INFO(node->get_logger(),
        "projector '%s' depth framebuffer setup complete, fb %d, tex id %d",
        name_.c_str(), shadow_framebuffer_, shadow_depth_texture_);
  }

  std::string msg;
  if (checkGLError2(msg)) {
    std::cerr << msg << std::endl;
    throw std::runtime_error(msg);
  }
}

Projector::~Projector()
{
}

std::string Projector::print()
{
  std::stringstream ss;
  ss << name_ << " ";
  ss << texture_name_ << " ";
  ss << frame_id_ << " ";
  ss << aov_y_ << " ";
  ss << aov_x_ << " ";
  return ss.str();
}

void Projector::draw(const std::vector<std::string>& texture_names,
    const std::string& texture_items)
{
  std::string name = name_ + " projector";
  ImGui::PushID(name.c_str());
  // ImGui::Begin(name.c_str());
  // TODO(lucasw) later re-use code in RosImage
  ImGui::Checkbox(name_.c_str(), &enable_);
  imgui_ros::inputText("frame", frame_id_);

  // select texture
  {
    // TODO(lucasw) this is a pain
    int texture_ind = 0;
    for (int i = 0; i < static_cast<int>(texture_names.size()); ++i) {
      if (texture_names[i] == texture_name_) {
        texture_ind = i;
        break;
      }
    }

    const bool changed = ImGui::Combo("texture", &texture_ind,
      texture_items.c_str());
    if (changed) {
      texture_name_ = texture_names[texture_ind];
    }
  }

  {
    double min = 0.01;
    double max = far_;
    ImGui::SliderScalar("near clip", ImGuiDataType_Double,
          &near_, &min, &max, "%lf", 3);
    min = near_;
    max = 100.0;
    ImGui::SliderScalar("far clip", ImGuiDataType_Double,
          &far_, &min, &max, "%lf", 3);
  }

  double min, max;
  // 0.0 means use texture width / height * aov_y
  min = 0.0;
  max = 170.0;
  ImGui::SliderScalar("aov x", ImGuiDataType_Double,
      &aov_x_, &min, &max, "%lf", 2);

  // TODO(lucasw) 0.0 should be orthogonal
  min = 1.0;
  max = 170.0;
  ImGui::SliderScalar("aov y", ImGuiDataType_Double,
      &aov_y_, &min, &max, "%lf", 2);

  min = 0.0;
  max = 100.0;
  ImGui::SliderScalar("max range", ImGuiDataType_Double,
      &max_range_, &min, &max, "%lf", 3);
  min = 0.0;
  max = 100.0;
  ImGui::SliderScalar("constant attenuation", ImGuiDataType_Double,
      &constant_attenuation_, &min, &max, "%lf", 3);
  ImGui::SliderScalar("linear attenuation", ImGuiDataType_Double,
      &linear_attenuation_, &min, &max, "%lf", 3);
  ImGui::SliderScalar("quadratic attenuation", ImGuiDataType_Double,
      &quadratic_attenuation_, &min, &max, "%lf", 3);

  ImVec2 image_size;
  image_size.x = shadow_width_;
  image_size.y = shadow_height_;
  ImGui::Image((void*)(intptr_t)shadow_depth_texture_, image_size);

  ImGui::PopID();
  // ImGui::End();
}

// Projector::render()
// {
// }
}  // namespace imgui_ros

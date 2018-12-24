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
using std::placeholders::_1;
using std::placeholders::_2;

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
  std::cout << "creating projector " << print() << "\n";

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

void Projector::draw()
{
  std::string name = name_ + " projector";
  // ImGui::Begin(name.c_str());
  // TODO(lucasw) later re-use code in RosImage
  ImGui::Checkbox(("projector##" + name).c_str(), &enable_);

  ImGui::Text("texture: %s", texture_name_.c_str());

  double min, max;

  // 0.0 means use texture width / height * aov_y
  min = 0.0;
  max = 170.0;
  ImGui::SliderScalar(("aov x##" + name).c_str(), ImGuiDataType_Double,
      &aov_x_, &min, &max, "%lf", 2);

  // TODO(lucasw) 0.0 should be orthogonal
  min = 1.0;
  max = 170.0;
  ImGui::SliderScalar(("aov y##" + name).c_str(), ImGuiDataType_Double,
      &aov_y_, &min, &max, "%lf", 2);

  min = 0.0;
  max = 100.0;
  ImGui::SliderScalar(("max range##" + name).c_str(), ImGuiDataType_Double,
      &max_range_, &min, &max, "%lf", 3);
  min = 0.0;
  max = 100.0;
  ImGui::SliderScalar(("constant attenuation##" + name).c_str(), ImGuiDataType_Double,
      &constant_attenuation_, &min, &max, "%lf", 3);
  ImGui::SliderScalar(("linear attenuation##" + name).c_str(), ImGuiDataType_Double,
      &linear_attenuation_, &min, &max, "%lf", 3);
  ImGui::SliderScalar(("quadratic attenuation##" + name).c_str(), ImGuiDataType_Double,
      &quadratic_attenuation_, &min, &max, "%lf", 3);

  // ImGui::End();
}

// Projector::render()
// {
// }

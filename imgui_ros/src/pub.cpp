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
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <imgui.h>
#include <imgui_ros/pub.h>

Pub::Pub(const std::string name, const std::string topic, const unsigned type,
    const float value, const float min, const float max,
    std::shared_ptr<rclcpp::Node> node) :
    Window(name, topic), type_(type),
    value_(value), min_(min), max_(max), node_(node) {
  msg_.reset(new std_msgs::msg::Float32);
  pub_ = node_->create_publisher<std_msgs::msg::Float32>(topic);
}

Pub::~Pub()  {

}

void Pub::draw() {
  // TODO(lucasw) typeToString()
  std::stringstream ss;
  ss << name_ << " - " << topic_;
  ImGui::Begin(ss.str().c_str());
  // const std::string text = topic_;
  // ImGui::Text("%.*s", static_cast<int>(text.size()), text.data());
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (type_ == imgui_ros::srv::AddWindow::Request::FLOAT32) {
      float new_value = value_;
      const bool changed = ImGui::SliderScalar(name_.c_str(), ImGuiDataType_Float,
          (void *)&new_value, (void*)&min_, (void*)&max_, "%f");
      if (changed) {
        value_ = new_value;
        msg_->data = value_;
        pub_->publish(msg_);
      }
    }
  }
  ImGui::End();
}


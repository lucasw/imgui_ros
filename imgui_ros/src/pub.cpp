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

namespace imgui_ros
{
Pub::Pub(const std::string name, const std::string topic,  // const unsigned type,
    ros::NodeHandle& nh) :
    Widget(name, topic),
    node_(node) {
}

// Pub::~Pub()  {
// }

BoolPub::BoolPub(const std::string name, const std::string topic,  // const unsigned type,
    const bool value,
    ros::NodeHandle& nh) :
    Pub(name, topic, node), value_(value)  {
  msg_.reset(new std_msgs::Bool);
  pub_ = nh.advertise<std_msgs::Bool>(topic, 2);
}

void BoolPub::draw() {
  // TODO(lucasw) typeToString()
  // const std::string text = topic_;
  // ImGui::Text("%.*s", static_cast<int>(text.size()), text.data());
  {
    std::lock_guard<std::mutex> lock(mutex_);

    {
      bool new_value = value_;
      const bool changed = ImGui::Checkbox(name_.c_str(), &new_value);
      if (changed) {
        // TODO(lucasw) optionall keep this checked, or uncheck immediately
        // value_ = new_value;
        msg_->data = new_value;
        pub_->publish(msg_);
      }
    }
  }
}

StringPub::StringPub(const std::string name, const std::string topic,
    const std::vector<std::string>& items,
    ros::NodeHandle& nh) :
    Pub(name, topic, node), items_(items) {
  msg_.reset(new std_msgs::String);
  if (items.size() > 0) {
    msg_->data = items[0];
  }
  for (auto item : items) {
    items_null_ += item + '\0';
  }
  // TODO(lucasw) bring back type for all the int types
  pub_ = node->create_publisher<std_msgs::String>(topic);
}

void StringPub::draw() {
  // TODO(lucasw) typeToString()
  std::stringstream ss;
  ss << name_ << " - " << topic_;
  std::lock_guard<std::mutex> lock(mutex_);

  // if there are multiple items then the user can select from them,
  // but can't type a new one in (it would be nice if they could)
  if (items_.size() > 1) {
    int new_value = value_;
    const bool changed = ImGui::Combo(name_.c_str(), &new_value, items_null_.c_str());
    if (changed) {
      value_ = new_value;
      msg_->data = items_[value_];
      pub_->publish(msg_);
    }
  } else {
    const size_t sz = 64;
    char buf[sz];
    const size_t sz2 = (msg_->data.size() > (sz - 1)) ? (sz - 1) : msg_->data.size();
    strncpy(buf, msg_->data.c_str(), sz2);
    buf[sz2 + 1] = '\0';
    const bool changed = ImGui::InputText(name_.c_str(), buf, sz,
        ImGuiInputTextFlags_EnterReturnsTrue);
    if (changed) {
      msg_->data = buf;
      pub_->publish(msg_);
    }
  }
}

MenuPub::MenuPub(const std::string name, const std::string topic,  // const unsigned type,
    const int value, const std::vector<std::string>& items,
    ros::NodeHandle& nh) :
    Pub(name, topic, node), value_(value), items_(items) {
  msg_.reset(new std_msgs::Int32);
  for (auto item : items) {
    items_null_ += item + '\0';
  }
  // TODO(lucasw) bring back type for all the int types
  pub_ = node->create_publisher<std_msgs::Int32>(topic);
}

void MenuPub::draw() {
  // TODO(lucasw) typeToString()
  std::stringstream ss;
  ss << name_ << " - " << topic_;
  std::lock_guard<std::mutex> lock(mutex_);

  int new_value = value_;
  const bool changed = ImGui::Combo(name_.c_str(), &new_value, items_null_.c_str());
  if (changed) {
    value_ = new_value;
    msg_->data = value_;
    pub_->publish(msg_);
  }
}
}  // namespace imgui_ros

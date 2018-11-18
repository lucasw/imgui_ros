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
#include <imgui_ros/sub.h>
using std::placeholders::_1;

Sub::Sub(const std::string name, const std::string topic,  // const unsigned type,
    std::shared_ptr<rclcpp::Node> node) :
    Widget(name, topic),
    node_(node) {
}

BoolSub::BoolSub(const std::string name, const std::string topic,  // const unsigned type,
    const bool value,
    std::shared_ptr<rclcpp::Node> node) :
    Sub(name, topic, node) {
  (void)value;
  // msg_.reset(new std_msgs::msg::Bool);
  sub_ = node_->create_subscription<std_msgs::msg::Bool>(topic,
      std::bind(&BoolSub::callback, this, _1));
}

void BoolSub::callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  msg_ = msg;
}

void BoolSub::draw() {
  std::lock_guard<std::mutex> lock(mutex_);
  std::stringstream ss;
  ss << name_ << ": ";
  if (msg_) {
    ss << (msg_->data ? "True" : "False");
  }
  ImGui::Text("%s", ss.str().c_str());
}

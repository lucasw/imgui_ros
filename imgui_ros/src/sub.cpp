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

// Sub::~Sub()  {
// }

FloatSub::FloatSub(const std::string name, const std::string topic,  // const unsigned type,
    const float value,
    std::shared_ptr<rclcpp::Node> node) :
    Sub(name, topic, node) {
  (void)value;
  // msg_.reset(new std_msgs::msg::Float32);
  // msg_.data = value;
  // TODO(lucasw) bring back type for all the float types
  sub_ = node_->create_subscription<std_msgs::msg::Float32>(topic,
      std::bind(&FloatSub::callback, this, _1));
}

void FloatSub::callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  msg_ = msg;
}

void FloatSub::draw() {
  // TODO(lucasw) typeToString()
  // const std::string text = topic_;
  // ImGui::Text("%.*s", static_cast<int>(text.size()), text.data());
  std::lock_guard<std::mutex> lock(mutex_);
  std::stringstream ss;
  // TODO(lucasw) Text box with label on side?
  // or just use other number widget but disable interaction?
  // ImGui::Value()?
  ss << name_ << ": ";
  if (msg_) {
    ss << msg_->data;
  }
  std::string text = ss.str();
  ImGui::Text("%s", ss.str().c_str());
}

FloatPlot::FloatPlot(const std::string name, const std::string topic, // const unsigned type,
      const float value,
      const float min, const float max,
      std::shared_ptr<rclcpp::Node> node) : FloatSub(name, topic, value, node),
        min_(min), max_(max)
{
  data_.push_back(value);
  data_.push_back(value);
}

void FloatPlot::callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  FloatSub::callback(msg);
  std::lock_guard<std::mutex> lock(mutex_);
  data_.push_back(msg->data);
  if (data_.size() > max_points_) {
    data_.erase(data_.begin(), data_.begin() + 1);
  }
}

void FloatPlot::draw() {
  // FloatSub::draw();
  std::lock_guard<std::mutex> lock(mutex_);
  if (data_.size() > 0) {
    ImGui::PlotLines(name_.c_str(), &data_[0], data_.size());
  }
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

IntSub::IntSub(const std::string name, const std::string topic,  // const unsigned type,
    const int value,
    std::shared_ptr<rclcpp::Node> node) :
    Sub(name, topic, node)  {
  (void)value;
  // msg_.reset(new std_msgs::msg::Int32);
  // msg_->data = value;
  // TODO(lucasw) bring back type for all the int types
  sub_ = node_->create_subscription<std_msgs::msg::Int32>(topic,
      std::bind(&IntSub::callback, this, _1));
}

void IntSub::callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  msg_ = msg;
}

void IntSub::draw() {
  std::lock_guard<std::mutex> lock(mutex_);
  std::stringstream ss;
  ss << name_ << ": ";
  if (msg_) {
    ss << msg_->data;
  }
  ImGui::Text("%s", ss.str().c_str());
}


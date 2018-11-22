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

#ifndef IMGUI_ROS_PARAM_H
#define IMGUI_ROS_PARAM_H

#include <deque>
#include <imgui.h>
#include <imgui_ros/srv/add_window.hpp>
#include <imgui_ros/window.h>
#include <mutex>
#include <opencv2/core.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

// TODO(lucasw)
// namespace imgui_ros
struct Param : public Widget {
  Param(const std::string name,
      const std::string node_name,
      const std::string parameter_name,
      uint8_t type,
      std::shared_ptr<rclcpp::Node> node) :
      Widget(name, node_name + "/" + parameter_name),
      node_name_(node_name_),
      parameter_name_(parameter_name),
      type_(type),
      node_(node)
  {
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, node_name_);
    param_sub_ = parameters_client_->on_parameter_event(
        std::bind(&Param::onParameterEvent, this, _1));
  }
  // ~Param();
  virtual void draw()
  {
    // TODO(lucasw) typeToString()
    // const std::string text = topic_;
    // ImGui::Text("%.*s", static_cast<int>(text.size()), text.data());
    std::lock_guard<std::mutex> lock(mutex_);
    std::stringstream ss;
    // TODO(lucasw) Text box with label on side?
    // or just use other number widget but disable interaction?
    // ImGui::Value()?
    ss << name_ << ": ";
    if (value_.type == rcl_interfaces::ParameterType::PARAMETER_BOOL) {
      ss << value_.bool_value;
    } else if (value_.type == rcl_interfaces::ParameterType::PARAMETER_INTEGER) {
      ss << value_.integer_value;
    } else if (value_.type == rcl_interfaces::ParameterType::PARAMETER_DOUBLE) {
      ss << value_.double_value;
    } else if (value_.type == rcl_interfaces::ParameterType::PARAMETER_STRING) {
      ss << value_.string_value;
    } else {
      ss << "TODO support this type " << value_.type;
    }
    std::string text = ss.str();
    ImGui::Text("%s", ss.str().c_str());
  }

protected:
  std::shared_ptr<rclcpp::Node> node_;
  std::string node_name_;
  std::string parameter_name_;
  uint8_t type_;

  rcl_interfaces::msg::ParameterValue value_;

  void onParameterEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
  {
    for (auto & parameter : event->new_parameters) {
      updateValue(parameter.value);
    }
    for (auto & parameter : event->changed_parameters) {
      updateValue(parameter.value);
    }
    // TODO(lucasw) do something when the parameter is deleted?
  }

  bool updateValue(const rcl_interfaces::msg::ParameterValue& new_value)
  {
    if (new_value.type != type_) {
      RCLCPP_WARN(get_logger(), "Wrong type %s %d",
          name, new_value.type);
      return false;
    }
    value_ = new_value;
    return true;
  }

  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_sub_;
};

#endif  // IMGUI_ROS_PARAM_H

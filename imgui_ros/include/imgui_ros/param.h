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
      // TODO(lucasw) this should be the Widget msg sub type, or the ParameterType?
      // Using ParameterType for now
      uint8_t type,
      double min,
      double max,
      std::shared_ptr<rclcpp::Node> node) :
      Widget(name, node_name + "/" + parameter_name),
      node_name_(node_name),
      parameter_name_(parameter_name),
      // type_(type),
      min_(min),
      max_(max),
      node_(node)
  {
    value_.type = type;
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node, node_name_);
    param_sub_ = parameters_client_->on_parameter_event(
        std::bind(&Param::onParameterEvent, this, std::placeholders::_1));
  }

  ~Param()
  {
  }

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
    auto fnc = std::bind(&Param::responseReceivedCallback, this, std::placeholders::_1);
    if (value_.type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
      bool value = value_.bool_value;
      // TODO(lucasw) is there a bool slider?
      const bool changed = ImGui::Checkbox(topic_.c_str(), &value);
      if (changed) {
        value_.bool_value = value;
        parameters_client_->set_parameters({
          rclcpp::Parameter(parameter_name_, value_.bool_value),
        }, fnc);
      }
      ss << value_.bool_value;
    } else if (value_.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
      ImS32 min = min_;
      ImS32 max = max_;
      ImS32 value = value_.integer_value;
      const bool changed = ImGui::SliderScalar(topic_.c_str(),
        ImGuiDataType_S32, &value, &min, &max, "%d");
      if (changed) {
        value_.integer_value = value;
        parameters_client_->set_parameters({
          rclcpp::Parameter(parameter_name_, value_.integer_value),
        }, fnc);
      }
      ss << value_.integer_value;
    } else if (value_.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
      double min = min_;
      double max = max_;
      double value = value_.double_value;
      // TODO(lucasw) may want to only return changed if slider is released
      // https://github.com/ocornut/imgui/issues/1875
      const bool changed = ImGui::SliderScalar(topic_.c_str(),
        ImGuiDataType_Double, &value, &min, &max, "%lf");
      if (changed) {
        value_.double_value = value;
        parameters_client_->set_parameters({
          rclcpp::Parameter(parameter_name_, value_.double_value),
        }, fnc);
      }
      ss << value_.double_value;
    } else if (value_.type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
      const std::string text = value_.string_value;
      const size_t sz = 64;
      char buf[sz];
      const size_t sz2 = (text.size() > (sz - 1)) ? (sz - 1) : text.size();
      strncpy(buf, text.c_str(), sz2);
      buf[sz2 + 1] = '\0';
      const bool changed = ImGui::InputText(topic_.c_str(), buf, sz,
          ImGuiInputTextFlags_EnterReturnsTrue);
      if (changed) {
        value_.string_value = buf;
        parameters_client_->set_parameters({
          rclcpp::Parameter(parameter_name_, value_.string_value),
        }, fnc);
      }
      ss << value_.string_value;
    } else {
      ss << "TODO support this type " << static_cast<int>(value_.type);
    }
    std::string text = ss.str();
    ImGui::Text("%s", ss.str().c_str());
  }

protected:
  std::string node_name_;
  std::string parameter_name_;
  // uint8_t type_;
  double min_ = 0.0;
  double max_ = 1.0;

  std::weak_ptr<rclcpp::Node> node_;

  rcl_interfaces::msg::ParameterValue value_;

  void responseReceivedCallback(
      const std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future)
  {
    std::shared_ptr<rclcpp::Node> node = node_.lock();
    if (!node) {
      return;
    }

    for (auto & result : future.get()) {
      if (!result.successful) {
        RCLCPP_ERROR(node->get_logger(), "Failed to set parameter: %s", result.reason.c_str());
      }
    }
  }

  void onParameterEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
  {
    for (auto & parameter : event->new_parameters) {
      if (parameter.name == parameter_name_) {
        updateValue(parameter.value);
      }
    }
    for (auto & parameter : event->changed_parameters) {
      if (parameter.name == parameter_name_) {
        updateValue(parameter.value);
      }
    }
    // TODO(lucasw) do something when the parameter is deleted?
  }

  bool updateValue(const rcl_interfaces::msg::ParameterValue& new_value)
  {
    std::shared_ptr<rclcpp::Node> node = node_.lock();
    if (!node) {
      return false;
    }

    if (new_value.type != value_.type) {
      RCLCPP_WARN(node->get_logger(), "Wrong type %s %d != %d",
          name_, new_value.type, value_.type);
      return false;
    }
    value_ = new_value;
    return true;
  }

  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_sub_;
};

#endif  // IMGUI_ROS_PARAM_H

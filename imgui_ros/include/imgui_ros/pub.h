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

#ifndef IMGUI_ROS_PUB_H
#define IMGUI_ROS_PUB_H

#include <imgui.h>
#include <imgui_ros/srv/add_window.hpp>
#include <imgui_ros/window.h>
#include <mutex>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int64.hpp>

// TODO(lucasw)
// namespace imgui_ros
// TODO(lucasw) template<typename MessageT> ?
// Or just make a subclass for each because the imgui draw code will be different
// for each?
struct Pub : public Widget {
  Pub(const std::string name, const std::string topic,  // const unsigned type,
      std::shared_ptr<rclcpp::Node> node);
  // ~Pub();
  virtual void draw() = 0;
protected:
  // unsigned type_ = imgui_ros::srv::AddWindow::Request::FLOAT32;
  std::shared_ptr<rclcpp::Node> node_;
};

template <class T>
struct GenericPub : public Pub {
  GenericPub(const std::string name, const std::string topic, // const unsigned type,
      const float value, const float min, const float max,
      std::shared_ptr<rclcpp::Node> node) :
    Pub(name, topic, node), value_(value), min_(min), max_(max)
  {
    msg_.reset(new T);
    // TODO(lucasw) bring back type for all the float types
    pub_ = node_->create_publisher<T>(topic);
  }

  ~GenericPub() {}
  virtual void draw()
  {
    // TODO(lucasw) typeToString()
    // const std::string text = topic_;
    // ImGui::Text("%.*s", static_cast<int>(text.size()), text.data());
    std::lock_guard<std::mutex> lock(mutex_);
    float new_value = value_;
    int imgui_data_type;
    bool changed = false;
    // switch (decltype(this)) {
    if (std::is_same<T, std_msgs::msg::Float32>::value) {
      changed = ImGui::SliderScalar(name_.c_str(), ImGuiDataType_Float,
        &new_value, (void*)&min_, (void*)&max_, "%f");
    } else if (std::is_same<T, std_msgs::msg::Int32>::value) {
      int val = value_;
      int min = min_;
      int max = max_;
      changed = ImGui::SliderScalar(name_.c_str(), ImGuiDataType_S32,
        &val, (void*)&min, (void*)&max, "%d");
      new_value = val;
    } else {
      changed = ImGui::SliderScalar(name_.c_str(), ImGuiDataType_Float,
        &new_value, (void*)&min_, (void*)&max_, "%f");
    }
    if (changed) {
      value_ = new_value;
      msg_->data = value_;
      pub_->publish(msg_);
    }
  }

protected:
  // TODO(lucasw) Fixed at float for now
  float value_ = 0.0;
  float min_ = 0.0;
  float max_ = 1.0;
  std::shared_ptr<T> msg_;
  typename rclcpp::Publisher<T>::SharedPtr pub_;
};

struct BoolPub : public Pub {
  BoolPub(const std::string name, const std::string topic, // const unsigned type,
      const bool value,
      std::shared_ptr<rclcpp::Node> node);
  ~BoolPub() {}
  virtual void draw();
protected:
  bool value_ = false;
  std::shared_ptr<std_msgs::msg::Bool> msg_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
};

struct IntPub : public Pub {
  IntPub(const std::string name, const std::string topic, // const unsigned type,
      const int value, const int min, const int max,
      std::shared_ptr<rclcpp::Node> node);
  ~IntPub() {}
  virtual void draw();
protected:
  int value_ = 0;
  int min_ = 0;
  int max_ = 1;
  std::shared_ptr<std_msgs::msg::Int32> msg_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
};

struct MenuPub : public Pub {
  MenuPub(const std::string name, const std::string topic, // const unsigned type,
      const int value, const std::vector<std::string>& items,
      std::shared_ptr<rclcpp::Node> node);
  ~MenuPub() {}
  virtual void draw();
protected:
  int value_ = 0;
  std::vector<std::string> items_;
  // formatted like how imgui Combo expects
  std::string items_null_ = "";
  std::shared_ptr<std_msgs::msg::Int32> msg_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
};


#endif  // IMGUI_ROS_PUB_H

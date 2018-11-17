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

#ifndef IMGUI_ROS_SUB_H
#define IMGUI_ROS_SUB_H

#include <deque>
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
struct Sub : public Widget {
  Sub(const std::string name, const std::string topic,  // const unsigned type,
      std::shared_ptr<rclcpp::Node> node);
  // ~Sub();
  virtual void draw() = 0;
protected:
  // unsigned type_ = imgui_ros::srv::AddWindow::Request::FLOAT32;
  std::shared_ptr<rclcpp::Node> node_;
};

// TODO(lucasw) use templates
struct FloatSub : public Sub {
  FloatSub(const std::string name, const std::string topic, // const unsigned type,
      const float value,
      std::shared_ptr<rclcpp::Node> node);
  ~FloatSub() {}
  virtual void draw();
protected:
  // TODO(lucasw) Fixed at float for now
  std::shared_ptr<std_msgs::msg::Float32> msg_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
  virtual void callback(const std_msgs::msg::Float32::SharedPtr msg);
};

struct FloatPlot : public FloatSub {
  FloatPlot(const std::string name, const std::string topic, // const unsigned type,
      const float value,
      const float min, const float max,
      std::shared_ptr<rclcpp::Node> node);
  ~FloatPlot() {}
  virtual void draw();
protected:
  size_t max_points_ = 100;
  // std::deque<float> data_;
  std::vector<float> data_;
  float min_;
  float max_;
  virtual void callback(const std_msgs::msg::Float32::SharedPtr msg);
};

struct BoolSub : public Sub {
  BoolSub(const std::string name, const std::string topic, // const unsigned type,
      const bool value,
      std::shared_ptr<rclcpp::Node> node);
  ~BoolSub() {}
  virtual void draw();
protected:
  std::shared_ptr<std_msgs::msg::Bool> msg_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  void callback(const std_msgs::msg::Bool::SharedPtr msg);
};

struct IntSub : public Sub {
  IntSub(const std::string name, const std::string topic, // const unsigned type,
      const int value,
      std::shared_ptr<rclcpp::Node> node);
  ~IntSub() {}
  virtual void draw();
protected:
  std::shared_ptr<std_msgs::msg::Int32> msg_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  void callback(const std_msgs::msg::Int32::SharedPtr msg);
};

#endif  // IMGUI_ROS_SUB_H

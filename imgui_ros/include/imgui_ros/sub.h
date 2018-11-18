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
  std::shared_ptr<rclcpp::Node> node_;
};

template <class T>
struct GenericSub : public Sub {
  GenericSub(const std::string name, const std::string topic,
      std::shared_ptr<rclcpp::Node> node) : Sub(name, topic, node)
  {
    sub_ = node_->create_subscription<T>(topic,
        std::bind(&GenericSub::callback, this, std::placeholders::_1));
  }
  ~GenericSub() {}
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
    if (msg_) {
      // only types with data members that can be used with streams will build
      ss << msg_->data;
    }
    std::string text = ss.str();
    ImGui::Text("%s", ss.str().c_str());
  }
protected:
  std::shared_ptr<T> msg_;
  typename rclcpp::Subscription<T>::SharedPtr sub_;
  virtual void callback(const typename T::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    msg_ = msg;
  }
};

template <class T>
struct PlotSub : public GenericSub<T> {
  // TODO(lucasw) how to get the type of T->data?
  // typedef decltype(T::*data) data_type;
  PlotSub(const std::string name, const std::string topic,
      float value,
      std::shared_ptr<rclcpp::Node> node) :
      GenericSub<T>(name, topic, node)
  {
    data_.push_back(value);
    data_.push_back(value);
  }

  ~PlotSub() {}
  virtual void draw()
  {
    // GenericSub<T>::draw();
    // Can't just refer to inherited mutex_
    std::lock_guard<std::mutex> lock(GenericSub<T>::mutex_);
    if (data_.size() > 0) {
      ImGui::PlotLines(GenericSub<T>::name_.c_str(), &data_[0], data_.size());
    }
  }
protected:
  size_t max_points_ = 100;
  std::vector<float> data_;
  // float min_;
  // float max_;
  virtual void callback(const typename T::SharedPtr msg)
  {
    GenericSub<T>::callback(msg);
    std::lock_guard<std::mutex> lock(GenericSub<T>::mutex_);
    data_.push_back(msg->data);
    if (data_.size() > max_points_) {
      data_.erase(data_.begin(), data_.begin() + 1);
    }
  }
};  // PlotSub

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

#endif  // IMGUI_ROS_SUB_H

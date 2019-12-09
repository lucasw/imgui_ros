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
#include <imgui_ros_msgs/AddWindow.h>
#include <imgui_ros/window.h>
#include <mutex>
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/String.h>

namespace imgui_ros
{
// TODO(lucasw) template<typename MessageT> ?
// Or just make a subclass for each because the imgui draw code will be different
// for each?
struct Pub : public Widget {
  Pub(const std::string name, const std::string topic,  // const unsigned type,
      ros::NodeHandle& nh);
  // ~Pub();
  virtual void draw() = 0;
protected:
  // unsigned type_ = imgui_ros::AddWindow::Request::FLOAT32;
};

template <class T>
struct GenericPub : public Pub {
  GenericPub(const std::string name, const std::string topic, // const unsigned type,
      const float value, const float min, const float max,
      ros::NodeHandle& nh) :
    Pub(name, topic, nh), min_(min), max_(max)
  {
    msg_.reset(new T);
    msg_->data = value;
    // TODO(lucasw) bring back type for all the float types
    pub_ = nh.advertise<T>(topic, 3);
  }

  ~GenericPub() {}
  virtual void draw()
  {
    // TODO(lucasw) typeToString()
    // const std::string text = topic_;
    // ImGui::Text("%.*s", static_cast<int>(text.size()), text.data());
    std::lock_guard<std::mutex> lock(mutex_);
    bool changed = false;
    // switch (decltype(this)) {
    if (std::is_same<T, std_msgs::Float32>::value) {
      float val = msg_->data;
      float min = min_;
      float max = max_;
      // std::cout << name_ << " " << val << " " << min << " " << max << "\n";
      changed = ImGui::SliderScalar(name_.c_str(), ImGuiDataType_Float,
        &val, &min, &max, "%f");
      if (changed) msg_->data = val;
    } else if (std::is_same<T, std_msgs::Float64>::value) {
      double val = msg_->data;
      double min = min_;
      double max = max_;
      changed = ImGui::SliderScalar(name_.c_str(), ImGuiDataType_Double,
        &val, &min, &max, "%lf");
      if (changed) msg_->data = val;
    // TODO(lucasw) combine 8/16/32
    } else if ((std::is_same<T, std_msgs::Int8>::value) ||
              (std::is_same<T, std_msgs::Int16>::value) ||
              (std::is_same<T, std_msgs::Int32>::value)) {
      ImS32 val = msg_->data;
      ImS32 min = min_;
      ImS32 max = max_;
      changed = ImGui::SliderScalar(name_.c_str(), ImGuiDataType_S32,
        &val, &min, &max, "%d");
      if (changed) msg_->data = val;
    } else if (std::is_same<T, std_msgs::Int64>::value) {
      ImS64 val = msg_->data;
      ImS64 min = min_;
      ImS64 max = max_;
      changed = ImGui::SliderScalar(name_.c_str(), ImGuiDataType_S64,
        &val, &min, &max, "%I64d");
      if (changed) msg_->data = val;
    } else if ((std::is_same<T, std_msgs::UInt8>::value) ||
              (std::is_same<T, std_msgs::UInt16>::value) ||
              (std::is_same<T, std_msgs::UInt32>::value)) {
      ImU32 val = msg_->data;
      ImU32 min = min_;
      ImU32 max = max_;
      changed = ImGui::SliderScalar(name_.c_str(), ImGuiDataType_U32,
        &val, &min, &max, "%d");
      if (changed) msg_->data = val;
    } else if (std::is_same<T, std_msgs::UInt64>::value) {
      ImU64 val = msg_->data;
      ImU64 min = min_;
      ImU64 max = max_;
      changed = ImGui::SliderScalar(name_.c_str(), ImGuiDataType_U64,
        &val, &min, &max, "%I64u");
      if (changed) msg_->data = val;

    } else {
      // ImGui::Text(name_.c_str());
      ImGui::Text("unsupported %.*s", static_cast<int>(name_.size()), name_.data());
    }
    if (changed) {
      pub_.publish(msg_);
    }
  }

protected:
  // TODO(lucasw) Fixed at double for now
  double min_ = 0.0;
  double max_ = 1.0;
  T msg_;
  typename ros::Publisher pub_;
};

struct BoolPub : public Pub {
  BoolPub(const std::string name, const std::string topic, // const unsigned type,
      const bool value,
      ros::NodeHandle& nh);
  ~BoolPub() {}
  virtual void draw();
protected:
  bool value_ = false;
  std_msgs::Bool msg_;
  ros::Publisher pub_;
};

struct StringPub : public Pub {
  StringPub(const std::string name, const std::string topic,
      const std::vector<std::string>& items,
      ros::NodeHandle& nh);
  ~StringPub() {}
  virtual void draw();
protected:
  int value_ = 0;
  std::string items_null_ = "";
  std::vector<std::string> items_;
  std_msgs::String msg_;
  ros::Publisher pub_;
};

struct MenuPub : public Pub {
  MenuPub(const std::string name, const std::string topic, // const unsigned type,
      const int value, const std::vector<std::string>& items,
      ros::NodeHandle& nh);
  ~MenuPub() {}
  virtual void draw();
protected:
  int value_ = 0;
  std::vector<std::string> items_;
  // formatted like how imgui Combo expects
  std::string items_null_ = "";
  std_msgs::Int32 msg_;
  ros::Publisher pub_;
};

}  // namespace imgui_ros

#endif  // IMGUI_ROS_PUB_H

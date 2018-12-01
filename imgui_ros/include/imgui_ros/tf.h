/*
 * Copyright (c) 2018 Lucas Walter
 * November 2018
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

#ifndef IMGUI_ROS_TF_H
#define IMGUI_ROS_TF_H

#include <imgui.h>
#include <imgui_ros/srv/add_window.hpp>
#include <imgui_ros/window.h>
#include <imgui_ros/sub.h>
#include <imgui_ros/pub.h>
#include <mutex>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
using std::placeholders::_1;

// TODO(lucasw)
// namespace imgui_ros
struct TfEcho : public Sub {
  TfEcho(const std::string name,
      const std::string parent, const std::string child,
      std::shared_ptr<tf2_ros::Buffer> tf_buffer,
      std::shared_ptr<rclcpp::Node> node);

  ~TfEcho() {}

  virtual void draw();
protected:
  std::string parent_;
  std::string child_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

struct TfBroadcaster : public Pub {
  TfBroadcaster(const std::string name,
      const std::string parent, const std::string child,
      const double min, const double max,
      std::shared_ptr<tf2_ros::Buffer> tf_buffer,
      std::shared_ptr<rclcpp::Node> node);

  ~TfBroadcaster() {}

  virtual void draw();
protected:
  void update();
  double min_;
  double max_;
  geometry_msgs::msg::TransformStamped ts_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};

#endif  // IMGUI_ROS_TF_H

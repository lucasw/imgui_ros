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

#ifndef IMGUI_ROS_BAG_CONSOLE_H
#define IMGUI_ROS_BAG_CONSOLE_H

#include <deque>
#include <imgui.h>
#include <imgui_ros_msgs/AddWindow.h>
#include <imgui_ros/sub.h>
#include <imgui_ros/window.h>
#include <mutex>
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>

namespace imgui_ros
{
// template <class T>
struct BagConsole : public Sub {
  BagConsole(const std::string name, const std::string topic,
      ros::NodeHandle& nh) : Sub(name, topic, nh)
  {
    sub_ = nh.subscribe(topic, 10, &BagConsole::callback, this);
  }
  ~BagConsole() {}
  virtual void draw();
protected:
  // 0.0 - 1.0, the ends should be sticky and always show the first or last element
  double position_ = 0.0;
  size_t max_num_ = 250;
  // typename T::ConstPtr msg_;
  std::deque<rosgraph_msgs::Log::ConstPtr> msgs_;
  // typename
  ros::Subscriber sub_;
  size_t selected_ = 0;
  // virtual void callback(const typename T::ConstPtr msg)
  void callback(const typename rosgraph_msgs::LogConstPtr msg);
};

}  // namespace imgui_ros
#endif  // IMGUI_ROS_BAG_CONSOLE_H

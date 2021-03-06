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
#include <imgui_ros/AddWindow.h>
#include <imgui_ros/window.h>
#include <mutex>
#include <opencv2/core.hpp>
#if 0
#include <rcl_interfaces/parameter_event.hpp>
#include <rcl_interfaces/parameter_type.hpp>
#include <rcl_interfaces/parameter_value.hpp>
#endif
#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace imgui_ros
{
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
      ros::NodeHandle& nh);
  ~Param();

  virtual void draw();

  bool update_ = false;
  std::string node_name_;
  std::string parameter_name_;
  // rcl_interfaces::ParameterValue value_;
  // ros::ParameterValue value_;
protected:
  // uint8_t type_;
  double min_ = 0.0;
  double max_ = 1.0;

  std::weak_ptr<ros::Node> node_;

#if 0
  void responseReceivedCallback(
      const std::shared_future<std::vector<rcl_interfaces::SetParametersResult>> future);

  // TODO(lucasw) need to push this up into containing viz3d class,
  // it will have a list of namespaces that it has parameter events for and will
  // receive the event and distribute the values to the proper param
  void onParameterEvent(const rcl_interfaces::ParameterEvent::SharedPtr event);
  bool updateValue(const rcl_interfaces::ParameterValue& new_value);

  ros::AsyncParametersClient::SharedPtr parameters_client_;
  ros::Subscription<rcl_interfaces::ParameterEvent>::SharedPtr param_sub_;
#endif
};

}  // namespace imgui_ros
#endif  // IMGUI_ROS_PARAM_H

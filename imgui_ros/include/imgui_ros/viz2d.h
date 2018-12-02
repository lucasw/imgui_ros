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

#ifndef IMGUI_ROS_VIZ2D_H
#define IMGUI_ROS_VIZ2D_H

#include <imgui.h>
#include <imgui_ros/srv/add_window.hpp>
#include <imgui_ros/window.h>
#include <imgui_ros/sub.h>
#include <map>
#include <mutex>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

// TODO(lucasw)
// namespace imgui_ros
struct Viz2D : public Sub {
  Viz2D(const std::string name,
      const std::string topic,
      const std::string frame_id,
      const std::vector<std::string>& frames,
      const double pixels_per_meter,
      std::shared_ptr<tf2_ros::Buffer> tf_buffer,
      std::shared_ptr<rclcpp::Node> node);

  ~Viz2D() {}

  virtual void draw();
protected:
  std::string frame_id_;
  std::vector<std::string> frames_;
  double pixels_per_meter_ = 10.0;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_sub_;
  // TODO(lucasw) make this a service later
  std::map<std::string, std::map<int, visualization_msgs::msg::Marker::SharedPtr> > markers_;
  void markerCallback(const visualization_msgs::msg::Marker::SharedPtr msg);

  ImVec2 offset_;
  bool dragging_view_ = false;
  ImVec2 drag_point_;

  void drawTf(ImDrawList* draw_list, ImVec2 origin, ImVec2 center);
  void drawMarkers(ImDrawList* draw_list, ImVec2 origin, ImVec2 center);
};

#endif  // IMGUI_ROS_VIZ2D_H

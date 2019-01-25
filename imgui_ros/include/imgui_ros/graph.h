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

#ifndef IMGUI_ROS_GRAPH_H
#define IMGUI_ROS_GRAPH_H

#include <imgui.h>
#include <imgui_ros/window.h>
#include <mutex>
#include <imgui_ros/node.h>
#include <rclcpp/rclcpp.hpp>

// TODO(lucasw)
// namespace imgui_ros
// TODO(lucasw) template<typename MessageT> ?
// Or just make a subclass for each because the imgui draw code will be different
// for each?
struct Graph : public Widget {
  Graph(const std::string name,
      std::shared_ptr<rclcpp::Node> node);
  // ~Graph();
  virtual void draw();
  virtual void update(const rclcpp::Time& stamp);

protected:
  // unsigned type_ = imgui_ros::srv::AddWindow::Request::FLOAT32;
  std::weak_ptr<rclcpp::Node> node_;

  bool opened_;

  std::shared_ptr<Connector> con_src_;
  std::shared_ptr<Connector> con_dst_;

  std::map<std::string, std::shared_ptr<Node> > nodes_;
  std::map<std::string, std::shared_ptr<Link> > links_;
  void linkNodes(
      const std::string& output_node_name, const std::string& output_node_con_name,
      const std::string& input_node_name, const std::string& input_node_con_name);
  rclcpp::Time start_, stamp_;

  void init();
  bool inited_ = false;

  ImVec2 scrolling_ = ImVec2(0.0f, 0.0f);
  bool show_grid_ = true;
  std::shared_ptr<Node> node_selected_;

  // adding a new link
  std::shared_ptr<Node> node_for_slot_selected_;
};

#endif  // IMGUI_ROS_GRAPH_H

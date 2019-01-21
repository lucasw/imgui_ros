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
#include <rclcpp/rclcpp.hpp>

// Adapting from https://gist.github.com/ocornut/7e9b3ec566a333d725d4 by ocornut
// NB: You can use math functions/operators on ImVec2 if you #define IMGUI_DEFINE_MATH_OPERATORS and #include "imgui_internal.h"
// Here we only declare simple +/- operators so others don't leak into the demo code.
static inline ImVec2 operator+(const ImVec2& lhs, const ImVec2& rhs) { return ImVec2(lhs.x + rhs.x, lhs.y + rhs.y); }
static inline ImVec2 operator-(const ImVec2& lhs, const ImVec2& rhs) { return ImVec2(lhs.x - rhs.x, lhs.y - rhs.y); }


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
protected:
  // unsigned type_ = imgui_ros::srv::AddWindow::Request::FLOAT32;
  std::weak_ptr<rclcpp::Node> node_;

  bool opened_;

  // Dummy
  struct Node
  {
    int     id_;
    char    name_[32];
    ImVec2  pos_, size_;
    float   value_ = 0.0;
    ImVec4  color_;
    int     inputs_count_, outputs_count_;

    const float NODE_SLOT_RADIUS = 4.0f;
    const ImVec2 NODE_WINDOW_PADDING = ImVec2(8.0f, 8.0f);

    Node(int id, const char* name, const ImVec2& pos, float value, const ImVec4& color, int inputs_count, int outputs_count)
    {
      id_ = id;
      strncpy(name_, name, 31);
      name_[31] = 0;
      pos_ = pos;
      value_ = value;
      color_ = color;
      inputs_count_ = inputs_count;
      outputs_count_ = outputs_count;
    }

    ImVec2 GetInputSlotPos(int slot_no) const
    {
      return ImVec2(pos_.x,
          pos_.y + size_.y * ((float)slot_no + 1) / ((float)inputs_count_ + 1));
    }
    ImVec2 GetOutputSlotPos(int slot_no) const
    {
      return ImVec2(pos_.x + size_.x,
          pos_.y + size_.y * ((float)slot_no + 1) / ((float)outputs_count_ + 1));
    }

    virtual void draw(ImDrawList* draw_list, ImVec2& offset, int& node_selected,
        int& node_hovered_in_list, int& node_hovered_in_scene,
        bool& open_context_menu);
  };
  struct NodeLink
  {
    int input_idx_, input_slot_, output_idx_, output_slot_;

    NodeLink(const int input_idx, const int input_slot,
        const int output_idx, const int output_slot)
    {
      input_idx_ = input_idx;
      input_slot_ = input_slot;
      output_idx_ = output_idx;
      output_slot_ = output_slot;
    }
  };

  ImVector<Node> nodes_;
  ImVector<NodeLink> links_;
  bool inited_ = false;
  ImVec2 scrolling_ = ImVec2(0.0f, 0.0f);
  bool show_grid_ = true;
  int node_selected_ = -1;
};



#endif  // IMGUI_ROS_GRAPH_H

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

#ifndef IMGUI_ROS_NODE_H
#define IMGUI_ROS_NODE_H

#include <imgui.h>
#include <imgui_ros/window.h>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

// Adapting from https://gist.github.com/ocornut/7e9b3ec566a333d725d4 by ocornut
// NB: You can use math functions/operators on ImVec2 if you #define IMGUI_DEFINE_MATH_OPERATORS and #include "imgui_internal.h"
// Here we only declare simple +/- operators so others don't leak into the demo code.
static inline ImVec2 operator+(const ImVec2& lhs, const ImVec2& rhs) { return ImVec2(lhs.x + rhs.x, lhs.y + rhs.y); }
static inline ImVec2 operator-(const ImVec2& lhs, const ImVec2& rhs) { return ImVec2(lhs.x - rhs.x, lhs.y - rhs.y); }

struct Link;
struct Node;

// either input or output
struct Connector : std::enable_shared_from_this<Connector>
{
  Connector(const bool input_not_output) : input_not_output_(input_not_output)
  {
    color_ = ImVec4(0.5, 0.5, 0.5, 0.6);
  }

  std::string name_;
  float value_;
  bool selected_ = false;
  const float RADIUS = 8.0f;
  ImVec4 color_;
  ImVec2 pos_;
  ImVec2 size_;

  bool input_not_output_ = true;

  void update();
  void draw(ImDrawList* draw_list, const ImVec2& offset,
      std::shared_ptr<Connector>& src, std::shared_ptr<Connector>& dst);
  ImVec2 getPos();

  std::shared_ptr<Node> parent_;
  std::shared_ptr<Link> link_;
};

  struct Node : std::enable_shared_from_this<Node>
  {
    std::string name_;
    ImVec2  pos_, size_;
    ImVec4  color_;
    bool selected_ = false;
    const ImVec2 NODE_WINDOW_PADDING = ImVec2(8.0f, 8.0f);

    Node(const std::string& name, const ImVec2& pos,
        const ImVec4& color) :
        name_(name)
    {
      pos_ = pos;
      color_ = color;
    }

    ImVec2 getPos() { return pos_; }

    void resetConnections();
    virtual void init() { resetConnections();}
    virtual void update(const double& seconds) { seconds_ = seconds;}

    virtual void draw2(ImDrawList* draw_list);
    virtual void draw(ImDrawList* draw_list, const ImVec2& offset,
        std::shared_ptr<Node>& node_hovered_in_list,
        std::shared_ptr<Node>& node_hovered_in_scene,
        bool& open_context_menu,
        std::shared_ptr<Node>& node_for_slot_selected,
        std::shared_ptr<Connector>& con_src, std::shared_ptr<Connector>& con_dst);

    double seconds_;

    std::map<std::string, std::shared_ptr<Connector> > inputs_;
    std::map<std::string, std::shared_ptr<Connector> > outputs_;
  };

  struct Link
  {
    Link(const std::string& name) : name_(name)
    {
    }

    virtual void update();
    virtual void draw(ImDrawList* draw_list, const ImVec2& offset);

    std::string name_;
    // TODO(lucasw) weak_ptr instead?
    std::shared_ptr<Connector> input_;
    std::map<std::string, std::shared_ptr<Connector> > outputs_;
  };

  ///////////////////////////////////////////////////////////////////////
  struct SignalGenerator : public Node
  {
    SignalGenerator(const std::string& name, const ImVec2& pos);
    virtual void init();
    virtual void update(const double& seconds);
    virtual void draw2(ImDrawList* draw_list);

    // TODO(lucasw) Turn these into inputs later
    float amplitude_ = 1.0;
    float frequency_ = 1.0;
  };

  struct SignalCombine : public Node
  {
    SignalCombine(const std::string& name, const ImVec2& pos);
    virtual void init();
    virtual void update(const double& seconds);
    virtual void draw2(ImDrawList* draw_list);
  };

#endif  // IMGUI_ROS_NODE_H

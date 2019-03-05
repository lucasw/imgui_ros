/*
 * Copyright (c) 2018 Lucas Walter
 * October 2018
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

#include <imgui.h>
#include <imgui_ros/node.h>
#include <math.h> // fmodf
#include <memory>

namespace imgui_ros
{
void Connector::update()
{

}

void Connector::draw(ImDrawList* draw_list, const ImVec2& offset,
    std::shared_ptr<Connector>& src, std::shared_ptr<Connector>& dst)
{
  const auto pos = getPos() + offset;
  // const auto col = IM_COL32(250, 150, 250, 150);
  // draw_list->AddCircleFilled(pos, RADIUS, col);

  draw_list->ChannelsSetCurrent(1); // Foreground
  ImGui::SetCursorScreenPos(pos);  //  + NODE_WINDOW_PADDING);
  ImGui::BeginGroup(); // Lock horizontal position
  ImGui::Text("%06.2f", value_);
  ImGui::EndGroup();

  size_ = ImGui::GetItemRectSize();  // + NODE_WINDOW_PADDING + NODE_WINDOW_PADDING;
  ImVec2 node_rect_max = pos + size_;

  ImGui::SetCursorScreenPos(pos);
  ImGui::InvisibleButton("output", size_);
  const bool is_hovered = ImGui::IsItemHovered();

  draw_list->ChannelsSetCurrent(0); // Background

  int green = 85;
  if (is_hovered) {
    green = 145;
    // if ((src == nullptr) && ImGui::IsMouseDragging(0)) {
    if (ImGui::IsMouseClicked(0)) {
      if (src == nullptr) {
        src = shared_from_this();
      } else if (src == shared_from_this()) {
        // unselect
        src = nullptr;
      } else {
        dst = shared_from_this();
      }
    }
  }
  int red = 85;
  if (src == shared_from_this()) {
    red = 130;
  }
  auto con_bg_color = IM_COL32(red, green, 75, 255);
  draw_list->AddRectFilled(pos, node_rect_max, con_bg_color, 4.0f);
}

ImVec2 Connector::getPos()
{
  if (!parent_) return pos_;
  return parent_->getPos() + pos_;
}

ImVec2 Connector::getEndPos()
{
  return getPos() + ImVec2(parent_->size_.x + size_.x, size_.y * 0.5);
}
////////////////////////////////////////////////////////////////////////////

void Node::resetConnections()
{
  // break circular shared_ptrs
  for (auto con_pair : inputs_) {
    con_pair.second->parent_ = nullptr;
  }
  inputs_.clear();
  for (auto con_pair : outputs_) {
    con_pair.second->parent_ = nullptr;
  }
  outputs_.clear();
}

void Node::draw2(ImDrawList* draw_list)
{
  (void)draw_list;
  // Display node contents first
  ImGui::Text("%s", name_.c_str());
  // ImGui::Text("%06.2f", inputs_[);
  // ImGui::ColorEdit3("##color", &color_.x);
}

void Node::draw(ImDrawList* draw_list, const ImVec2& offset,
    std::shared_ptr<Node>& node_hovered_in_list,
    std::shared_ptr<Node>& node_hovered_in_scene,
    bool& open_context_menu,
    std::shared_ptr<Connector>& con_src, std::shared_ptr<Connector>& con_dst)
{
  ImVec2 node_rect_min = offset + pos_;

  draw_list->ChannelsSetCurrent(1); // Foreground
  ImGui::SetCursorScreenPos(node_rect_min + NODE_WINDOW_PADDING);
  ImGui::BeginGroup(); // Lock horizontal position
  draw2(draw_list);
  ImGui::EndGroup();

  // Display node contents first
  draw_list->ChannelsSetCurrent(1); // Foreground
  bool old_any_active = ImGui::IsAnyItemActive();
  ImGui::SetCursorScreenPos(node_rect_min + NODE_WINDOW_PADDING);

  // Save the size of what we have emitted and whether any of the widgets are being used
  bool node_widgets_active = (!old_any_active && ImGui::IsAnyItemActive());
  size_ = ImGui::GetItemRectSize() + NODE_WINDOW_PADDING + NODE_WINDOW_PADDING;
  ImVec2 node_rect_max = node_rect_min + size_;

  // Display node box
  draw_list->ChannelsSetCurrent(0); // Background
  ImGui::SetCursorScreenPos(node_rect_min);
  ImGui::InvisibleButton("node", size_);
  if (ImGui::IsItemHovered())
  {
    node_hovered_in_scene = shared_from_this();
    open_context_menu |= ImGui::IsMouseClicked(1);
  }
  bool node_moving_active = ImGui::IsItemActive();
  if (node_widgets_active || node_moving_active) {
    // TODO(lucasw) how to unselect?
    selected_ = true;
  }
  if (node_moving_active && ImGui::IsMouseDragging(0)) {
    pos_ = pos_ + ImGui::GetIO().MouseDelta;
  }

  int red = 60;
  int green = 60;
  int blue = 60;
  if (node_hovered_in_list == shared_from_this()) {
    red = 100;
  }
  if (node_hovered_in_scene == shared_from_this()) {
    green = 100;
  }
  if (node_hovered_in_list == nullptr && selected_) {
    blue = 100;
  }
  ImU32 node_bg_color = IM_COL32(red, green, blue, 255);

  draw_list->AddRectFilled(node_rect_min, node_rect_max, node_bg_color, 4.0f);
  draw_list->AddRect(node_rect_min, node_rect_max, IM_COL32(100, 100, 100, 255), 4.0f);

  for (auto input_pair : inputs_) {
    input_pair.second->draw(draw_list, offset,
        con_src, con_dst);
  }

  for (auto output_pair : outputs_) {
    output_pair.second->draw(draw_list, offset + ImVec2(size_.x, 0),
        con_src, con_dst);
  }
}

///////////////////////////////////////////////////////////////////////////////
void Link::update()
{
  for (auto output_pair : outputs_) {
    output_pair.second->value_ = input_->value_;
  }
}

void Link::draw(ImDrawList* draw_list, const ImVec2& offset)
{
  if (!input_) {
    return;
  }

  const ImVec2 p1 = offset + input_->getEndPos();
  // ImGui::Text("p1 %f %f", p1.x, p1.y);
  for (auto output_pair : outputs_) {
    const auto output = output_pair.second;
    if (!output) {
      continue;
    }
    const ImVec2 p2 = offset + output->getPos() +
        ImVec2(0.0 /*-output->size_.x*/, output->size_.y * 0.5);
    // ImGui::Text("p2 %f %f", p2.x, p2.y);
    draw_list->AddBezierCurve(
        p1, p1 + ImVec2(+80, 0),
        p2 + ImVec2(-80, 0), p2,
        IM_COL32(200, 200, 100, 255), 3.0f);
  }
}

/////////////////////////////////////////////////////////////////////////////////
SignalGenerator::SignalGenerator(const std::string& name,
    const ImVec2& pos) : Node(name, pos, ImColor(255, 0, 0, 255))
{
}

void SignalGenerator::init()
{
  resetConnections();
  auto con = std::make_shared<Connector>(false);
  con->name_ = "signal";
  con->parent_ = shared_from_this();
  con->pos_ = ImVec2(0, 10);
  outputs_[con->name_] = con;
}

void SignalGenerator::update(const double& seconds)
{
  // TODO(lucasw) make a changing frequency change the output seamlessly
  // with a dynamic phase.
  outputs_["signal"]->value_ = amplitude_ * sin(seconds * frequency_ * M_PI * 2.0);
  Node::update(seconds);
}

void SignalGenerator::draw2(ImDrawList* draw_list)
{
  Node::draw2(draw_list);
  ImGui::SliderFloat("##frequency", &frequency_, 0.0f, 15.0f, "Freq %.2f", 3);
  ImGui::SliderFloat("##amplitude", &amplitude_, 0.0f, 100.0f, "Amp %.2f", 3);
}

/////////////////////////////////////////////////////////////////////////////////
SignalCombine::SignalCombine(const std::string& name, const ImVec2& pos) :
    Node(name, pos, ImColor(255, 100, 0, 255))
{
}

void SignalCombine::init()
{
  resetConnections();
  {
    auto con = std::make_shared<Connector>(true);
    con->name_ = "in1";
    con->parent_ = shared_from_this();
    con->pos_ = ImVec2(-40, 10);
    inputs_[con->name_] = con;
  }
  {
    auto con = std::make_shared<Connector>(true);
    con->name_ = "in2";
    con->parent_ = shared_from_this();
    con->pos_ = ImVec2(-40, 50);
    inputs_[con->name_] = con;
  }
  {
    auto con = std::make_shared<Connector>(false);
    con->name_ = "signal";
    con->parent_ = shared_from_this();
    con->pos_ = ImVec2(0, 10);
    outputs_[con->name_] = con;
  }
}

void SignalCombine::update(const double& seconds)
{
  float value = 0.0;
  for (auto input_pair : inputs_) {
    value += input_pair.second->value_;  // input_node_->value_;  // * coefficient_[i];
  }
  outputs_["signal"]->value_ = value;

  Node::update(seconds);
}

void SignalCombine::draw2(ImDrawList* draw_list)
{
  Node::draw2(draw_list);
}

////////////
FloatPublisher::FloatPublisher(const std::string& name, const ImVec2& pos,
    const std::string& topic) :
    Node(name, pos, ImColor(255, 100, 0, 255)),
    topic_(topic)
{
}

void FloatPublisher::init()
{
  resetConnections();
  {
    auto con = std::make_shared<Connector>(true);
    con->name_ = "input";
    con->parent_ = shared_from_this();
    con->pos_ = ImVec2(-40, 10);
    inputs_[con->name_] = con;
  }
}

void FloatPublisher::update(const double& seconds)
{
  float value = 0.0;
  for (auto input_pair : inputs_) {
    value += input_pair.second->value_;  // input_node_->value_;  // * coefficient_[i];
  }
  outputs_["signal"]->value_ = value;

  // TODO(lucasw) may want an update rate control
  // that won't publish every update.
  if (pub_) {
    std_msgs::msg::Float32 msg;
    msg.data = value;
    pub_->publish(msg);
  }
  Node::update(seconds);
}

void FloatPublisher::draw2(ImDrawList* draw_list)
{
  Node::draw2(draw_list);
  ImGui::Text("Topic: '%s'", topic_.c_str());
}
}  // namespace imgui_ros

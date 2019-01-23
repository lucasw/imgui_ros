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

void Connector::update()
{

}

void Connector::draw(ImDrawList* draw_list, const ImVec2& offset)
{
  auto col = IM_COL32(250, 150, 250, 150);
  draw_list->AddCircleFilled(getPos() + offset, RADIUS, col);
  ImGui::Text("%06.2f", value_);
}

ImVec2 Connector::getPos()
{
  if (!parent_) return pos_;
  return parent_->getPos() + pos_;
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
    int& node_hovered_in_list, int& node_hovered_in_scene,
    bool& open_context_menu,
    std::shared_ptr<Node>& node_for_slot_selected)
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
    node_hovered_in_scene = id_;
    open_context_menu |= ImGui::IsMouseClicked(1);
  }
  bool node_moving_active = ImGui::IsItemActive();
  if (node_widgets_active || node_moving_active) {
    selected_ = true;
  }
  if (node_moving_active && ImGui::IsMouseDragging(0)) {
    pos_ = pos_ + ImGui::GetIO().MouseDelta;
  }

  ImU32 node_bg_color = (node_hovered_in_list == id_ ||
      node_hovered_in_scene == id_ ||
      (node_hovered_in_list == -1 && selected_)) ? IM_COL32(75, 75, 75, 255) : IM_COL32(60, 60, 60, 255);

  draw_list->AddRectFilled(node_rect_min, node_rect_max, node_bg_color, 4.0f);
  draw_list->AddRect(node_rect_min, node_rect_max, IM_COL32(100, 100, 100, 255), 4.0f);

  for (auto input_pair : inputs_) {
    input_pair.second->draw(draw_list, offset);
  }

  for (auto output_pair : outputs_) {
    output_pair.second->draw(draw_list, offset);
  }

  #if 0
  for (int slot_idx = 0; slot_idx < static_cast<int>(output_links_.size()); slot_idx++) {
    const ImVec2 pos = offset + getOutputSlotPos(slot_idx);
    const ImVec2 slot_half_size = ImVec2(NODE_SLOT_RADIUS, NODE_SLOT_RADIUS);
    const ImVec2 slot_size = ImVec2(NODE_SLOT_RADIUS * 2.0, NODE_SLOT_RADIUS * 2.0);
    // draw_list->ChannelsSetCurrent(0); // Background
    ImGui::SetCursorScreenPos(pos - slot_half_size);
    ImGui::InvisibleButton("output", slot_size);
    ImColor col = IM_COL32(150, 150, 150, 150);

    // only allow new selections if not already dragging around a link for another node
    // or another output on this node
    if (((node_for_slot_selected == nullptr) || (node_for_slot_selected == shared_from_this())) &&
        ((slot_selected_ == -1) || (slot_selected_ == slot_idx))) {

      // TODO(lucasw) the hovering doesn't work if outside the box of this node
      // half the circle is outside it, fix that or just move circle to inside.
      if (ImGui::IsItemHovered()) {
        col = IM_COL32(200, 200, 200, 200);
        if (ImGui::IsMouseDragging(1)) {
          col = IM_COL32(250, 250, 250, 250);
          slot_selected_ = slot_idx;
          node_for_slot_selected = shared_from_this();
        }
      }

      if (ImGui::IsMouseDragging(1)) {
        // while the mouse continues to drag draw the potential new link
        if (slot_idx == slot_selected_) {
          col = IM_COL32(250, 150, 250, 150);
          ImVec2 mouse_pos = ImGui::GetMousePos();
          draw_list->AddCircleFilled(mouse_pos, NODE_SLOT_RADIUS, col);
          draw_list->AddBezierCurve(
              pos, pos + ImVec2(+50, 0),
              mouse_pos + ImVec2(-50, 0), mouse_pos,
              IM_COL32(200, 200, 100, 255), 3.0f);
        }
      } else {
        // if the mouse is no longer dragging the slot_selected is reset
        slot_selected_ = -1;
        node_for_slot_selected = nullptr;
      }
    }

  }
  #endif
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

  const ImVec2 p1 = offset + input_->getPos();
  // ImGui::Text("p1 %f %f", p1.x, p1.y);
  for (auto output_pair : outputs_) {
    const auto output = output_pair.second;
    if (!output) {
      continue;
    }
    const ImVec2 p2 = offset + output->getPos();
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
  auto con = std::make_shared<Connector>();
  con->parent_ = shared_from_this();
  con->pos_ = ImVec2(0, 10);
  outputs_["signal"] = con;
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
    auto con = std::make_shared<Connector>();
    con->parent_ = shared_from_this();
    con->pos_ = ImVec2(0, 10);
    inputs_["in1"] = con;
  }
  {
    auto con = std::make_shared<Connector>();
    con->parent_ = shared_from_this();
    con->pos_ = ImVec2(0, 50);
    inputs_["in2"] = con;
  }
  {
    auto con = std::make_shared<Connector>();
    con->parent_ = shared_from_this();
    con->pos_ = ImVec2(100, 10);
    outputs_["signal"] = con;
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

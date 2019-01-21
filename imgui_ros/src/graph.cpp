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
#include <imgui_ros/graph.h>
#include <math.h> // fmodf

static const std::string topic = "";
Graph::Graph(const std::string name,
    std::shared_ptr<rclcpp::Node> node) :
    Widget(name, topic),
    node_(node)
{
}

// Creating a node graph editor for ImGui
// Quick demo, not production code! This is more of a demo of how to use ImGui to create custom stuff.
// Better version by @daniel_collin here https://gist.github.com/emoon/b8ff4b4ce4f1b43e79f2
// See https://github.com/ocornut/imgui/issues/306
// v0.03: fixed grid offset issue, inverted sign of 'scrolling'
// Animated gif: https://cloud.githubusercontent.com/assets/8225057/9472357/c0263c04-4b4c-11e5-9fdf-2cd4f33f6582.gif


// Really dumb data structure provided for the example.
// Note that we storing links are INDICES (not ID) to make example code shorter, obviously a bad idea for any general purpose code.
void Graph::draw()
{
  ImGui::SetNextWindowSize(ImVec2(700, 600), ImGuiSetCond_FirstUseEver);
  if (!ImGui::Begin("Example: Custom Node Graph", &opened_))
  {
    ImGui::End();
    return;
  }

  if (!inited_)
  {
    nodes_.push_back(std::make_shared<SignalGenerator>(nodes_.size(), "Sine1", ImVec2(40, 50)));
    nodes_.push_back(std::make_shared<SignalGenerator>(nodes_.size(), "Sine2", ImVec2(40, 150)));
    nodes_.push_back(std::make_shared<Node>(nodes_.size(), "Combine", ImVec2(270, 80), 1.0f, ImColor(0, 200, 100), 2, 2));
    links_.push_back(NodeLink(links_.Size, 0, 2, 0));
    links_.push_back(NodeLink(links_.Size, 0, 2, 1));
    inited_ = true;
    std::cout << "initted graph\n";
  }

  // Draw a list of nodes on the left side
  bool open_context_menu = false;
  int node_hovered_in_list = -1;
  int node_hovered_in_scene = -1;
  ImGui::BeginChild("node_list", ImVec2(100, 0));
  ImGui::Text("Nodes");
  ImGui::Separator();
  for (int node_idx = 0; node_idx < static_cast<int>(nodes_.size()); node_idx++)
  {
    auto node = nodes_[node_idx];
    ImGui::PushID(node->id_);
    if (ImGui::Selectable(node->name_, node->id_ == node_selected_)) {
      node_selected_ = node->id_;
    }
    if (ImGui::IsItemHovered())
    {
      node_hovered_in_list = node->id_;
      open_context_menu |= ImGui::IsMouseClicked(1);
    }
    ImGui::PopID();
  }
  ImGui::EndChild();

  ImGui::SameLine();
  ImGui::BeginGroup();

  // Create our child canvas
  ImGui::Text("Hold middle mouse button to scroll (%.2f,%.2f)", scrolling_.x, scrolling_.y);
  ImGui::SameLine(ImGui::GetWindowWidth() - 100);
  ImGui::Checkbox("Show grid", &show_grid_);
  ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(1, 1));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  ImGui::PushStyleColor(ImGuiCol_ChildWindowBg, IM_COL32(60, 60, 70, 200));
  ImGui::BeginChild("scrolling_region", ImVec2(0, 0), true,
      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoMove);
  ImGui::PushItemWidth(120.0f);

  ImVec2 offset = ImGui::GetCursorScreenPos() + scrolling_;
  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  // Display grid
  if (show_grid_)
  {
    ImU32 GRID_COLOR = IM_COL32(200, 200, 200, 40);
    float GRID_SZ = 64.0f;
    ImVec2 win_pos = ImGui::GetCursorScreenPos();
    ImVec2 canvas_sz = ImGui::GetWindowSize();
    for (float x = fmodf(scrolling_.x, GRID_SZ); x < canvas_sz.x; x += GRID_SZ)
      draw_list->AddLine(ImVec2(x, 0.0f) + win_pos, ImVec2(x, canvas_sz.y) + win_pos, GRID_COLOR);
    for (float y = fmodf(scrolling_.y, GRID_SZ); y < canvas_sz.y; y += GRID_SZ)
      draw_list->AddLine(ImVec2(0.0f, y) + win_pos, ImVec2(canvas_sz.x, y) + win_pos, GRID_COLOR);
  }

  // Display links
  draw_list->ChannelsSplit(2);
  draw_list->ChannelsSetCurrent(0); // Background
  for (int link_idx = 0; link_idx < links_.Size; link_idx++)
  {
    NodeLink* link = &links_[link_idx];
    auto node_inp = nodes_[link->input_idx_];
    auto node_out = nodes_[link->output_idx_];
    ImVec2 p1 = offset + node_inp->GetOutputSlotPos(link->input_slot_);
    ImVec2 p2 = offset + node_out->GetInputSlotPos(link->output_slot_);
    draw_list->AddBezierCurve(p1, p1 + ImVec2(+50, 0), p2 + ImVec2(-50, 0),
        p2, IM_COL32(200, 200, 100, 255), 3.0f);
  }

  // Display nodes
  for (int node_idx = 0; node_idx < static_cast<int>(nodes_.size()); node_idx++)
  {
    auto node = nodes_[node_idx];
    ImGui::PushID(node->id_);
    node->draw(draw_list, offset, node_selected_, node_hovered_in_list,
        node_hovered_in_scene, open_context_menu);
    ImGui::PopID();
  }
  draw_list->ChannelsMerge();

  // Open context menu
  if (!ImGui::IsAnyItemHovered() && ImGui::IsMouseHoveringWindow() && ImGui::IsMouseClicked(1))
  {
    node_selected_ = node_hovered_in_list = node_hovered_in_scene = -1;
    open_context_menu = true;
  }
  if (open_context_menu)
  {
    ImGui::OpenPopup("context_menu");
    if (node_hovered_in_list != -1) {
      node_selected_ = node_hovered_in_list;
    }
    if (node_hovered_in_scene != -1) {
      node_selected_ = node_hovered_in_scene;
    }
  }

  // Draw context menu
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8, 8));
  if (ImGui::BeginPopup("context_menu"))
  {
    auto node = node_selected_ != -1 ? nodes_[node_selected_] : NULL;
    ImVec2 scene_pos = ImGui::GetMousePosOnOpeningCurrentPopup() - offset;
    if (node)
    {
      ImGui::Text("Node '%s'", node->name_);
      ImGui::Separator();
      if (ImGui::MenuItem("Rename..", NULL, false, false)) {}
      if (ImGui::MenuItem("Delete", NULL, false, false)) {}
      if (ImGui::MenuItem("Copy", NULL, false, false)) {}
    }
    else
    {
      if (ImGui::MenuItem("Add")) {
        nodes_.push_back(std::make_shared<Node>(nodes_.size(), "New node", scene_pos, 0.5f,
            ImColor(100, 100, 200), 2, 2));
      }
      if (ImGui::MenuItem("Paste", NULL, false, false)) {}
    }
    ImGui::EndPopup();
  }
  ImGui::PopStyleVar();

  // Scrolling
  if (ImGui::IsWindowHovered() && !ImGui::IsAnyItemActive() && ImGui::IsMouseDragging(2, 0.0f)) {
    scrolling_ = scrolling_ + ImGui::GetIO().MouseDelta;
  }

  ImGui::PopItemWidth();
  ImGui::EndChild();
  ImGui::PopStyleColor();
  ImGui::PopStyleVar(2);
  ImGui::EndGroup();

  ImGui::End();
}

void Graph::Node::draw(ImDrawList* draw_list, ImVec2& offset, int& node_selected,
    int& node_hovered_in_list, int& node_hovered_in_scene,
    bool& open_context_menu)
{
  ImVec2 node_rect_min = offset + pos_;

  // Display node contents first
  draw_list->ChannelsSetCurrent(1); // Foreground
  bool old_any_active = ImGui::IsAnyItemActive();
  ImGui::SetCursorScreenPos(node_rect_min + NODE_WINDOW_PADDING);
  ImGui::BeginGroup(); // Lock horizontal position
  ImGui::Text("%s", name_);
  ImGui::SliderFloat("##value", &value_, 0.0f, 1.0f, "Alpha %.2f");
  ImGui::ColorEdit3("##color", &color_.x);
  ImGui::EndGroup();

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
    node_selected = id_;
  }
  if (node_moving_active && ImGui::IsMouseDragging(0)) {
    pos_ = pos_ + ImGui::GetIO().MouseDelta;
  }

  ImU32 node_bg_color = (node_hovered_in_list == id_ ||
      node_hovered_in_scene == id_ ||
      (node_hovered_in_list == -1 && node_selected == id_)) ? IM_COL32(75, 75, 75, 255) : IM_COL32(60, 60, 60, 255);
  draw_list->AddRectFilled(node_rect_min, node_rect_max, node_bg_color, 4.0f);
  draw_list->AddRect(node_rect_min, node_rect_max, IM_COL32(100, 100, 100, 255), 4.0f);
  for (int slot_idx = 0; slot_idx < inputs_count_; slot_idx++) {
    draw_list->AddCircleFilled(offset + GetInputSlotPos(slot_idx),
        NODE_SLOT_RADIUS, IM_COL32(150, 150, 150, 150));
  }
  for (int slot_idx = 0; slot_idx < outputs_count_; slot_idx++) {
    draw_list->AddCircleFilled(offset + GetOutputSlotPos(slot_idx),
        NODE_SLOT_RADIUS, IM_COL32(150, 150, 150, 150));
  }
}

Graph::SignalGenerator::SignalGenerator(const int id, const char* name,
    const ImVec2& pos) : Node(id, name, pos, 0.0, ImColor(255, 0, 0, 255), 0, 1)
{

}

void Graph::SignalGenerator::update()
{
  Node::update();
}

void Graph::SignalGenerator::draw(ImDrawList* draw_list, ImVec2& offset, int& node_selected,
        int& node_hovered_in_list, int& node_hovered_in_scene,
        bool& open_context_menu)
{
  Node::draw(draw_list, offset, node_selected, node_hovered_in_list,
      node_hovered_in_scene, open_context_menu);
}
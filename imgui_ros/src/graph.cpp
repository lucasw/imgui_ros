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
#include <memory>

static const std::string topic = "";
Graph::Graph(const std::string name,
    std::shared_ptr<rclcpp::Node> node) :
    Widget(name, topic),
    node_(node)
{
  start_ = node->now();
}

// Creating a node graph editor for ImGui
// Quick demo, not production code! This is more of a demo of how to use ImGui to create custom stuff.
// Better version by @daniel_collin here https://gist.github.com/emoon/b8ff4b4ce4f1b43e79f2
// See https://github.com/ocornut/imgui/issues/306
// v0.03: fixed grid offset issue, inverted sign of 'scrolling'
// Animated gif: https://cloud.githubusercontent.com/assets/8225057/9472357/c0263c04-4b4c-11e5-9fdf-2cd4f33f6582.gif

void Graph::update(const rclcpp::Time& stamp)
{
  // std::cout << "update " << stamp.nanoseconds() << "\n";
  const double seconds = (stamp - start_).nanoseconds() / 1e9;
  for (auto links_pair : links_)
  {
    links_pair.second->update();
  }

  for (auto node_pair : nodes_)
  {
    node_pair.second->update(seconds);
  }
  stamp_ = stamp;
}

// Really dumb data structure provided for the example.
// Note that we storing links are INDICES (not ID) to make example code shorter, obviously a bad idea for any general purpose code.
void Graph::draw()
{
  const double seconds = (stamp_ - start_).nanoseconds() / 1e9;
  // std::cout << "draw " << seconds << "\n";

  ImGui::SetNextWindowSize(ImVec2(700, 600), ImGuiSetCond_FirstUseEver);

  if (!inited_)
  {
    // TODO(lucasw) if a node with the same name already exist it is not
    // going to destruct properly because of circular shared_ptrs - need to fix that.
    nodes_["Sine1"] = std::make_shared<SignalGenerator>("Sine1", ImVec2(40, 50));
    nodes_["Sine2"] = std::make_shared<SignalGenerator>("Sine2", ImVec2(40, 150));
    nodes_["Combine1"] = std::make_shared<SignalCombine>("Combine1", ImVec2(270, 80));

    // create the connections
    for (auto node_pair : nodes_) {
      node_pair.second->init();
    }

    linkNodes("Sine1", "signal", "Combine1", "in1");
    linkNodes("Sine2", "signal", "Combine1", "in2");

    inited_ = true;
    std::cout << "initted graph\n";
  }

  // Draw a list of nodes on the left side
  bool open_context_menu = false;
  int node_hovered_in_list = -1;
  int node_hovered_in_scene = -1;
  ImGui::BeginChild("node_list", ImVec2(200, 0));
  ImGui::Text("Nodes at time: %f", seconds);
  ImGui::Separator();
  int node_idx = 0;
  for (auto node_pair : nodes_)
  {
    auto node = node_pair.second;
    // TODO(lucasw)
    node->id_ = node_idx;
    ImGui::PushID(node_idx);
    #if 0
    if (ImGui::Selectable(node->name_.c_str(), node->id_ == node_selected_)) {
      node_selected_ = node->id_;
    }
    #endif
    if (ImGui::IsItemHovered())
    {
      node_hovered_in_list = node->id_;
      open_context_menu |= ImGui::IsMouseClicked(1);
    }
    ImGui::PopID();
    ++node_idx;
  }
  // ImGui::EndChild();

  // ImGui::BeginChild("link list", ImVec2(100, 0));
  ImGui::Separator();
  for (const auto link_pair : links_)
  {
    const auto link = link_pair.second;
    ImGui::Text("%s : %s -> %lu", link->name_.c_str(),
        link->input_->name_.c_str(), link->outputs_.size());
    for (const auto node_pair : link->outputs_) {
      ImGui::Text("  - %s", node_pair.second->name_.c_str());
    }
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
  for (auto link_pair : links_)
  {
    auto link = link_pair.second;
    link->draw(draw_list, offset);
  }

  // Display nodes
  for (auto node_pair : nodes_)
  {
    auto node = node_pair.second;
    ImGui::PushID(node->id_);
    node->draw(draw_list, offset, node_hovered_in_list,
        node_hovered_in_scene, open_context_menu,
        node_for_slot_selected_);
    ImGui::PopID();
  }
  draw_list->ChannelsMerge();

  #if 0
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
  #endif

  // Draw context menu
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8, 8));

  // TODO(lucasw) bring this back
  #if 0
  if (ImGui::BeginPopup("context_menu"))
  {
    auto node = node_selected_ != -1 ? nodes_[node_selected_] : NULL;
    ImVec2 scene_pos = ImGui::GetMousePosOnOpeningCurrentPopup() - offset;
    if (node)
    {
      ImGui::Text("Node '%s'", node->name_.c_str());
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
  #endif
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
}


void Graph::linkNodes(
    const std::string& output_node_name, const std::string& output_node_con_name,
    const std::string& input_node_name, const std::string& input_node_con_name)
{
  if (nodes_.count(output_node_name) < 1) {
    std::cerr << output_node_name << " doesn't exist\n";
    return;
  }
  auto output_node = nodes_[output_node_name];
  if (nodes_.count(input_node_name) < 1) {
    std::cerr << output_node_name << " doesn't exist\n";
    return;
  }
  auto input_node = nodes_[input_node_name];
  if (output_node->outputs_.count(output_node_con_name) < 1) {
    // TODO(lucasw) throw
    std::cerr << output_node_name << " input " << output_node_con_name << " doesn't exit\n";
    return;
  }
  auto output_con = output_node->outputs_[output_node_con_name];
  if (input_node->inputs_.count(input_node_con_name) < 1) {
    // TODO(lucasw) throw
    std::cerr << input_node_name << " input " << input_node_con_name << " doesn't exit\n";
    return;
  }
  auto input_con = input_node->inputs_[input_node_con_name];

  // next connect the ouput connector to the input connector

  const std::string link_output_con_name = input_node_name + "#" + input_node_con_name;
  if (input_con->link_ != nullptr) {
    if (input_con->link_->input_ == output_con) {
      // already linked, don't need to do anything
      return;
    }
    // break incoming link if any
    input_con->link_->outputs_.erase(link_output_con_name);
  }

  if (output_con->link_ == nullptr) {
    // need a new link here (TODO(lucasw) though every output may as well already have one)
    const std::string new_link_name = output_node_name + "#" + output_node_con_name;
    output_con->link_ = std::make_shared<Link>(new_link_name);
    links_[new_link_name] = output_con->link_;
    output_con->link_->input_ = output_con;
  }

  output_con->link_->outputs_[link_output_con_name] = input_con;
  input_con->link_ = output_con->link_;
}

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

namespace imgui_ros
{
static const std::string topic = "";
Graph::Graph(const std::string name,
    ros::NodeHandle& nh) :
    Widget(name, topic),
    nh_(nh)
{
  start_ = ros::Time::now();
}

// Creating a node graph editor for ImGui
// Quick demo, not production code! This is more of a demo of how to use ImGui to create custom stuff.
// Better version by @daniel_collin here https://gist.github.com/emoon/b8ff4b4ce4f1b43e79f2
// See https://github.com/ocornut/imgui/issues/306
// v0.03: fixed grid offset issue, inverted sign of 'scrolling'
// Animated gif: https://cloud.githubusercontent.com/assets/8225057/9472357/c0263c04-4b4c-11e5-9fdf-2cd4f33f6582.gif

void Graph::update(const ros::Time& stamp)
{
  // std::cout << "update " << stamp.nanoseconds() << "\n";
  const double seconds = (stamp - start_).toSec();

  if ((con_src_ != nullptr) && (con_dst_ != nullptr)) {
    if (con_src_->input_not_output_ != con_dst_->input_not_output_) {
      // linkNodes(con_src_, con_dst_);
      bool success;
      if (con_dst_->input_not_output_) {
        success = linkNodes(con_src_->parent_->name_, con_src_->name_,
                            con_dst_->parent_->name_, con_dst_->name_);
      } else {
        success = linkNodes(con_dst_->parent_->name_, con_dst_->name_,
                            con_src_->parent_->name_, con_src_->name_);
      }

      if (success) {
        con_src_ = nullptr;
      }
    }
    con_dst_ = nullptr;
  }

  // copy all the outputs to inputs
  for (auto links_pair : links_)
  {
    links_pair.second->update();
  }

  // process all the inputs into outputs
  for (auto node_pair : nodes_)
  {
    node_pair.second->update(seconds);
  }
  stamp_ = stamp;
}

void Graph::init()
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

  std::cout << "initted graph\n";
}

// Really dumb data structure provided for the example.
// Note that we storing links are INDICES (not ID) to make example code shorter, obviously a bad idea for any general purpose code.
void Graph::draw()
{
  const double seconds = (stamp_ - start_).toSec();
  // std::cout << "draw " << seconds << "\n";

  ImGui::SetNextWindowSize(ImVec2(700, 600), ImGuiCond_FirstUseEver);

  if (!inited_)
  {
    init();
    inited_ = true;
  }

  // Draw a list of nodes on the left side
  bool open_context_menu = false;
  std::shared_ptr<Node> node_hovered_in_list;
  std::shared_ptr<Node> node_hovered_in_scene;
  ImGui::BeginChild("nodes and link list", ImVec2(200, 0));
  ImGui::Text("Nodes at time: %f", seconds);
  ImGui::Separator();
  // int node_idx = 0;
  for (auto node_pair : nodes_)
  {
    auto node = node_pair.second;
    // ImGui::PushID(node_idx);
    if (ImGui::Selectable(node->name_.c_str(), node == node_selected_)) {
      node_selected_ = node;
    }
    if (ImGui::IsItemHovered())
    {
      node_hovered_in_list = node;
      open_context_menu |= ImGui::IsMouseClicked(1);
    }
    // ImGui::PopID();
    // ++node_idx;
  }
  // ImGui::EndChild();

  // ImGui::BeginChild("link list", ImVec2(100, 0));
  ImGui::Separator();
  for (const auto link_pair : links_)
  {
    // TODO(lucasw) make these selectable also
    const auto link = link_pair.second;
    ImGui::Text("%s : %s -> %lu", link->name_.c_str(),
        link->input_->name_.c_str(), link->outputs_.size());
    for (const auto node_pair : link->outputs_) {
      ImGui::Text("  - %s", node_pair.second->name_.c_str());
    }
  }

  ImGui::Separator();
  if (con_src_) {
    ImGui::Text("connector source %s %s",
        con_src_->parent_->name_.c_str(), con_src_->name_.c_str());
  } else {
    ImGui::Text("connector source - none");
  }
  // This should always be none, as soon as something is clicked it resets
  // to nullptr
  if (con_dst_) {
    ImGui::Text("connector dest %s %s",
        con_dst_->parent_->name_.c_str(), con_dst_->name_.c_str());
  } else {
    ImGui::Text("connector dest - none");
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
  int id = 0;
  for (auto node_pair : nodes_)
  {
    auto node = node_pair.second;
    ImGui::PushID(id);
    node->draw(draw_list, offset,
        node_hovered_in_list,
        node_hovered_in_scene,
        open_context_menu,
        con_src_, con_dst_);
    ImGui::PopID();
    ++id;
  }

  // draw a link from the last selected connector to current mouse position
  if ((con_src_ != nullptr) && (con_dst_ == nullptr)) {
    ImVec2 p1;
    ImVec2 p2;
    if (con_src_->input_not_output_) {
      p2 = offset + con_src_->getPos() + ImVec2(0.0, con_src_->size_.y * 0.5);
      p1 = ImGui::GetMousePos();
    } else {
      p2 = ImGui::GetMousePos();
      p1 = offset + con_src_->getEndPos();
    }

    draw_list->AddBezierCurve(
        p1, p1 + ImVec2(+80, 0),
        p2 + ImVec2(-80, 0), p2,
        IM_COL32(220, 190, 100, 205), 3.0f);
  }
  draw_list->ChannelsMerge();

  // Open context menu
  if (!ImGui::IsAnyItemHovered() && ImGui::IsWindowHovered() && ImGui::IsMouseClicked(1))
  {
    node_selected_ = node_hovered_in_list = node_hovered_in_scene = nullptr;
    open_context_menu = true;
  }
  if (open_context_menu)
  {
    ImGui::OpenPopup("context_menu");
    if (node_hovered_in_list != nullptr) {
      node_selected_ = node_hovered_in_list;
    }
    if (node_hovered_in_scene != nullptr) {
      node_selected_ = node_hovered_in_scene;
    }
  }

  // Draw context menu
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8, 8));

  if (ImGui::BeginPopup("context_menu"))
  {
    auto node = node_selected_;
    // ImVec2 scene_pos = ImGui::GetMousePosOnOpeningCurrentPopup() - offset;
    if (node)
    {
      ImGui::Text("Node '%s'", node->name_.c_str());
      ImGui::Separator();
      if (ImGui::MenuItem("TBD Rename..", NULL, false, false)) {}
      if (ImGui::MenuItem("TBD Delete", NULL, false, false)) {}
      if (ImGui::MenuItem("TBD Copy", NULL, false, false)) {}
    }
    else
    {
      // TODO(lucasw) add a menu item for every node type
      if (ImGui::MenuItem("TBD Add")) {
        // nodes_.push_back(std::make_shared<Node>(nodes_.size(), "New node", scene_pos, 0.5f,
        //    ImColor(100, 100, 200), 2, 2));
      }
      if (ImGui::MenuItem("TBD Paste", NULL, false, false)) {}
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
}


bool Graph::linkNodes(
    const std::string& output_node_name, const std::string& output_node_con_name,
    const std::string& input_node_name, const std::string& input_node_con_name)
{
  if (nodes_.count(output_node_name) < 1) {
    std::cerr << "output node '" << output_node_name << "' doesn't exist\n";
    return false;
  }
  auto output_node = nodes_[output_node_name];
  if (output_node->outputs_.count(output_node_con_name) < 1) {
    // TODO(lucasw) throw
    std::cerr << "'" << output_node_name << "' output '"
        << output_node_con_name << "' doesn't exit\n";
    return false;
  }
  auto output_con = output_node->outputs_[output_node_con_name];

  if (nodes_.count(input_node_name) < 1) {
    std::cerr << "input node '" << input_node_name << "' doesn't exist\n";
    return false;
  }
  auto input_node = nodes_[input_node_name];
  if (input_node->inputs_.count(input_node_con_name) < 1) {
    // TODO(lucasw) throw
    std::cerr << "'" << input_node_name << "' input '"
        << input_node_con_name << "' doesn't exit\n";
    return false;
  }
  auto input_con = input_node->inputs_[input_node_con_name];

  // next connect the ouput connector to the input connector

  const std::string link_output_con_name = input_node_name + "#" + input_node_con_name;
  if (input_con->link_ != nullptr) {
    if (input_con->link_->input_ == output_con) {
      // already linked, don't need to do anything
      return false;
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
  return true;
}

}  // namespace imgui_ros

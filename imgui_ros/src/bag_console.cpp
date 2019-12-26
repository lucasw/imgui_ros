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

// #define IMGUI_DEFINE_MATH_OPERATORS

#include <imgui.h>
#include <imgui_internal.h>
#include <imgui_ros/bag_console.h>

namespace imgui_ros
{

void BagConsole::draw()
{
  const int min = 0;
  const int max = msgs_.size();
  // TEMP before using built-in imgui console (if it actually works here)

  {
    int pos = position_;
    ImGui::SliderInt("scroll", &pos, min, max, "%d");
    position_ = pos;
  }

  ImGui::Checkbox("pause", &pause_);  // this causes messages to get lost
  ImGui::SameLine();
  size_t num_columns = 0;
  bool changed_columns = false;
  for (auto& column : columns_) {
    if (column.name_ != "time") {
      if (ImGui::Checkbox(column.name_.c_str(), &column.enable_)) {  // this causes messages to get lost
        changed_columns = true;
      }
      ImGui::SameLine();
    }
    if (column.enable_) {
      ++num_columns;
    }
  }

	const auto win_width = ImGui::GetWindowContentRegionMax().x - ImGui::GetWindowContentRegionMin().x;
  // TODO(lucasw) typeToString()
  // const std::string text = topic_;
  // ImGui::Text("%.*s", static_cast<int>(text.size()), text.data());
  std::lock_guard<std::mutex> lock(mutex_);

  const size_t start_ind = position_;

  // TODO(lucasw) have stamp optionally in unix time, or HH MM SS,
  // or seconds since some other timestamp
  //
  // Optionally have name, file, function, line, and topics in columns
  // Color code according to level: green, white, yellow, red, dark red (or black on red background)
  ImGui::Text("%zu / %zu log messages", start_ind, msgs_.size());

  ImGui::Columns(num_columns);
  size_t ind = 0;
  for (const auto& column : columns_) {
    if (column.enable_) {
      ImGui::Text("%s", column.name_.c_str());  // this causes messages to get lost
      ImGui::NextColumn();
      if (changed_columns || (count_ == 0)) {
        // TODO(lucasw) need to store live width and restore that
        // on changed_columns instead of going back to default.
        ImGui::SetColumnWidth(ind, win_width * column.width_);
      }
      ++ind;
    }
  }

  for (size_t i = start_ind; (i < start_ind + view_num) && (i < msgs_.size()); ++i) {
    const auto& msg = msgs_[i];
    for (const auto& column : columns_) {
      column.draw(msg, selected_, i);
    }
  }
  ImGui::Columns(1);

  ++count_;
}

void BagConsole::Column::draw(const rosgraph_msgs::Log::ConstPtr& msg,
    size_t& selected, size_t& i) const
{
  if (!enable_) {
    return;
  }
  const std::unordered_map<std::string, std::function<void()>> draw_ops{
    {"time", [&]() {
      const auto sel_flags = ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_PressedOnClick;
      const std::string stamp = std::to_string(msg->header.stamp.toSec());
      // TODO(lucasw) how to make any column selectable if time is disabled
      if (ImGui::Selectable((stamp + "##unique").c_str(), i == selected, sel_flags)) {
        // Don't add to rosout here
        std::cout << "selection " << i << " " << *msg << "\n";
        selected = i;
      }
      ImGui::NextColumn();
    }},
    {"msg", [&]() {
      ImGui::Text("%s", msg->msg.c_str());
      ImGui::NextColumn();
    }},
    {"name", [&]() {
      ImGui::Text("%s", msg->name.c_str());
      ImGui::NextColumn();
    }},
    {"file", [&](){
      ImGui::Text("%s", msg->file.c_str());
      ImGui::NextColumn();
    }},
    {"function", [&](){
      ImGui::Text("%s", msg->function.c_str());
      ImGui::NextColumn();
    }},
    {"line", [&](){
      ImGui::Text("%u", msg->line);
      ImGui::NextColumn();
    }}
  };

  draw_ops.find(name_)->second();
}

void BagConsole::callback(const typename rosgraph_msgs::LogConstPtr msg)
{
  // have mode where pause still pauses the screen but still records
  // new messages, but have to watch buffer fill levels.
  if (pause_) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  msgs_.push_back(msg);
  if (msgs_.size() >= max_num_) {
    msgs_.pop_front();
    if (selected_ < max_num_) {
      selected_--;
      // if the user has selected a message, keep it on screen instead
      // of scrolling past
      if ((position_ > 0) && (position_ < max_num_)) {
        position_ -= 1;
      }
    }
  }
}

}  // namespace imgui_ros

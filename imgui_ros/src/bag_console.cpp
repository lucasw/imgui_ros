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
#include <imgui_ros/bag_console.h>

namespace imgui_ros
{

void BagConsole::draw()
{
  const int min = 0;
  const int max = msgs_.size();
  // TEMP before using built-in imgui console (if it actually works here)
  int pos = position_;
  ImGui::SliderInt("scroll", &pos, min, max, "%d");
  position_ = pos;

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

  size_t num_columns = 6;
  ImGui::Columns(num_columns);
  ImGui::Text("Time");
  ImGui::NextColumn();
  ImGui::Text("msg");
  ImGui::NextColumn();
  ImGui::Text("name");
  ImGui::NextColumn();
  ImGui::Text("file");
  ImGui::NextColumn();
  ImGui::Text("function");
  ImGui::NextColumn();
  ImGui::Text("line");
  ImGui::NextColumn();

  for (size_t i = start_ind; (i < start_ind + view_num) && (i < msgs_.size()); ++i) {
    const auto& msg = msgs_[i];
    // ImGui::Text("%s", msg->msg.c_str());

    if (ImGui::Selectable((msg->msg + "##unique").c_str(), i == selected_,
                          ImGuiSelectableFlags_SpanAllColumns)) {
      if (i != selected_) {
        ROS_INFO_STREAM(i << " " << msg->msg);
        selected_ = i;
      }
    }
    ImGui::NextColumn();
    ImGui::Text("%f", msg->header.stamp.toSec());
    ImGui::NextColumn();
    ImGui::Text("%s", msg->name.c_str());
    ImGui::NextColumn();
    ImGui::Text("%s", msg->file.c_str());
    ImGui::NextColumn();
    ImGui::Text("%s", msg->function.c_str());
    ImGui::NextColumn();
    ImGui::Text("%u", msg->line);
    ImGui::NextColumn();
  }
  ImGui::Columns(1);

}

void BagConsole::callback(const typename rosgraph_msgs::LogConstPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  msgs_.push_back(msg);
  if (msgs_.size() >= max_num_) {
    msgs_.pop_front();
    if ((selected_ >= 0) && (selected_ < max_num_)) {
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

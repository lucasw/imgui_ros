/*
 * Copyright (c) 2018 Lucas Walter
 * November 2018
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

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <imgui_ros/viz2d.h>

// TODO(lucasw)
// namespace imgui_ros
Viz2D::Viz2D(const std::string name,
    const std::string frame_id,
    const std::vector<std::string>& frames,
    const double pixels_per_meter,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<rclcpp::Node> node) :
    Sub(name, frame_id, node),
    frame_id_(frame_id),
    frames_(frames),
    pixels_per_meter_(pixels_per_meter),
    tf_buffer_(tf_buffer)
{
  // RCLCPP_INFO(node->get_logger(), "new tf echo %s to %s", parent_.c_str(), child_.c_str());
}

void Viz2D::draw()
{
  ImGui::Text("%s", name_.c_str());
  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  // ImDrawList API uses screen coordinates!
  ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
  // Resize canvas to what's available
  ImVec2 canvas_size = ImGui::GetContentRegionAvail();
  if (canvas_size.x < 50.0f) canvas_size.x = 50.0f;
  if (canvas_size.y < 50.0f) canvas_size.y = 50.0f;
  ImVec2 corner = ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y);
  draw_list->AddRectFilledMultiColor(canvas_pos, corner,
            IM_COL32(50, 50, 50, 255), IM_COL32(50, 50, 60, 255),
            IM_COL32(60, 60, 70, 255), IM_COL32(50, 50, 60, 255));
  draw_list->AddRect(canvas_pos, corner,
          IM_COL32(255, 255, 255, 255));
  draw_list->PushClipRect(canvas_pos, corner, true);

  ImVec2 center = ImVec2(canvas_pos.x + canvas_size.x * 0.5,
      canvas_pos.y + canvas_size.y * 0.5);
  ImVec2 origin = center;

  const ImU32 connection = IM_COL32(255, 255, 0, 32);
  const ImU32 red = IM_COL32(255, 0, 0, 128);
  const ImU32 green = IM_COL32(0, 255, 0, 128);
  // const ImU32 blue = IM_COL32(0, 0, 255, 128);
  float len = 16;
  draw_list->AddLine(origin, ImVec2(center.x + len, center.y), red, 2.0f);
  draw_list->AddLine(origin, ImVec2(center.x, center.y + len), green, 2.0f);
  // TODO(lucasw) draw a grid

  for (auto frame : frames_) {
    try {
      geometry_msgs::msg::TransformStamped tf;
      tf = tf_buffer_->lookupTransform(frame_id_, frame, tf2::TimePointZero);
      const ImVec2 pos = ImVec2(center.x + tf.transform.translation.x * pixels_per_meter_,
          center.y + tf.transform.translation.y * pixels_per_meter_);
      // TODO(lucasw) need to transform points extended in x and y
      // a short distance away from the frame
      // origin to capture the rotation of the frame.
      draw_list->AddLine(origin, pos, connection, 1.0f);
      draw_list->AddLine(pos, ImVec2(pos.x + len, pos.y), red, 2.0f);
      draw_list->AddLine(pos, ImVec2(pos.x, pos.y + len), green, 2.0f);
    } catch (tf2::TransformException& ex) {
      // ImGui::Text("%s", ex.what());
    }
  }

  draw_list->PopClipRect();
}

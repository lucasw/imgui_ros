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
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <imgui_ros/viz2d.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// TODO(lucasw) overload <<
std::string printVec(const geometry_msgs::msg::Vector3& vec)
{
  std::stringstream ss;
  ss << vec.x << ", "
      << vec.y << ", "
      << vec.z << "  ";
  return ss.str();
}

std::string printVec(const geometry_msgs::msg::Vector3Stamped& vec)
{
  std::stringstream ss;
  ss << vec.header.frame_id << ": " << ": " << printVec(vec.vector);
  return ss.str();
}

// TODO(lucasw)
// namespace imgui_ros
Viz2D::Viz2D(const std::string name,
    const std::string topic,
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
  marker_sub_ = node_->create_subscription<visualization_msgs::msg::Marker>(topic,
      std::bind(&Viz2D::markerCallback, this, std::placeholders::_1));
}

void Viz2D::markerCallback(const visualization_msgs::msg::Marker::SharedPtr msg)
{
  // TODO(lucasw) handle DELETE later
  markers_[msg->ns][msg->id] = msg;
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
  // make background and border around the canvas
  draw_list->AddRectFilledMultiColor(canvas_pos, corner,
            IM_COL32(50, 50, 50, 255), IM_COL32(50, 50, 60, 255),
            IM_COL32(60, 60, 70, 255), IM_COL32(50, 50, 60, 255));
  draw_list->AddRect(canvas_pos, corner,
          IM_COL32(255, 255, 255, 255));
  draw_list->PushClipRect(canvas_pos, corner, true);

  ImVec2 center = ImVec2(canvas_pos.x + canvas_size.x * 0.5,
      canvas_pos.y + canvas_size.y * 0.5);
  ImVec2 origin = center;

  const ImU32 connection_color = IM_COL32(255, 255, 0, 32);
  const ImU32 red = IM_COL32(255, 0, 0, 180);
  const ImU32 green = IM_COL32(0, 255, 0, 180);
  const ImU32 blue = IM_COL32(0, 0, 255, 180);
  std::vector<ImU32> colors;
  colors.push_back(red);
  colors.push_back(green);
  colors.push_back(blue);
  const float len = 32;
  // TODO(lucasw) draw a grid

  const double sc = pixels_per_meter_;
  for (auto frame : frames_) {
    try {
      geometry_msgs::msg::TransformStamped tf;
      tf = tf_buffer_->lookupTransform(frame_id_, frame, tf2::TimePointZero);
      auto pos = tf.transform.translation;
      const ImVec2 im_pos = ImVec2(center.x + pos.x * sc,
          center.y + pos.y * sc);
      draw_list->AddLine(origin, im_pos, connection_color, 1.0f);

      // draw_list->AddText(ImGui::GetFont(), ImGui::GetFontSize(),
      //    im_pos, blue, frame.c_str(), NULL, 0.0f, nullptr);
      const ImU32 text_color = IM_COL32(200, 200, 255, 230);
      draw_list->AddText(ImVec2(im_pos.x + 10, im_pos.y + 5),
          text_color, frame.c_str(), NULL);

      // TODO(lucasw) need to transform points extended in x and y
      // a short distance away from the frame
      // origin to capture the rotation of the frame.
      auto tf_pos_origin = geometry_msgs::msg::Vector3Stamped();
      tf_pos_origin.header.frame_id = frame;
      tf_pos_origin.header.stamp = tf.header.stamp;
      tf_pos_origin.vector.x = 0;
      tf_pos_origin.vector.y = 0;
      tf_pos_origin.vector.z = 0;
      auto tf_pos_x = tf_pos_origin;
      tf_pos_x.vector.x = 1.0;
      auto tf_pos_y = tf_pos_origin;
      tf_pos_y.vector.y = 1.0;
      auto tf_pos_z = tf_pos_origin;
      tf_pos_z.vector.z = 1.0;
      std::vector<geometry_msgs::msg::Vector3Stamped> vectors;
      vectors.push_back(tf_pos_x);
      vectors.push_back(tf_pos_y);
      vectors.push_back(tf_pos_z);

      // origin_out should be the same as pos above- it isn't, is it because
      // the transform only rotates?
      // tf_buffer_->transform(
      //    tf_pos_origin, origin_out, frame_id_);
      // std::cout << "o " << printVec(tf_pos_origin) << " -> " << printVec(origin_out) << "\n";

      for (size_t i = 0; i < vectors.size() && i < colors.size(); ++i) {
        geometry_msgs::msg::Vector3Stamped vector_in_viz_frame;
        tf_buffer_->transform(vectors[i], vector_in_viz_frame, frame_id_);
        // std::cout << i << " " << printVec(vector) << " -> "
        //     << printVec(vector_in_viz_frame) << "\n";
        const auto im_vec = ImVec2(im_pos.x + vector_in_viz_frame.vector.x * len,
            im_pos.y + vector_in_viz_frame.vector.y * len);
        draw_list->AddLine(im_pos, im_vec, colors[i], 2.0f);
      }
    } catch (tf2::TransformException& ex) {
      // ImGui::Text("%s", ex.what());
    }
  }

  for (auto marker_ns : markers_) {
    for (auto marker_pair : marker_ns.second) {
      try {
        auto marker = (marker_pair.second);
        geometry_msgs::msg::TransformStamped tf;
        tf = tf_buffer_->lookupTransform(frame_id_, marker->header.frame_id, tf2::TimePointZero);
        std::vector<geometry_msgs::msg::PointStamped> rect_3d;
        geometry_msgs::msg::PointStamped pt;
        pt.header.frame_id = marker->header.frame_id;
        pt.header.stamp = tf.header.stamp;

        pt.point.x = -marker->scale.x * 0.5;
        pt.point.y = -marker->scale.y * 0.5;
        rect_3d.push_back(pt);
        pt.point.x = -marker->scale.x * 0.5;
        pt.point.y = marker->scale.y * 0.5;
        rect_3d.push_back(pt);
        pt.point.x = marker->scale.x * 0.5;
        pt.point.y = marker->scale.y * 0.5;
        rect_3d.push_back(pt);
        pt.point.x = marker->scale.x * 0.5;
        pt.point.y = -marker->scale.y * 0.5;
        rect_3d.push_back(pt);
        std::vector<ImVec2> rect_2d;
        for (auto pt : rect_3d) {
          geometry_msgs::msg::PointStamped pt_in_viz_frame;
          tf_buffer_->transform(pt, pt_in_viz_frame, frame_id_);
          rect_2d.push_back(ImVec2(origin.x + pt_in_viz_frame.point.x * sc,
              origin.y + pt_in_viz_frame.point.y * sc));
        }
        for (size_t i = 0; i < rect_2d.size(); ++i) {
          draw_list->AddLine(rect_2d[i], rect_2d[(i + 1) % rect_2d.size()],
              IM_COL32(marker->color.r * 255,
                  marker->color.g * 255,
                  marker->color.b * 255,
                  marker->color.a * 255),
              2.0f);
        }
      } catch (tf2::TransformException& ex) {

      }
    }  // loop through marker ids in this namespace
  }  // loop through marker namespace sets
  draw_list->PopClipRect();
}

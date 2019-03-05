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
#include <math.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// TODO(lucasw) move to utility header
// TODO(lucasw) the ignore doesn't work
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvariadic-macros"

#define LOG(__type__, msg, ...) { if (node_) {std::shared_ptr<rclcpp::Node> node = node_.lock(); if (node) { RCLCPP_##__type__(node->get_logger(), msg, ##__VA_ARGS__) }}}
#define LOG0(__type__, msg) { std::shared_ptr<rclcpp::Node> node = node_.lock(); if (node) { RCLCPP_##__type__(node->get_logger(), msg) }}

#define INFO(msg, ...) LOG(INFO, msg, ##__VA_ARGS__)
// #define INFO(msg, ...) { std::shared_ptr<rclcpp::Node> node = node_.lock(); if (node) { RCLCPP_INFO(node->get_logger(), msg, ##__VA_ARGS__) }}

#define WARN(msg, ...) LOG(WARN, msg, ##__VA_ARGS__)
#define WARN0(msg) LOG0(WARN, msg)
// { std::shared_ptr<rclcpp::Node> node = node_.lock(); if (node) { RCLCPP_WARN(node->get_logger(), msg, ##__VA_ARGS__) }}

// #define ERROR(msg, ...) LOG(ERROR, msg, ##__VA_ARGS__)
// #define DEBUG(msg, ...) LOG(ERROR, msg, ##__VA_ARGS__)

#pragma GCC diagnostic pop

namespace imgui_ros
{
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
    tf_buffer_(tf_buffer),
    pixels_per_meter_(pixels_per_meter)
{
  // RCLCPP_INFO(node->get_logger(), "new tf echo %s to %s", parent_.c_str(), child_.c_str());
  marker_sub_ = node->create_subscription<visualization_msgs::msg::Marker>(topic,
      std::bind(&Viz2D::markerCallback, this, std::placeholders::_1));
}

void Viz2D::markerCallback(const visualization_msgs::msg::Marker::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  // TODO(lucasw) handle DELETE later
  if ((msg->action == visualization_msgs::msg::Marker::ADD) ||
     (msg->action == visualization_msgs::msg::Marker::MODIFY)) {
     // TODO(lucasw) these aren't working, are failing in strange ways
    // INFO("adding/modifying marker %s %d %s", msg->ns, msg->id, msg->header.frame_id);
    std::cout << "viz add/modify " << msg->ns << " " << msg->id << " "
        << msg->header.frame_id << "\n";
    markers_[msg->ns][msg->id] = msg;
  } else if (msg->action == visualization_msgs::msg::Marker::DELETE) {
    if ((markers_.count(msg->ns) > 0) && (markers_[msg->ns].count(msg->id)) > 0) {
      // WARN("erasing marker %s %d %s", msg->ns, msg->id, msg->header.frame_id);
      std::cout << "viz erasing " << msg->ns << " " << msg->id << " "
          << msg->header.frame_id << "\n";
      markers_[msg->ns].erase(msg->id);
    }
  } else if (msg->action == visualization_msgs::msg::Marker::DELETEALL) {
    // WARN0("clearing markers");
    std::cout << "viz clearing markers" << "\n";
    markers_.clear();
  }
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

  ImGui::InvisibleButton((name_ + "_canvas").c_str(), canvas_size);
  ImVec2 mouse_pos_in_canvas = ImVec2(ImGui::GetIO().MousePos.x - canvas_pos.x, ImGui::GetIO().MousePos.y - canvas_pos.y);

  if (dragging_view_) {
    // This allows continued dragging outside the canvas
    offset_ = ImVec2(
        offset_.x + mouse_pos_in_canvas.x - drag_point_.x,
        offset_.y + mouse_pos_in_canvas.y - drag_point_.y);
    drag_point_ = mouse_pos_in_canvas;
    if (!ImGui::IsMouseDown(0)) {
      dragging_view_ = false;
    }
  }
  if (dragging_scale_) {
    // drag_point_ = mouse_pos_in_canvas;
    const float base = 100.0;
    const float diff = mouse_pos_in_canvas.y - drag_point_.y;
    pixels_per_meter_live_ = pixels_per_meter_ * exp(diff / base);
    if (!ImGui::IsMouseDown(1)) {
      dragging_scale_ = false;
      pixels_per_meter_ = pixels_per_meter_live_;
    }
  }
  if (ImGui::IsItemHovered()) {
    if (!dragging_view_ && ImGui::IsMouseClicked(0)) {
      drag_point_ = mouse_pos_in_canvas;
      dragging_view_ = true;
    }
    if (!dragging_scale_ && ImGui::IsMouseClicked(1)) {
      drag_point_ = mouse_pos_in_canvas;
      dragging_scale_ = true;
    }
  }

  ImVec2 origin = ImVec2(center.x + offset_.x, center.y + offset_.y);

  double sc = pixels_per_meter_;
  if (dragging_scale_)
    sc = pixels_per_meter_live_;

  // TODO(lucasw) draw a grid
  drawTf(draw_list, origin, center, sc);
  drawMarkers(draw_list, origin, center, sc);
  draw_list->PopClipRect();
}

void Viz2D::drawTf(ImDrawList* draw_list, ImVec2 origin, ImVec2 center,
    const float sc)
{
  (void)center;
  const ImU32 connection_color = IM_COL32(255, 255, 0, 32);

  const ImU32 red = IM_COL32(255, 0, 0, 180);
  const ImU32 green = IM_COL32(0, 255, 0, 180);
  const ImU32 blue = IM_COL32(0, 0, 255, 180);
  std::vector<ImU32> colors;
  colors.push_back(red);
  colors.push_back(green);
  colors.push_back(blue);

  const float len = 0.25;
  for (auto frame : frames_) {
    try {
      geometry_msgs::msg::TransformStamped tf;
      tf = tf_buffer_->lookupTransform(frame_id_, frame, tf2::TimePointZero);
      #if 0
      {
        std::shared_ptr<rclcpp::Node> node = node_.lock();
        if (node && ((node->now().nanoseconds() / 1e9 - tf.header.stamp.sec) > 4)) {
          continue;
        }
      }
      #endif

      auto pos = tf.transform.translation;
      const ImVec2 im_pos = ImVec2(origin.x + pos.x * sc,
          origin.y + pos.y * sc);
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
        const auto im_vec = ImVec2(
            im_pos.x + vector_in_viz_frame.vector.x * len * sc,
            im_pos.y + vector_in_viz_frame.vector.y * len * sc);
        draw_list->AddLine(im_pos, im_vec, colors[i], 2.0f);
      }
    } catch (tf2::TransformException& ex) {
      // ImGui::Text("%s", ex.what());
    }
  }
}  // draw tf

void Viz2D::drawMarkers(ImDrawList* draw_list, ImVec2 origin, ImVec2 center,
    const float sc)
{
  (void)center;
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto marker_ns : markers_) {
    std::vector<int> markers_to_remove;
    for (auto marker_pair : marker_ns.second) {
      try {
        auto marker = marker_pair.second;
        geometry_msgs::msg::TransformStamped tf;
        tf = tf_buffer_->lookupTransform(frame_id_, marker->header.frame_id, tf2::TimePointZero);
        {
          std::shared_ptr<rclcpp::Node> node = node_.lock();
          #if 0
          // TODO(lucasw) this doesn't work when the tf is static, the stamp is zero
          if (node && ((node->now().nanoseconds() / 1e9 - tf.header.stamp.sec) > 4)) {
            RCLCPP_INFO(node->get_logger(), "removing old marker %s",
                marker->header.frame_id.c_str());
            markers_to_remove.push_back(marker_pair.first);
            continue;
          }
          #endif
        }
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

        // TODO(lucasw) later could use orientation also
        auto offset = marker->pose.position;
        std::vector<ImVec2> rect_2d;
        for (auto pt : rect_3d) {
          pt.point.x += offset.x;
          pt.point.y += offset.y;
          pt.point.z += offset.z;

          geometry_msgs::msg::PointStamped pt_in_viz_frame;
          tf_buffer_->transform(pt, pt_in_viz_frame, frame_id_);
          rect_2d.push_back(ImVec2(origin.x + pt_in_viz_frame.point.x * sc,
              origin.y + pt_in_viz_frame.point.y * sc));
        }
        ImVec2 text_pos = rect_2d[0];
        for (size_t i = 0; i < rect_2d.size(); ++i) {
          auto x = rect_2d[i].x;
          auto y = rect_2d[i].y;
          if (x < text_pos.x)
            text_pos.x = x;
          if (y > text_pos.y)
            text_pos.y = y;
          draw_list->AddLine(rect_2d[i], rect_2d[(i + 1) % rect_2d.size()],
              IM_COL32(marker->color.r * 255,
                  marker->color.g * 255,
                  marker->color.b * 255,
                  marker->color.a * 255),
              2.0f);
        }

        const ImU32 text_color = IM_COL32(250, 200, 225, 230);
        draw_list->AddText(ImVec2(text_pos.x + 1, text_pos.y + 3),
            text_color, marker->text.c_str(), NULL);
      } catch (tf2::TransformException& ex) {

      }
    }  // loop through marker ids in this namespace

    for (auto ind : markers_to_remove) {
      // TODO(lucasw) is this changing the outer container?
      marker_ns.second.erase(ind);
    }
  }  // loop through marker namespace sets
}
}  // namespace imgui_ros

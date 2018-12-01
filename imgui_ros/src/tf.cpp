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
#include <imgui_ros/tf.h>
#include <tf2/LinearMath/Matrix3x3.h>


// TODO(lucasw)
// namespace imgui_ros
TfEcho::TfEcho(const std::string name,
    const std::string parent, const std::string child,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<rclcpp::Node> node) :
    Sub(name, parent, node),
    parent_(parent),
    child_(child),
    tf_buffer_(tf_buffer)
{
  RCLCPP_DEBUG(node->get_logger(), "new tf echo %s to %s", parent_.c_str(), child_.c_str());
}

void TfEcho::draw()
{
  try {
    geometry_msgs::msg::TransformStamped tf;
    tf = tf_buffer_->lookupTransform(parent_, child_, tf2::TimePointZero);
    std::stringstream ss;
    ss << name_ << " tf: "
        << tf.header.stamp.sec << "." << tf.header.stamp.nanosec
        << " " << tf.transform.translation.x
        << " " << tf.transform.translation.y
        << " " << tf.transform.translation.z
        << ", " << tf.transform.rotation.x
        << " " << tf.transform.rotation.y
        << " " << tf.transform.rotation.z
        << " " << tf.transform.rotation.w;
    ImGui::Text("%s", ss.str().c_str());
  } catch (tf2::TransformException& ex) {
    ImGui::Text("%s", ex.what());
  }
}

TfBroadcaster::TfBroadcaster(const std::string name,
    const std::string parent, const std::string child,
    const double min, const double max,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<rclcpp::Node> node) :
    Pub(name, parent, node),
    min_(min),
    max_(max),
    tf_buffer_(tf_buffer)
{
  RCLCPP_DEBUG(node->get_logger(), "new tf echo %s to %s", parent.c_str(), child.c_str());

  ts_.header.frame_id = parent;
  ts_.child_frame_id = child;
  ts_.transform.rotation.w = 1.0;

  double update_rate = 30.0;
  int period = 1000 / update_rate;
  timer_ = node_->create_wall_timer(std::chrono::milliseconds(period),
      std::bind(&TfBroadcaster::update, this));
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
}

void TfBroadcaster::update()
{
  if (ts_.header.frame_id == "")
    return;
  if (ts_.child_frame_id == "")
    return;

  tf_broadcaster_->sendTransform(ts_);
}

bool inputText(const std::string name, std::string& text)
{
  const size_t sz = 64;
  char buf[sz];

  const size_t sz2 = (text.size() > (sz - 1)) ? (sz - 1) : text.size();
  strncpy(buf, text.c_str(), sz2);
  buf[sz2] = '\0';
  const bool changed = ImGui::InputText(name.c_str(), buf, sz,
      ImGuiInputTextFlags_EnterReturnsTrue);
  if (changed) {
    text = buf;
    return true;
  }
  return false;
}

void TfBroadcaster::draw()
{
  // TODO(lucasw) lock guard around ts usage?
  inputText("parent", ts_.header.frame_id);
  inputText("child", ts_.child_frame_id);
  ImGui::SliderScalar("x", ImGuiDataType_Double,
      &ts_.transform.translation.x, &min_, &max_, "%lf");
  ImGui::SliderScalar("y", ImGuiDataType_Double,
      &ts_.transform.translation.y, &min_, &max_, "%lf");
  ImGui::SliderScalar("z", ImGuiDataType_Double,
      &ts_.transform.translation.z, &min_, &max_, "%lf");

  double roll, pitch, yaw;
  double min = -3.2;
  double max = 3.2;
  tf2::Quaternion quat(
    ts_.transform.rotation.x,
    ts_.transform.rotation.y,
    ts_.transform.rotation.z,
    ts_.transform.rotation.w);
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  ImGui::SliderScalar("roll", ImGuiDataType_Double,
      &roll, &min, &max, "%lf");
  ImGui::SliderScalar("pitch", ImGuiDataType_Double,
      &pitch, &min, &max, "%lf");
  ImGui::SliderScalar("yaw", ImGuiDataType_Double,
      &yaw, &min, &max, "%lf");
  quat.setRPY(roll, pitch, yaw);
  ts_.transform.rotation.x = quat.x();
  ts_.transform.rotation.y = quat.y();
  ts_.transform.rotation.z = quat.z();
  ts_.transform.rotation.w = quat.w();
}

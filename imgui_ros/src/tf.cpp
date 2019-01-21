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

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <imgui_ros/tf.h>
// #include <imgui_internal.h>  // for PushMulti
#include <iomanip>
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

void rot2RPY(const geometry_msgs::msg::Quaternion rotation,
    double& roll, double& pitch, double& yaw)
{
  tf2::Quaternion quat(
      rotation.x,
      rotation.y,
      rotation.z,
      rotation.w);
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

void putText(const std::string& text)
{
  ImGui::Text("%s", text.c_str());
}

void putText(const std::stringstream& text)
{
  putText(text.str());
}

void putText(const std::string& label, double value)
{
  std::stringstream ss;
  ss << label;
  if ((value == 0.0) && (std::signbit(value)))
    value = 0.0;
  if (!std::signbit(value))
    ss << " ";
  ss << value;
  putText(ss);
}

void TfEcho::draw()
{
  ImGui::Separator();
  ImGui::PushID(name_.c_str());
  try {
    geometry_msgs::msg::TransformStamped tf;
    tf = tf_buffer_->lookupTransform(parent_, child_, tf2::TimePointZero);
    std::stringstream ss;
    ss << name_ << " " << parent_ << " -> " << child_ << "\n" << "time: "
    //    << std::setprecision(3) << std::setw(4) << std::setfill('0') << std::internal
        << tf.header.stamp.sec << "." << tf.header.stamp.nanosec;
    ImGui::Text("%s", ss.str().c_str());

    // ImGui::BeginChild("xyz");
    // ImGui::PushID("xyz");
    {
      ImGui::Columns(3);
      putText("x: ", tf.transform.translation.x);
      ImGui::NextColumn();
      putText("y: ", tf.transform.translation.y);
      ImGui::NextColumn();
      putText("z: ", tf.transform.translation.z);
      ImGui::NextColumn();
    }
    // ImGui::PopID();
    // ImGui::EndChild();

    double roll, pitch, yaw;
    rot2RPY(tf.transform.rotation, roll, pitch, yaw);

    putText("r: ", roll);
    ImGui::NextColumn();
    putText("p: ", pitch);
    ImGui::NextColumn();
    putText("y: ", yaw);
    ImGui::NextColumn();

    ImGui::Columns(1);
  } catch (tf2::TransformException& ex) {
    ImGui::Text("%s", ex.what());
  }
  ImGui::PopID();
}

///////////////////////////////////////////////////////////////////////////////
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
  default_ts_.transform.rotation.w = 1.0;
}

TfBroadcaster::TfBroadcaster(
    const imgui_ros::msg::TfWidget& tf,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<rclcpp::Node> node) :
    Pub(tf.name, tf.transform_stamped.header.frame_id, node),
    min_(tf.min),
    max_(tf.max),
    ts_(tf.transform_stamped),
    default_ts_(tf.transform_stamped),
    tf_buffer_(tf_buffer)
{
  RCLCPP_INFO(node->get_logger(), "new tf echo %s to %s, %f",
      ts_.header.frame_id.c_str(),
      ts_.child_frame_id.c_str(),
      default_ts_.transform.translation.x);
}

#if 0
void TfBroadcaster::update(const rclcpp::Time& stamp)
{
  (void)stamp;
  if (ts_.header.frame_id == "")
    return;
  if (ts_.child_frame_id == "")
    return;

#if 0
  auto node = node_.lock();
  if (node) {
    ts_.header.stamp = node->now();
  }
#endif
  // tf_broadcaster_->sendTransform(ts_);
  // tf2_msgs::msg::TFMessage tfs;
  // tfs.transforms.push_back(ts_);
  // tf_pub_->publish(tfs);
}
#endif

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
  if (ImGui::CollapsingHeader((name_ + "##header").c_str())) {
  // ImGui::Separator();
  ImGui::PushID(name_.c_str());
  // TODO(lucasw) lock guard around ts usage?
  inputText("parent", ts_.header.frame_id);
  inputText("child", ts_.child_frame_id);

  double min, max;

  min = default_ts_.transform.translation.x + min_;
  max = default_ts_.transform.translation.x + max_;
  ImGui::SliderScalar("x", ImGuiDataType_Double,
      &ts_.transform.translation.x, &min, &max, "%lf");

  min = default_ts_.transform.translation.y + min_;
  max = default_ts_.transform.translation.y + max_;
  ImGui::SliderScalar("y", ImGuiDataType_Double,
      &ts_.transform.translation.y, &min, &max, "%lf");

  min = default_ts_.transform.translation.z + min_;
  max = default_ts_.transform.translation.z + max_;
  ImGui::SliderScalar("z", ImGuiDataType_Double,
      &ts_.transform.translation.z, &min, &max, "%lf");

  double roll, pitch, yaw;
  rot2RPY(ts_.transform.rotation, roll, pitch, yaw);

  min = -3.2;
  max = 3.2;
  ImGui::SliderScalar("roll", ImGuiDataType_Double,
      &roll, &min, &max, "%lf");

  // TODO(lucasw) need to eliminate glitches from going beyond pi/2
  double pitch_min = -1.57;
  double pitch_max = 1.57;
  ImGui::SliderScalar("pitch", ImGuiDataType_Double,
      &pitch, &pitch_min, &pitch_max, "%lf");

  ImGui::SliderScalar("yaw", ImGuiDataType_Double,
      &yaw, &min, &max, "%lf");

  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  ts_.transform.rotation.x = quat.x();
  ts_.transform.rotation.y = quat.y();
  ts_.transform.rotation.z = quat.z();
  ts_.transform.rotation.w = quat.w();
  ImGui::PopID();
  }
}

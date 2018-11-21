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
  RCLCPP_INFO(node->get_logger(), "new tf echo %s to %s", parent_.c_str(), child_.c_str());
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

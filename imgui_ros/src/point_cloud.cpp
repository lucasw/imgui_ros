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

// #include "imgui.h"
// #include "imgui_impl_opengl3.h"
// #include "imgui_impl_sdl.h"
// #include <imgui_ros/AddWindow.h>
#include <algorithm>
#include <imgui_ros/point_cloud.h>
// #include <opencv2/highgui.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


PointCloud::PointCloud(const std::string name, const std::string topic,
    std::shared_ptr<rclcpp::Node> node
    ) :
    Sub(name, topic, node)
{
  sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(topic,
      std::bind(&PointCloud::pointCloud2Callback, this, std::placeholders::_1));
}

void PointCloud::pointCloud2Callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  msg_ = msg;
  // std::cout << "new point cloud " << msg_->data.size() << "\n";
  // this requires pcl::console::print to be linked in, the code is identical
  // to below anyhow.
  pcl::fromROSMsg(*msg,  cloud_);
}

void PointCloud::draw()
{
  #if 1
  {
    int num_points = cloud_.points.size();
    ImGui::Text("point cloud points %d", num_points);
  }
  #else
  if (msg_) {
    ImGui::Text("point cloud data size %d", static_cast<int>(msg_->data.size()));
  }
  #endif
}

void PointCloud::render()
{
}

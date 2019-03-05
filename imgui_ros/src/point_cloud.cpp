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

namespace imgui_ros
{
PointCloud::PointCloud(const std::string name, const std::string topic,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<rclcpp::Node> node
    ) :
    Sub(name, topic, node)
{
  shape_ = std::make_shared<Shape>(name + "_shape",
      "", "default", "default", tf_buffer);
  shape_->draw_mode_ = 2;  // GL_POINTS;
  // this may not exist as an available texture-
  // need an AddPointCloud service that specifies this instead of using Widget
  shape_->emission_texture_ = "white";

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

  shape_->frame_id_ = msg->header.frame_id;
  shape_->vertices_.resize(cloud_.points.size());
  shape_->indices_.resize(cloud_.points.size());
  for (size_t i = 0; i < cloud_.points.size(); ++i) {
    auto& cpt = cloud_.points[i];
    DrawVert pt;
    pt.pos.x = cpt.x;
    pt.pos.y = cpt.y;
    pt.pos.z = cpt.z;

    uint32_t rgb = *reinterpret_cast<int*>(&cpt.rgb);
    uint8_t r = (rgb >> 16) & 0x0000ff;
    uint8_t g = (rgb >> 8)  & 0x0000ff;
    uint8_t b = (rgb)       & 0x0000ff;
    pt.col.x = static_cast<float>(r) / 255.0;
    pt.col.y = static_cast<float>(g) / 255.0;
    pt.col.z = static_cast<float>(b) / 255.0;
    pt.col.w = 1.0;  // alpha
    shape_->vertices_[i] = pt;
    shape_->indices_[i] = i;
  }
  shape_->init();
}

void PointCloud::draw()
{
  #if 1
  int num_points = cloud_.points.size();
  ImGui::Text("point cloud points %d", num_points);
  #else
  if (msg_) {
    ImGui::Text("point cloud data size %d", static_cast<int>(msg_->data.size()));
  }
  #endif
}

void PointCloud::render()
{
}
}  // namespace imgui_ros

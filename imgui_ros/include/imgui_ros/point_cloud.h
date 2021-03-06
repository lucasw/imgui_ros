/*
 * Copyright (c) 2017 Lucas Walter
 * June 2017
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

#ifndef IMGUI_ROS_POINT_CLOUD_H
#define IMGUI_ROS_POINT_CLOUD_H

#include <imgui.h>
#include <imgui_ros/sub.h>
#include <imgui_ros/surface.h>
#include <map>
#include <memory>
#include <mutex>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_msgs/TFMessage.h>
#include <vector>


namespace imgui_ros
{
struct PointCloud : public Sub
{
  PointCloud(const std::string name, const std::string topic,
      std::shared_ptr<tf2_ros::Buffer> tf_buffer,
      ros::NodeHandle& node);
  ~PointCloud() {}

  virtual void draw();
  void render();
  std::shared_ptr<Shape> shape_;
protected:
  pcl::PointCloud<pcl::PointXYZRGB> cloud_;
  sensor_msgs::PointCloud2::ConstPtr msg_;
  void pointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr msg);
  ros::Subscriber sub_;
};

}  // namespace imgui_ros
#endif  // IMGUI_ROS_POINT_CLOUD_H

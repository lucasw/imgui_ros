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

#ifndef IMGUI_ROS_IMAGE_TRANSFER_H
#define IMGUI_ROS_IMAGE_TRANSFER_H

#include <deque>
#include <imgui.h>
#include <imgui_ros/imgui_impl_opengl3.h>
#include <imgui_ros/window.h>
#include <mutex>
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace imgui_ros
{
// TODO(lucasw) get rid of this and restore pub/sub to where needed
class ImageTransfer
{
public:
  ImageTransfer();
  ~ImageTransfer();
  virtual void postInit();

  ros::Timer update_timer_;

  bool getSub(const std::string& topic, sensor_msgs::ImagePtr& image);

  bool publish(const std::string& topic, sensor_msgs::ImagePtr& image);

  void setRosPub(const std::string& topic);  // , const bool ros_pub);
  // TODO(lucasw) need way to remove publisher or subscriber

  // TODO(lucasw) virtual void draw()

  void update(const ros::TimerEvent& event);

  void draw(ros::Time cur);

private:
  ros::NodeHandle nh_;

  bool initted_ = false;
  std::map<std::string, std::mutex> sub_mutexes_;
  void imageCallback(const sensor_msgs::Image::ConstPtr msg, const std::string topic);

  bool show_unused_ = false;

  std::map<std::string, sensor_msgs::ImagePtr> from_sub_;
  std::map<std::string, ros::Subscriber> subs_;

  std::mutex pub_mutex_;
  std::deque<std::pair<std::string, sensor_msgs::ImagePtr>> to_pub_;
  std::map<std::string, ros::Publisher> pubs_;
};
}  // namespace imgui_ros
#endif  // IMGUI_ROS_IMAGE_TRANSFER_H

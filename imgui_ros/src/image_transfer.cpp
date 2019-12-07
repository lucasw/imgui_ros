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

#include <imgui.h>
#include <imgui_ros/image_transfer.h>
#include <imgui_ros/imgui_impl_opengl3.h>
// #include <internal_pub_sub/internal_pub_sub.hpp>
#include <imgui_ros/window.h>
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <thread>

using namespace std::chrono_literals;

namespace imgui_ros
{
ImageTransfer::ImageTransfer()
{

}

ImageTransfer::~ImageTransfer()
{
  ROS_INFO_STREAM("shutting down image transfer");
}

void ImageTransfer::postInit()
{
  // TODO(lucasw) only reason to run this in timer update rather than lockstep in main update
  // would be to allow running it in separate thread.
  update_timer_ = nh_.createTimer(ros::Duration(0.033),
      &ImageTransfer::update, this);
  ROS_INFO_STREAM("started image transfer");
}

bool ImageTransfer::getSub(const std::string& topic, sensor_msgs::ImagePtr& image)
{
  std::lock_guard<std::mutex> lock(sub_mutexes_[topic]);
  if (subs_.count(topic) < 1) {
    // TODO(lucasw) is it better to create the publisher here
    // or inside the thread update is running in?
    //std::function<void(const sensor_msgs::ImagePtr)> fnc;
    // fnc = std::bind(&ImageTransfer::imageCallback, this, std::placeholders::_1,
    //        topic);
    // subs_[topic] = create_subscription<sensor_msgs::Image>(topic, fnc);
    // TODO(lucasw) don't use remapping with imgui_ros currently, it will break
    // subs_[topic] = create_internal_subscription(topic, fnc);
    // subs_[topic] = nullptr;

    subs_[topic] = nh_.subscribe<sensor_msgs::Image>(topic, 4,
        boost::bind(&ImageTransfer::imageCallback, this, _1, topic));
  }
  // TODO(lucasw) if the sub doesn't exist at all need to create it
  if (from_sub_.count(topic) < 1) {
    return false;
  }
  image = from_sub_[topic];
  // this will remove the image from the queue, so can't have
  // multiple subscriber on same message- they need to share downstream from here
  // So instead keep all the most recent messages on every topic
  // from_sub_.erase(topic);
  return true;
}

bool ImageTransfer::publish(const std::string& topic, sensor_msgs::ImagePtr& image)
{
  std::lock_guard<std::mutex> lock(pub_mutex_);
  // TODO(lucasw) another image copy, need to go back to pointers once this is working
  to_pub_.push_back(std::pair<std::string, sensor_msgs::Image>(topic, image));
  return true;
}

void ImageTransfer::setRosPub(const std::string& topic)  // , const bool ros_pub)
{
  // pubs_[topic] = create_internal_publisher(topic);
  pubs_[topic] = nh_.advertise<sensor_msgs::Image>(topic, 5);
;
  // pubs_[topic]->ros_enable_ = ros_pub;
}

void ImageTransfer::update(const ros::TimerEvent& e)
{
  (void)e;
  // std::lock_guard<std::mutex> lock(sub_mutex_);
  if (!initted_) {
    std::cout << "image transfer 0x" << std::hex << std::this_thread::get_id()
        << std::dec << "\n";
    initted_ = true;
  }
  {
    while (to_pub_.size() > 0) {
      std::string topic;
      sensor_msgs::ImagePtr image;
      {
        std::lock_guard<std::mutex> lock(pub_mutex_);
        topic = to_pub_.front().first;
        image = to_pub_.front().second;
        to_pub_.pop_front();
      }
      if (pubs_.count(topic) > 0) {
        pubs_[topic].publish(image);
      }
      // TODO(lucasw) else debug error
    }
  }  // publish all queued up messages
}

void ImageTransfer::draw(ros::Time cur)
{
  (void)cur;
  // auto cur = now();
#if 0
  ImGui::Separator();
  ImGui::Text("enable sensor_msgs/Image publishing, otherwise in-process only");
  ImGui::Checkbox("show unused", &show_unused_);

  // TODO(lucasw) turn all the publishers on or off with a master checkbox
  // ImGui::Checkbox("multisample", &multisample_);
  ImGui::Columns(2);
  // TODO(lucasw) or just loop through local pubs_?  This is more
  // of an internal_pub_sub widget draw.
  for (auto topic_pair : core_->topics_) {
    auto topic = topic_pair.second;
    if (!topic) {
      continue;
    }
    ImGui::Text("%s", topic->full_topic_.c_str());
    ImGui::NextColumn();
    ImGui::Text("%lu pubs, %lu subs", topic->pubs_.size(), topic->subs_.size());
    ImGui::NextColumn();
    for (auto weak_pub : topic->pubs_) {
      auto pub = weak_pub.lock();
      if (!pub) {
        continue;
      }

      float rate = 0.0;
      if (pub->stamps_.size() > 2) {
        ros::Time earliest = pub->stamps_.front();
        rate = static_cast<float>(pub->stamps_.size()) /
          ((cur - earliest).toSec());
      }
      if (!show_unused_ && (topic->subs_.size() == 0) && (rate < 0.05)) {
        continue;
      }

      ImGui::Checkbox(("enable ##" + topic->full_topic_).c_str(), &pub->enable_);
      ImGui::NextColumn();
      ImGui::Checkbox(("ros2 dds ##" + topic->full_topic_).c_str(), &pub->ros_enable_);
      ImGui::NextColumn();
      ImGui::Text("%0.2f Hz", rate);
      ImGui::NextColumn();
      ImGui::Text("%0.5f pub duration", pub->publish_duration_.toSec());
      ImGui::NextColumn();
      float time_since_last = 0.0;
      if (pub->stamps_.size() > 0) {
        ros::Time latest = pub->stamps_.back();
        time_since_last =  (cur - latest).toSec();
      }
      ImGui::Text("%0.2f since last", time_since_last);
      ImGui::NextColumn();
      ImGui::NextColumn();
    }
  }
  ImGui::Columns(1);
#endif
}

void ImageTransfer::imageCallback(const sensor_msgs::ImagePtr& msg, const std::string& topic)
{
  // std::cout << "image transfer " << topic << " msg received " << msg->header.stamp.sec << "\n";
  std::lock_guard<std::mutex> lock(sub_mutexes_[topic]);
  from_sub_[topic] = msg;
}
}  // namespace imgui_ros

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
#include <internal_pub_sub/internal_pub_sub.hpp>
#include <imgui_ros/window.h>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

ImageTransfer::ImageTransfer()
{

}

ImageTransfer::~ImageTransfer()
{
  RCLCPP_INFO(get_logger(), "shutting down image transfer");
}

void ImageTransfer::postInit(std::shared_ptr<internal_pub_sub::Core> core)
{
  internal_pub_sub::Node::postInit(core);
  update_timer_ = this->create_wall_timer(33ms,
      std::bind(&ImageTransfer::update, this));
  RCLCPP_INFO(get_logger(), "started image transfer");
}

bool ImageTransfer::getSub(const std::string& topic, sensor_msgs::msg::Image::SharedPtr& image)
{
  std::lock_guard<std::mutex> lock(sub_mutexes_[topic]);
  if (subs_.count(topic) < 1) {
    // TODO(lucasw) is it better to create the publisher here
    // or inside the thread update is running in?
    std::function<void(std::shared_ptr<sensor_msgs::msg::Image>)> fnc;
    fnc = std::bind(&ImageTransfer::imageCallback, this, std::placeholders::_1,
            topic);
    // subs_[topic] = create_subscription<sensor_msgs::msg::Image>(topic, fnc);
    // TODO(lucasw) don't use remapping with imgui_ros currently, it will break
    subs_[topic] = create_internal_subscription(topic, fnc);
    // subs_[topic] = nullptr;
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

bool ImageTransfer::publish(const std::string& topic, sensor_msgs::msg::Image::SharedPtr image)
{
  std::lock_guard<std::mutex> lock(pub_mutex_);
  to_pub_.push_back(std::pair<std::string, sensor_msgs::msg::Image::SharedPtr>(topic, image));
  return true;
}

void ImageTransfer::setRosPub(const std::string& topic, const bool ros_pub)
{
  auto pub = get_create_internal_publisher(topic);
  if (pub) {
    pub->ros_enable_ = ros_pub;
  }
}

void ImageTransfer::update()
{
  // std::lock_guard<std::mutex> lock(sub_mutex_);
  if (!initted_) {
    std::cout << "image transfer 0x" << std::hex << std::this_thread::get_id()
        << std::dec << "\n";
    initted_ = true;
  }
  {
    while (to_pub_.size() > 0) {
      std::string topic;
      sensor_msgs::msg::Image::SharedPtr image;
      {
        std::lock_guard<std::mutex> lock(pub_mutex_);
        topic = to_pub_.front().first;
        image = to_pub_.front().second;
        to_pub_.pop_front();
      }
      auto pub = get_create_internal_publisher(topic);
      if (pub) {
        pub->publish(image);
      }
    }
  }  // publish all queued up messages
}

void ImageTransfer::draw(rclcpp::Time cur)
{
  // auto cur = now();

  ImGui::Separator();
  ImGui::Text("enable sensor_msgs/Image publishing, otherwise in-process only");
  ImGui::Checkbox("show unused", &show_unused_);

  // TODO(lucasw) turn all the publishers on or off with a master checkbox
  // ImGui::Checkbox("multisample", &multisample_);
  ImGui::Columns(2);
  for (auto pub_pair : core_->publishers_) {
    auto pub = pub_pair.second;
    if (pub) {
      float rate = 0.0;
      if (pub->stamps_.size() > 2) {
        rclcpp::Time earliest = pub->stamps_.front();
        rate = static_cast<float>(pub->stamps_.size()) /
          ((cur - earliest).nanoseconds() / 1e9);
      }
      if (!show_unused_ && (pub->subs_.size() == 0) && (rate < 0.05)) {
        continue;
      }

      ImGui::Checkbox(pub->topic_.c_str(), &pub->enable_);
      ImGui::NextColumn();
      ImGui::Checkbox(("ros2 dds ##" + pub->topic_).c_str(), &pub->ros_enable_);
      ImGui::NextColumn();
      ImGui::Text("%lu subs", pub->subs_.size());
      ImGui::NextColumn();
      ImGui::Text("%0.2f Hz", rate);
      ImGui::NextColumn();
      ImGui::Text("%0.5f pub duration", pub->publish_duration_.nanoseconds() / 1e9);
      ImGui::NextColumn();
      float time_since_last = 0.0;
      if (pub->stamps_.size() > 0) {
        rclcpp::Time latest = pub->stamps_.back();
        time_since_last =  (cur - latest).nanoseconds() / 1e9;
      }
      ImGui::Text("%0.2f since last", time_since_last);
      ImGui::NextColumn();
    }
  }
  ImGui::Columns(1);
}

void ImageTransfer::imageCallback(sensor_msgs::msg::Image::SharedPtr msg, const std::string& topic)
{
  // std::cout << "image transfer " << topic << " msg received " << msg->header.stamp.sec << "\n";
  std::lock_guard<std::mutex> lock(sub_mutexes_[topic]);
  from_sub_[topic] = msg;
}

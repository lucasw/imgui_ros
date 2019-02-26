/*
 * Copyright (c) 2019 Lucas Walter
 * January 2019
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

#include <deque>
#include <functional>
#include <internal_pub_sub/publisher.hpp>
#include <internal_pub_sub/node.hpp>
#include <internal_pub_sub/topic.hpp>
#include <list>
#include <rclcpp/rclcpp.hpp>

namespace internal_pub_sub
{

Publisher::Publisher(
    const std::shared_ptr<Topic> topic,
    std::shared_ptr<Node> node) :
    topic_(topic),
    full_topic_(topic->full_topic_)
{
  if (node) {
    RCLCPP_INFO(node->get_logger(),
        "creating new publisher with ros pub option '%s'",
        topic->full_topic_.c_str());
    // if (topic != "") {
    ros_pub_ = node->create_publisher<sensor_msgs::msg::Image>(topic->full_topic_);
    // }  else {
      // This is the case where the publisher is node doesn't exist yet, and the subscriber is causing it to come into existence
      // TODO(lucasw) though that doesn't really make sense
      // ros_pub_ = node->create_publisher<sensor_msgs::msg::Image>(remapped_topic);
    // }
  } else {
    std::cout << this << " creating new publisher without ros pub for now '"
        << topic->full_topic_ << "')\n";
    ros_enable_ = false;
  }
  clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
}

Publisher::~Publisher()
{
  std::cout << this << " shutting down publisher '" << full_topic_ << "'\n";
  // TODO(lucasw) delete out of core_
}

void Publisher::publish(sensor_msgs::msg::Image::SharedPtr msg)
{
  auto t0 = clock_->now();
  // TODO(lucasw) make this optional
  rclcpp::Time cur = msg->header.stamp;
  stamps_.push_back(cur);
  if (stamps_.size() > 50) {
    stamps_.pop_front();
  } else if ((stamps_.size() > 10) &&
      ((cur - stamps_.front()).nanoseconds() > 2e9)) {
    stamps_.pop_front();
  }

  if (!enable_) {
    return;
  }

  if (ros_enable_ && ros_pub_) {
    ros_pub_->publish(msg);
    return;
  }

  // TODO(lucasw) in this system the callbacks are called in the thread of the publisher
  // and block it until they finish- maybe should make a thread here?
  // Otherwise best practice is that subscribers should do very little in callbacks.
  std::lock_guard<std::mutex> lock(sub_mutex_);

  if (auto topic = topic_.lock()) {
    // std::cout << topic_ << " publishing to " << sub->topic_ << "\n";
    topic->publish(msg);
  } else {
    // TODO(lucasw) should be impossible to get here after remove_if above
    // std::cerr << topic_ << " bad sub lock\n";
  }

  publish_duration_ = clock_->now() - t0;
}

}  // internal_pub_sub

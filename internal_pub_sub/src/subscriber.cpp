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
#include <internal_pub_sub/node.hpp>
#include <internal_pub_sub/subscriber.hpp>
#include <list>
#include <rclcpp/rclcpp.hpp>


namespace internal_pub_sub
{

Subscriber::Subscriber(const std::string& topic, const std::string& remapped_topic,
      Function callback,
      std::shared_ptr<Node> node) :
      topic_(topic),
      remapped_topic_(remapped_topic)
{
  setFullTopic(node, topic_);
  setFullTopic(node, remapped_topic_);

  bind(callback);
  if (node) {
    std::cout << this << " creating new subscriber with ros sub '" << topic_ << "'\n";
    ros_sub_ = node->create_subscription<sensor_msgs::msg::Image>(topic_, callback_);
    // TODO(lucasw) test remapped_topic_ == ros_sub_->get_topic_name() ?
    std::cout << topic_ << " remapped to " << remapped_topic_ << "\n";
  } else {
    std::cout << this << " creating new subscriber without ros sub '" << topic_ << "'\n";
  }
}

Subscriber::~Subscriber()
{
  std::cout << this << " shutting down subscriber '" << topic_ << "'\n";
}

void Subscriber::bind(Function fn)
{
  callback_ = fn;
}

void Subscriber::callback(sensor_msgs::msg::Image::SharedPtr msg)
{
  if (callback_) {
    callback_(msg);
  }
}

}  // internal_pub_sub

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
#include <internal_pub_sub/topic.hpp>
#include <internal_pub_sub/subscriber.hpp>
#include <list>
#include <rclcpp/rclcpp.hpp>

namespace internal_pub_sub
{

Topic::Topic(
    const std::string& full_topic,
    std::shared_ptr<Node> node) :
    full_topic_(full_topic)
{
  (void)node;
  std::cout << "creating new topic " << full_topic_ << "\n";
}

Topic::~Topic()
{
  std::cout << this << " shutting down topic '" << full_topic_ << "'\n";
  // TODO(lucasw) delete out of core_
}

void Topic::publish(sensor_msgs::msg::Image::SharedPtr msg)
{
  // TODO(lucasw) in this system the callbacks are called in the thread of the publisher
  // and block it until they finish- maybe should make a thread here?
  // Otherwise best practice is that subscribers should do very little in callbacks.
  std::lock_guard<std::mutex> lock(sub_mutex_);

  for (auto sub_weak : subs_) {
    if (auto sub = sub_weak.lock()) {
      // std::cout << topic_ << " publishing to " << sub->topic_ << "\n";
      sub->callback(msg);
    } else {
      // TODO(lucasw) should be impossible to get here after remove_if above
      // std::cerr << topic_ << " bad sub lock\n";
    }
  }
}

void Topic::clean()
{
  {
  std::lock_guard<std::mutex> lock(sub_mutex_);
  // remove dead subscribers
  subs_.remove_if([](std::weak_ptr<Subscriber> p) {
      if (auto sp = p.lock()) {
        return false;
      }
      std::cout << "removing dead sub\n";
      return true;
      });
  }

  pubs_.remove_if([](std::weak_ptr<Publisher> p) {
      if (auto sp = p.lock()) {
        return false;
      }
      std::cout << "removing dead sub\n";
      return true;
      });
}

}  // internal_pub_sub

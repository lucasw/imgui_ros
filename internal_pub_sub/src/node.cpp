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

#include <internal_pub_sub/node.hpp>
#include <internal_pub_sub/core.hpp>
#include <internal_pub_sub/publisher.hpp>
#include <internal_pub_sub/subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

namespace internal_pub_sub
{

Node::Node()
{
}

// TODO(lucasw) how to make this automatic
// all the inheriting nodes need to call this
void Node::postInit(std::shared_ptr<Core> core)
{
  core_ = core;
  if (core_ == nullptr) {
    RCLCPP_INFO(get_logger(), "creating new Core for this node");
    core_ = std::make_shared<internal_pub_sub::Core>();
  }
}

std::string Node::getRemappedTopic(const std::string& topic)
{
#if 0
  for (auto pair : remappings_) {
    std::cout << "'" << pair.first << "' -> '" << pair.second << "', '" << topic << "'\n";
  }
#endif
  std::string remapped_topic = remappings_[topic];
  if (remapped_topic == "") {
    RCLCPP_WARN(get_logger(), "unexpected unremapped topic '%s', %d",
        topic.c_str(), remappings_.size());
    remapped_topic = topic;
    remappings_[topic] = remapped_topic;
  }
  return remapped_topic;
}

// TODO(lucasw) make a get_create_publisher and get_subscription convenience function here
std::shared_ptr<Publisher> Node::create_internal_publisher(const std::string& topic)
{
  auto node = std::static_pointer_cast<internal_pub_sub::Node>(shared_from_this());
  return core_->create_publisher(topic, getRemappedTopic(topic), node);
}

std::shared_ptr<Subscriber> Node::create_internal_subscription(const std::string& topic,
    Function callback)
{
  auto node = std::static_pointer_cast<internal_pub_sub::Node>(shared_from_this());
  return core_->create_subscription(topic, getRemappedTopic(topic), callback, node);
}

}  // internal_pub_sub

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
#include <internal_pub_sub/internal_pub_sub.hpp>
#include <list>
#include <rclcpp/rclcpp.hpp>

namespace internal_pub_sub
{

// TODO(lucasw) need to make this take a vector of pairs of inputs and outputs,
// and synchronize them so all the messages on all the topics will get republished together
// with the same timestamp.
Republisher::Republisher(
    const std::vector<std::string>& inputs,
    const std::vector<std::string>& outputs,
    const size_t skip,
    std::shared_ptr<Core> core,
    std::shared_ptr<Node> node) :
    input_topics_(inputs),
    // outputs_(outputs),
    skip_max_(skip)
{
  if (!core) {
    return;
  }
  if (inputs.size() != outputs.size()) {
    std::cerr << "republisher size mismatch " << inputs.size() << " != "
        << outputs.size() << "\n";
    return;
  }
  std::cout << "new republisher " << this << " " << input_topics_.size() << "\n";
  for (size_t i = 0; i < inputs.size(); ++i) {
    // TODO(lucasw) ignoring remapping for now
    subs_.push_back(core->create_subscription(inputs[i], inputs[i],
        std::bind(&Republisher::callback, this, std::placeholders::_1, inputs[i]), node));
    pubs_[inputs[i]] = core->create_publisher(outputs[i], outputs[i], node);
  }
}

Republisher::Republisher(const std::string& input, const std::string& output,
    const size_t skip,
    std::shared_ptr<Core> core,
    std::shared_ptr<Node> node) :
    skip_max_(skip)
{
  if (!core) {
    return;
  }
  input_topics_.push_back(input);
  // TODO(lucasw) if the node has a remapping this will break
  subs_.push_back(core->create_subscription(input, input,
      std::bind(&Republisher::callback, this, std::placeholders::_1, input), node));
  pubs_[input] = core->create_publisher(output, output, node);
}

Republisher::~Republisher()
{
  // TODO(lucasw) what is result if Republisher goes out of scope/gets deleted?
  // The publishers on the input topics detect that the subscribers here
  // are dead and removes them.
  std::cout << "Republisher " << this << " shutting down "
      << input_topics_.size() << " " << subs_.size() << " " << pubs_.size() << "  ";
  for (auto& topic : input_topics_) {
    std::cout << topic << " ";
  }
  std::cout << "\n";
}

bool Republisher::allMessagesReceived(rclcpp::Time stamp)
{
  if (messages_.count(stamp) < 1) {
    return false;
  }

  return (messages_[stamp].size() == input_topics_.size());
}

void Republisher::callback(sensor_msgs::msg::Image::SharedPtr msg, const std::string& topic)
{
  auto stamp = msg->header.stamp;
  messages_[stamp][topic] = msg;
  // std::cout << "callback " << topic << " " << stamp.sec << " "
  //     << messages_[stamp].size() << " " << input_topics_.size() << "\n";
  if (!allMessagesReceived(stamp)) {
    // TODO(lucasw) need to do cleanup on messages_ to get rid of old messages
    return;
  }
  ++skip_count_;
  if (skip_count_ > skip_max_) {
    skip_count_ = 0;
  }
  if (skip_count_ == 0) {
    // TODO(lucasw) maybe should queue this up instead of republishing here (in the same
    // thread as the publishing caller)
    for (auto pair : messages_[stamp]) {
      pubs_[pair.first]->publish(pair.second);
    }
  }
  messages_.erase(stamp);
}

}  // internal_pub_sub

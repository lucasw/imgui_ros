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
#include <internal_pub_sub/topic.hpp>
#include <list>
#include <rclcpp/rclcpp.hpp>

// inline
void setFullTopic(std::shared_ptr<rclcpp::Node> node, std::string& topic)
{
  if (!node)
    return;

  // don't change the topic if it is already on the root
  if ((topic.size() > 0) && (topic[0] != '/')) {
    std::string ns = node->get_namespace();
    // if the ns is on the root the namespace is '/', but if it isn't
    // the namespace doesn't have a trailing /.
    if ((ns.size() > 0) && (ns[ns.size() - 1] != '/')) {
      ns += "/";
    }
    topic = ns + topic;
  }
}

namespace internal_pub_sub
{

Core::Core(const bool ros_enable_default) : ros_enable_default_(ros_enable_default)
{
  std::cout << "0x" << std::hex << std::this_thread::get_id() << std::dec
      << " new internal pub sub core" << std::endl;
}

Core::~Core()
{
  std::cout << "0x" << std::hex << std::this_thread::get_id() << std::dec
      << " shutting down internal pub sub core" << std::endl;
}

std::shared_ptr<Subscriber> Core::create_subscription(
    std::string topic,
    std::string remapped_topic,
    Function callback,
    std::shared_ptr<Node> node)
{
  setFullTopic(node, topic);
  setFullTopic(node, remapped_topic);

  // TODO(lucasw) look through topics and see if callback is already there?
  // otherwise the same callback will get called as many times as this has been
  // called with it.
  auto sub = std::make_shared<Subscriber>(topic, remapped_topic, callback, node);

  if ((topics_.count(remapped_topic) < 1)) {
    topics_[remapped_topic] = std::make_shared<Topic>(remapped_topic, node);
  }

  std::cout << "CORE creating new subscriber on topic :'" << remapped_topic << "' (remapped from '" << topic << "')\n";
  std::lock_guard<std::mutex> lock(topics_[remapped_topic]->sub_mutex_);
  topics_[remapped_topic]->subs_.push_back(sub);

  return sub;
}

std::shared_ptr<Publisher> Core::create_publisher(
    std::string topic,
    std::string remapped_topic,
    std::shared_ptr<Node> node)
{
  std::shared_ptr<Publisher> pub;

  setFullTopic(node, topic);
  setFullTopic(node, remapped_topic);

  if ((topics_.count(remapped_topic) < 1)) {
    topics_[remapped_topic] = std::make_shared<Topic>(remapped_topic, node);
  }

  //  || (!(pub = publishers_[topic].lock()))) {
  std::cout << "CORE creating new publisher on topic :'" << remapped_topic
      << "' (remapped from '" << topic << "'), ros_enable "
      << ros_enable_default_ << "\n";
  pub = std::make_shared<Publisher>(topics_[remapped_topic], node);
  pub->ros_enable_ = ros_enable_default_;
  topics_[remapped_topic]->pubs_.push_back(pub);

  return pub;
}

// it's up to the caller to hold on to the shared ptr then discard it when finish,
// currently nothing is done with the weak_ptr hear but maybe in the future,
// and whatever is done will scan for dead pointers and erase them.
std::shared_ptr<Republisher> Core::create_republisher(
    const std::vector<std::string>& input_topics,
    const std::vector<std::string>& output_topics,
    std::shared_ptr<Node> node)
{
  auto repub = std::make_shared<Republisher>(
      input_topics, output_topics, 4, shared_from_this(), node);
  std::weak_ptr<Republisher> weak_repub = repub;
  republishers_.push_back(repub);
  return repub;
}

// TODO(lucasw) maybe all future publishers also need to be able to be enabled also?
void Core::rosEnableAllPublishers(const bool enable)
{
  for (auto topic_pair : topics_) {
    auto topic = topic_pair.second;
    if (!topic) {
      continue;
    }
    for (auto weak_pub : topic->pubs_) {
      auto pub = weak_pub.lock();
      if (!pub) {
        continue;
      }
      pub->ros_enable_ = enable;
    }
  }
}

void Core::clean()
{
  // std::cout << "CORE clean " << publishers_.size() << "\n";
  // TODO(lucasw)
  // need to use reference otherwise use_count below will be increased
  // by additional shared_ptrs, though this may cause problems with
  // multi threading?
  // TODO(lucasw) lock a mutex?
  for (auto& topic_pair : topics_) {
    auto& topic = topic_pair.second;
    if (!topic) {
      topics_.erase(topic_pair.first);
      continue;
    }
    topic->clean();
    // if nothing else has a shared_ptr to this pub, and it has no subscribers,
    // get rid of it
    // std::cout << "  pub " << pub->topic_ << " has subs " << pub->subs_.size() << ", use_count "
    //    << pub.use_count() << "\n";
    if (topic->subs_.size() < 1) {
      if (topic.use_count() == 1) {
        topics_.erase(topic_pair.first);
      }
    }
  }
}

}  // internal_pub_sub

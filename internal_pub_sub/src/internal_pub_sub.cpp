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

Publisher::Publisher(
    const std::string& topic,
    const std::string& remapped_topic,
    std::shared_ptr<Node> node) :
    topic_(topic),
    remapped_topic_(remapped_topic)
{
  setFullTopic(node, topic_);
  setFullTopic(node, remapped_topic_);
  if (node) {
    RCLCPP_INFO(node->get_logger(),
        "creating new publisher with ros pub option '%s' (remapped from '%s')",
        remapped_topic_.c_str(), topic_.c_str());
    // if (topic != "") {
    ros_pub_ = node->create_publisher<sensor_msgs::msg::Image>(topic);
    // }  else {
      // This is the case where the publisher is node doesn't exist yet, and the subscriber is causing it to come into existence
      // TODO(lucasw) though that doesn't really make sense
      // ros_pub_ = node->create_publisher<sensor_msgs::msg::Image>(remapped_topic);
    // }
  } else {
    std::cout << this << " creating new publisher without ros pub for now '" << remapped_topic_
        << "' (remapped from '" << topic_ << "')\n";
    ros_enable_ = false;
  }
  clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
}

Publisher::~Publisher()
{
  std::cout << this << " shutting down publisher '" << topic_ << "'\n";
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

  for (auto sub_weak : subs_) {
    if (auto sub = sub_weak.lock()) {
      // std::cout << topic_ << " publishing to " << sub->topic_ << "\n";
      sub->callback(msg);
    } else {
      // TODO(lucasw) should be impossible to get here after remove_if above
      // std::cerr << topic_ << " bad sub lock\n";
    }
  }

  publish_duration_ = clock_->now() - t0;
}

void Publisher::clean()
{
  // remove dead subscribers
  subs_.remove_if([](std::weak_ptr<Subscriber> p) {
      if (auto sp = p.lock()) {
        return false;
      }
      std::cout << "removing dead sub\n";
      return true;
      });
}


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
    const std::string& topic,
    const std::string& remapped_topic,
    Function callback,
    std::shared_ptr<Node> node)
{
  // TODO(lucasw) look through topics and see if callback is already there?
  // otherwise the same callback will get called as many times as this has been
  // called with it.
  auto sub = std::make_shared<Subscriber>(topic, remapped_topic, callback, node);

  auto pub = get_create_publisher(topic, remapped_topic, node, false);
  if (pub) {
    std::cout << "CORE creating new subscriber on topic :'" << remapped_topic << "' (remapped from '" << topic << "')\n";
    std::lock_guard<std::mutex> lock(pub->sub_mutex_);
    pub->subs_.push_back(sub);
  } else {
    std::cerr << "couldn't create new subscriber on topic :'" << remapped_topic << "' (remapped from '" << topic << "')\n";
  }

  return sub;
}

std::shared_ptr<Publisher> Core::get_create_publisher(
    std::string topic,
    std::string remapped_topic,
    std::shared_ptr<Node> node,
    const bool create_ros_pub)
{
  std::shared_ptr<Publisher> pub;

  setFullTopic(node, topic);
  setFullTopic(node, remapped_topic);

  if ((publishers_.count(remapped_topic) < 1)) { //  || (!(pub = publishers_[topic].lock()))) {
    std::cout << "CORE creating new publisher on topic :'" << remapped_topic << "' (remapped from '" << topic << "'), ros_enable "
        << ros_enable_default_ << "\n";
    pub = std::make_shared<Publisher>(topic, remapped_topic, create_ros_pub ? node : nullptr);
    pub->ros_enable_ = ros_enable_default_;
    publishers_[remapped_topic] = pub;
  } else {
    pub = publishers_[remapped_topic];
    // if a subscription created a partial publisher earlier, fill in the ros_pub_ now with the proper node
    if (create_ros_pub && pub->ros_pub_ == nullptr) {
      pub->ros_pub_ = node->create_publisher<sensor_msgs::msg::Image>(topic);
    }
  }

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
  for (auto pub_pair : publishers_) {
    auto pub = pub_pair.second;
    if (pub) {
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
  for (auto& pub_pair : publishers_) {
    auto& pub = pub_pair.second;
    if (!pub) {
      publishers_.erase(pub_pair.first);
      continue;
    }
    pub->clean();
    // if nothing else has a shared_ptr to this pub, and it has no subscribers,
    // get rid of it
    // std::cout << "  pub " << pub->topic_ << " has subs " << pub->subs_.size() << ", use_count "
    //    << pub.use_count() << "\n";
    if (pub->subs_.size() < 1) {
      if (pub.use_count() == 1) {
        publishers_.erase(pub_pair.first);
      }
    }
  }
}

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
    pubs_[inputs[i]] = core->get_create_publisher(outputs[i], outputs[i], node);
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
  pubs_[input] = core->get_create_publisher(output, output, node);
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

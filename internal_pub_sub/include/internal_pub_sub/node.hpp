#ifndef INTERNAL_PUB_SUB_NODE_HPP
#define INTERNAL_PUB_SUB_NODE_HPP

#include <deque>
#include <functional>
#include <internal_pub_sub/core.hpp>
#include <internal_pub_sub/utility.hpp>
#include <list>
#include <rclcpp/rclcpp.hpp>

namespace internal_pub_sub
{

class Node : public rclcpp::Node
// , std::enable_shared_from_this<Node>
// have to std::static_pointer_cast<internal_pub_sub::Node>(shared_from_this())
{
public:
  Node()  // : rclcpp::Node()
  {
  }

  // TODO(lucasw) how to make this automatic
  // all the inheriting nodes need to call this
  virtual void postInit(std::shared_ptr<Core> core)
  {
    core_ = core;
    if (core_ == nullptr) {
      RCLCPP_INFO(get_logger(), "creating new Core for this node");
      core_ = std::make_shared<internal_pub_sub::Core>();
    }
  }

  std::string getRemappedTopic(const std::string& topic)
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
  std::shared_ptr<Publisher> get_create_internal_publisher(const std::string& topic)
  {
      auto node = std::static_pointer_cast<internal_pub_sub::Node>(shared_from_this());
    return core_->get_create_publisher(topic, getRemappedTopic(topic), node);
  }

  std::shared_ptr<Subscriber> create_internal_subscription(const std::string& topic,
      Function callback)
  {
    auto node = std::static_pointer_cast<internal_pub_sub::Node>(shared_from_this());
    return core_->create_subscription(topic, getRemappedTopic(topic), callback, node);
  }

  std::map<std::string, std::string> remappings_;
protected:
  std::shared_ptr<Core> core_;
};

}  // internal_pub_sub
#endif  // INTERNAL_PUB_SUB_NODE_HPP

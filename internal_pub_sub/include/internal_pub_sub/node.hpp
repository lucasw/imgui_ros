#ifndef INTERNAL_PUB_SUB_NODE_HPP
#define INTERNAL_PUB_SUB_NODE_HPP

#include <internal_pub_sub/utility.hpp>
#include <rclcpp/rclcpp.hpp>

namespace internal_pub_sub
{

class Core;
class Publisher;
class Subscriber;

class Node : public rclcpp::Node
// , std::enable_shared_from_this<Node>
// have to std::static_pointer_cast<internal_pub_sub::Node>(shared_from_this())
{
public:
  Node();

  // TODO(lucasw) how to make this automatic
  // all the inheriting nodes need to call this
  virtual void postInit(std::shared_ptr<Core> core);

  std::string getRemappedTopic(const std::string& topic);

  std::shared_ptr<Publisher> create_internal_publisher(const std::string& topic);
  std::shared_ptr<Subscriber> create_internal_subscription(const std::string& topic,
      Function callback);

  std::map<std::string, std::string> remappings_;
protected:
  std::shared_ptr<Core> core_;
};

}  // internal_pub_sub
#endif  // INTERNAL_PUB_SUB_NODE_HPP

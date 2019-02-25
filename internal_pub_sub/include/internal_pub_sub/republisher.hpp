#ifndef INTERNAL_PUB_SUB_REPUBLISHER_HPP
#define INTERNAL_PUB_SUB_REPUBLISHER_HPP

#include <deque>
#include <functional>
#include <internal_pub_sub/core.hpp>
#include <list>
#include <rclcpp/rclcpp.hpp>

// Only support sensor_msgs::msg::Image initially, later make it generic
#include <sensor_msgs/msg/image.hpp>

namespace internal_pub_sub
{

class Core;
class Node;

struct Republisher
{
  // TODO(lucasw) need to make this take a vector of pairs of inputs and outputs,
  // and synchronize them so all the messages on all the topics will get republished together
  // with the same timestamp.
  Republisher(
      const std::vector<std::string>& inputs,
      const std::vector<std::string>& outputs,
      const size_t skip,
      std::shared_ptr<Core> core,
      std::shared_ptr<Node> node = nullptr);

  Republisher(const std::string& input, const std::string& output,
      const size_t skip,
      std::shared_ptr<Core> core,
      std::shared_ptr<Node> node = nullptr);

  ~Republisher();

  bool allMessagesReceived(rclcpp::Time stamp);

  void callback(sensor_msgs::msg::Image::SharedPtr msg, const std::string& topic);

  std::vector<std::string> input_topics_;

  std::vector<std::shared_ptr<Subscriber> > subs_;
  std::map<std::string, std::shared_ptr<Publisher> > pubs_;

  std::map<rclcpp::Time,
      std::map<std::string, sensor_msgs::msg::Image::SharedPtr> > messages_;
  size_t skip_count_ = 0;
  size_t skip_max_ = 0;
};  // Republisher

}  // internal_pub_sub
#endif  // INTERNAL_PUB_SUB_REPUBLISHER_HPP

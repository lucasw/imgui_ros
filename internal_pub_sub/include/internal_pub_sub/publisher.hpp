#ifndef INTERNAL_PUB_SUB_PUBLISHER_HPP
#define INTERNAL_PUB_SUB_PUBLISHER_HPP

#include <deque>
#include <functional>
#include <list>
#include <rclcpp/rclcpp.hpp>

// Only support sensor_msgs::msg::Image initially, later make it generic
#include <sensor_msgs/msg/image.hpp>

namespace internal_pub_sub
{

class Subscriber;
class Node;
class Topic;

struct Publisher  // : std::enable_shared_from_this<Publisher>
{
  Publisher(
      std::shared_ptr<Topic> topic,
      std::shared_ptr<Node> node=nullptr);
  ~Publisher();

  rclcpp::Duration publish_duration_ = rclcpp::Duration(0, 0);
  void publish(sensor_msgs::msg::Image::SharedPtr msg);
  void clean();

  std::weak_ptr<Topic> topic_;
  std::string full_topic_;

  bool enable_ = true;
  bool ros_enable_ = false;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ros_pub_;
  std::mutex sub_mutex_;

  std::deque<rclcpp::Time> stamps_;
  rclcpp::Clock::SharedPtr clock_;
};

}  // internal_pub_sub
#endif  // INTERNAL_PUB_SUB_PUBLISHER_HPP

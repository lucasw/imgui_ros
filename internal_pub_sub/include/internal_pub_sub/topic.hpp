#ifndef INTERNAL_PUB_SUB_TOPIC_HPP
#define INTERNAL_PUB_SUB_TOPIC_HPP

#include <deque>
#include <functional>
#include <list>
#include <rclcpp/rclcpp.hpp>

// Only support sensor_msgs::msg::Image initially, later make it generic
#include <sensor_msgs/msg/image.hpp>

namespace internal_pub_sub
{

class Publisher;
class Subscriber;
class Node;

struct Topic  // : std::enable_shared_from_this<Topic>
{
  Topic(
      const std::string& full_topic,
      std::shared_ptr<Node> node=nullptr);
  ~Topic();

  // these should be the full path topics
  std::string full_topic_;

  void publish(sensor_msgs::msg::Image::SharedPtr msg);
  void clean();

  std::list<std::weak_ptr<Publisher> > pubs_;
  std::list<std::weak_ptr<Subscriber> > subs_;

  std::mutex sub_mutex_;
};

}  // internal_pub_sub
#endif  // INTERNAL_PUB_SUB_TOPIC_HPP

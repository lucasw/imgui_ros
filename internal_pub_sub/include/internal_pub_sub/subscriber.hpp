#ifndef INTERNAL_PUB_SUB_SUBSCRIBER_HPP
#define INTERNAL_PUB_SUB_SUBSCRIBER_HPP

#include <functional>
#include <internal_pub_sub/utility.hpp>
#include <list>
#include <rclcpp/rclcpp.hpp>

// Only support sensor_msgs::msg::Image initially, later make it generic
#include <sensor_msgs/msg/image.hpp>


namespace internal_pub_sub
{

class Node;

struct Subscriber
{
public:
  Subscriber(const std::string& topic, const std::string& remapped_topic,
      Function callback,
      std::shared_ptr<Node> node=nullptr);
  ~Subscriber();

  void bind(Function fn);
  void callback(sensor_msgs::msg::Image::SharedPtr msg);

  // TODO(lucasw) weak_ptr to node, then have container delete this
  // when it goes dead?
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ros_sub_;
  std::string topic_;
  std::string remapped_topic_;
private:
  Function callback_;
};

}  // internal_pub_sub
#endif  // INTERNAL_PUB_SUB_SUBSCRIBER_HPP

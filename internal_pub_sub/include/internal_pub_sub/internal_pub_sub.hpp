#ifndef INTERNAL_PUB_SUB_INTERNAL_PUB_SUB_HPP
#define INTERNAL_PUB_SUB_INTERNAL_PUB_SUB_HPP

#include <deque>
#include <functional>
#include <list>
#include <rclcpp/rclcpp.hpp>

// Only support sensor_msgs::msg::Image initially, later make it generic
#include <sensor_msgs/msg/image.hpp>

typedef std::function<void (sensor_msgs::msg::Image::SharedPtr)> Function;

// inline
void setFullTopic(std::shared_ptr<rclcpp::Node> node, std::string& topic);

namespace internal_pub_sub
{

struct Subscriber
{
public:
  Subscriber(const std::string& topic,
      Function callback,
      std::shared_ptr<rclcpp::Node> node=nullptr);
  ~Subscriber();

  void bind(Function fn);
  void callback(sensor_msgs::msg::Image::SharedPtr msg);

  // TODO(lucasw) weak_ptr to node, then have container delete this
  // when it goes dead?
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ros_sub_;
  std::string topic_;
private:
  Function callback_;
};

struct Publisher  // : std::enable_shared_from_this<Publisher>
{
  Publisher(const std::string& topic,
      std::shared_ptr<rclcpp::Node> node=nullptr);
  ~Publisher();

  std::string topic_;

  rclcpp::Duration publish_duration_ = rclcpp::Duration(0, 0);
  void publish(sensor_msgs::msg::Image::SharedPtr msg);
  void clean();

  std::list<std::weak_ptr<Subscriber> > subs_;

  bool enable_ = true;
  bool ros_enable_ = false;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ros_pub_;
  std::mutex sub_mutex_;

  std::deque<rclcpp::Time> stamps_;

  rclcpp::Clock::SharedPtr clock_;
};

class Core;

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
      std::shared_ptr<rclcpp::Node> node = nullptr);

  Republisher(const std::string& input, const std::string& output,
      const size_t skip,
      std::shared_ptr<Core> core,
      std::shared_ptr<rclcpp::Node> node = nullptr);

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

class Core : std::enable_shared_from_this<Core>
{
public:
  Core(const bool ros_enable_default=false);
  ~Core();

  std::shared_ptr<Subscriber> create_subscription(const std::string& topic,
      Function callback,
      std::shared_ptr<rclcpp::Node> node=nullptr);
  std::shared_ptr<Publisher> get_create_publisher(std::string topic,
      std::shared_ptr<rclcpp::Node> node=nullptr);

  // it's up to the caller to hold on to the shared ptr then discard it when finish,
  // currently nothing is done with the weak_ptr hear but maybe in the future,
  // and whatever is done will scan for dead pointers and erase them.
  std::shared_ptr<Republisher> create_republisher(
      const std::vector<std::string>& input_topics,
      const std::vector<std::string>& output_topics,
      std::shared_ptr<rclcpp::Node> node=nullptr);

  bool ros_enable_default_ = false;
  // TODO(lucasw) maybe all future publishers also need to be able to be enabled also?
  void rosEnableAllPublishers(const bool enable);

  void clean();

  // TODO(lucasw)
  // These can't be weak_ptrs because a subscriber without a publisher may need it
  // to stick around, so for now can't delete publishers
  // A deletion function would have to check for zero subscribers.
  std::map<std::string, std::shared_ptr<Publisher> > publishers_;

  // TODO(lucasw) do republishers need to be kept here at all?
  // maybe for visualization can get a list of all republishers.
  std::list<std::weak_ptr<Republisher> > republishers_;
};  // Core

class Node : public rclcpp::Node
{
public:
  Node() : rclcpp::Node()
  {

  }

  void setCore(std::shared_ptr<Core> core)
  {
    core_ = core;
  }
protected:
  std::shared_ptr<Core> core_;
};

}  // internal_pub_sub
#endif  // INTERNAL_PUB_SUB_INTERNAL_PUB_SUB_HPP

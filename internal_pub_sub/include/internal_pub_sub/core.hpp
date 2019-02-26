#ifndef INTERNAL_PUB_SUB_CORE_HPP
#define INTERNAL_PUB_SUB_CORE_HPP

#include <deque>
#include <functional>
#include <internal_pub_sub/utility.hpp>
#include <list>
#include <rclcpp/rclcpp.hpp>

namespace internal_pub_sub
{
class Node;
class Subscriber;
class Publisher;
class Republisher;
class Topic;

class Core : std::enable_shared_from_this<Core>
{
public:
  Core(const bool ros_enable_default=false);
  ~Core();

  std::shared_ptr<Subscriber> create_subscription(
      const std::string& topic,
      const std::string& remapped_topic,
      Function callback,
      std::shared_ptr<Node> node=nullptr);
  std::shared_ptr<Publisher> create_publisher(
      std::string topic,
      std::string remapped_topic,
      std::shared_ptr<Node> node=nullptr);

  // TODO(lucasw) should the parent node remapping alter what the republisher does?
  // For now it does not.
  // it's up to the caller to hold on to the shared ptr then discard it when finish,
  // currently nothing is done with the weak_ptr hear but maybe in the future,
  // and whatever is done will scan for dead pointers and erase them.
  std::shared_ptr<Republisher> create_republisher(
      const std::vector<std::string>& input_topics,
      const std::vector<std::string>& output_topics,
      std::shared_ptr<Node> node=nullptr);

  bool ros_enable_default_ = false;
  // TODO(lucasw) maybe all future publishers also need to be able to be enabled also?
  void rosEnableAllPublishers(const bool enable);

  // TODO(lucasw) it's very easy to forget to have this called
  void clean();

  // TODO(lucasw)
  // These can't be weak_ptrs because a subscriber without a publisher may need it
  // to stick around, so for now can't delete publishers
  // A deletion function would have to check for zero subscribers.
  std::map<std::string, std::shared_ptr<Topic> > topics_;

  // TODO(lucasw) do republishers need to be kept here at all?
  // maybe for visualization can get a list of all republishers.
  std::list<std::weak_ptr<Republisher> > republishers_;
};  // Core

}  // internal_pub_sub
#endif  // INTERNAL_PUB_SUB_CORE_HPP

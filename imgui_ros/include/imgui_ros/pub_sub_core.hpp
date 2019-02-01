#ifndef IMGUI_ROS_PUB_SUB_CORE_HPP
#define IMGUI_ROS_PUB_SUB_CORE_HPP

#include <functional>
#include <list>
#include <rclcpp/rclcpp.hpp>

// Only support sensor_msgs::msg::Image initially, later make it generic
#include <sensor_msgs/msg/image.hpp>

typedef std::function<void (sensor_msgs::msg::Image::SharedPtr)> Function;

inline void setFullTopic(std::shared_ptr<rclcpp::Node> node, std::string& topic)
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

struct Subscriber
{
public:
  Subscriber(const std::string& topic,
      Function callback,
      std::shared_ptr<rclcpp::Node> node=nullptr) :
      topic_(topic)
  {
    setFullTopic(node, topic_);

    bind(callback);
    if (node) {
      std::cout << this << " creating new subscriber with ros sub '" << topic_ << "'\n";
      ros_sub_ = node->create_subscription<sensor_msgs::msg::Image>(topic_, callback_);
    } else {
      std::cout << this << " creating new subscriber without ros sub '" << topic_ << "'\n";
    }
  }
  ~Subscriber()
  {
    std::cout << this << " shutting down subscriber '" << topic_ << "'\n";
  }
  void bind(Function fn)
  {
    callback_ = fn;
  }
  void callback(sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (callback_) {
      callback_(msg);
    }
  }

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
      std::shared_ptr<rclcpp::Node> node=nullptr) :
      topic_(topic)
  {
    setFullTopic(node, topic_);
    if (node) {
      std::cout << this << " creating new publisher with ros pub option '" << topic_ << "'\n";
      ros_pub_ = node->create_publisher<sensor_msgs::msg::Image>(topic);
    } else {
      std::cout << this << " creating new publisher without ros pub option '" << topic_ << "'\n";
      ros_enable_ = false;
    }
  }
  ~Publisher()
  {
    std::cout << this << " shutting down publisher '" << topic_ << "'\n";
    // TODO(lucasw) delete out of core_
  }
  std::string topic_;

  void publish(sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (ros_enable_ && ros_pub_) {
      ros_pub_->publish(msg);
      return;
    }

    std::vector<std::weak_ptr<Subscriber> > to_remove;

    std::lock_guard<std::mutex> lock(sub_mutex_);
    for (auto sub_weak : subs_) {
      if (auto sub = sub_weak.lock()) {
        // std::cout << topic_ << " publishing to " << sub->topic_ << "\n";
        sub->callback(msg);
      } else {
        std::cerr << topic_ << " bad sub lock\n";
        to_remove.push_back(sub_weak);
      }
    }

    #if 0
    for (auto sub_weak : to_remove) {
      std::cout << "removing sub\n";
      // no match for operator
      subs_.remove(sub_weak);
    }
    #endif
  }

  std::list<std::weak_ptr<Subscriber> > subs_;

  // the internal method isn't working fully yet- only updates once
  bool ros_enable_ = false;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ros_pub_;
  std::mutex sub_mutex_;
};

class Core
{
public:
  Core()
  {
    std::cout << std::this_thread::get_id() << " new non-ros core" << std::endl;
  }
  ~Core()
  {
    std::cout << std::this_thread::get_id() << " shutting down non-ros core" << std::endl;
  }

  std::shared_ptr<Subscriber> create_subscription(const std::string& topic,
      Function callback,
      std::shared_ptr<rclcpp::Node> node=nullptr)
  {
    // TODO(lucasw) look through topics and see if callback is already there?
    // otherwise the same callback will get called as many times as this has been
    // called with it.
    auto sub = std::make_shared<Subscriber>(topic, callback, node);

    auto pub = get_create_publisher(topic, node);
    if (pub) {
      std::cout << "creating new subscriber on topic :'" << pub->topic_ << "'\n";
      std::lock_guard<std::mutex> lock(pub->sub_mutex_);
      pub->subs_.push_back(sub);
    } else {
      std::cerr << "couldn't create new subscriber on topic :'" << topic << "'\n";
    }

    return sub;
  }
  std::shared_ptr<Publisher> get_create_publisher(std::string topic,
      std::shared_ptr<rclcpp::Node> node=nullptr)
  {
    std::shared_ptr<Publisher> pub;

    setFullTopic(node, topic);

    if ((publishers_.count(topic) < 1)) { //  || (!(pub = publishers_[topic].lock()))) {
      std::cout << "creating new publisher on topic :'" << topic << "'\n";
      pub = std::make_shared<Publisher>(topic, node);
      pub->ros_enable_ = ros_enable_default_;
      publishers_[topic] = pub;
    } else {
      pub = publishers_[topic];
    }

    return pub;
  }

  bool ros_enable_default_ = false;
  // TODO(lucasw) maybe all future publishers also need to be able to be enabled also?
  void rosEnableAllPublishers(const bool enable)
  {
    for (auto pub_pair : publishers_) {
      auto pub = pub_pair.second;
      if (pub) {
        pub->ros_enable_ = enable;
      }
    }
  }
  // TODO(lucasw)
  // These can't be weak_ptrs because a subscriber without a publisher may need it
  // to stick around, so for now can't delete publishers
  // A deletion function would have to check for zero subscribers.
  std::map<std::string, std::shared_ptr<Publisher> > publishers_;
};

#endif  // IMGUI_ROS_PUB_SUB_CORE_HPP

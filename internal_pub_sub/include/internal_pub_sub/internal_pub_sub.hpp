#ifndef INTERNAL_PUB_SUB_INTERNAL_PUB_SUB_HPP
#define INTERNAL_PUB_SUB_INTERNAL_PUB_SUB_HPP

#include <deque>
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

namespace internal_pub_sub
{

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
    // TODO(lucasw) make this optional
    rclcpp::Time cur = msg->header.stamp;
    stamps_.push_back(cur);
    if (stamps_.size() > 50) {
      stamps_.pop_front();
    } else if ((stamps_.size() > 10) &&
        ((cur - stamps_.front()).nanoseconds() > 2e9)) {
      stamps_.pop_front();
    }

    if (ros_enable_ && ros_pub_) {
      ros_pub_->publish(msg);
      return;
    }

    std::lock_guard<std::mutex> lock(sub_mutex_);
    // remove dead subscribers
    subs_.remove_if([](std::weak_ptr<Subscriber> p) {
        if (auto sp = p.lock()) {
          return false;
        }
        std::cout << "removing dead sub\n";
        return true;
        });

    for (auto sub_weak : subs_) {
      if (auto sub = sub_weak.lock()) {
        // std::cout << topic_ << " publishing to " << sub->topic_ << "\n";
        sub->callback(msg);
      } else {
        // TODO(lucasw) should be impossible to get here after remove_if above
        std::cerr << topic_ << " bad sub lock\n";
      }
    }
  }

  std::list<std::weak_ptr<Subscriber> > subs_;

  // the internal method isn't working fully yet- only updates once
  bool ros_enable_ = false;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ros_pub_;
  std::mutex sub_mutex_;

  std::deque<rclcpp::Time> stamps_;
};

class Core
{
public:
  Core(const bool ros_enable_default=false) : ros_enable_default_(ros_enable_default)
  {
    std::cout << "0x" << std::hex << std::this_thread::get_id() << std::dec
        << " new internal pub sub core" << std::endl;
  }
  ~Core()
  {
    std::cout << "0x" << std::hex << std::this_thread::get_id() << std::dec
        << " shutting down internal pub sub core" << std::endl;
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
      std::cout << "creating new publisher on topic :'" << topic << "' "
          << ros_enable_default_ << "\n";
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
};  // Core

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
      std::shared_ptr<rclcpp::Node> node = nullptr) :
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
    for (size_t i = 0; i < inputs.size(); ++i) {
      // output_topics_.push_back(output);
      subs_.push_back(core->create_subscription(inputs[i],
          std::bind(&Republisher::callback, this, std::placeholders::_1, inputs[i]), node));
      pubs_[inputs[i]] = core->get_create_publisher(outputs[i], node);
    }
  }
  Republisher(const std::string& input, const std::string& output,
      const size_t skip,
      std::shared_ptr<Core> core,
      std::shared_ptr<rclcpp::Node> node = nullptr) :
      skip_max_(skip)
  {
    if (!core) {
      return;
    }
    input_topics_.push_back(input);
    // output_topics_.push_back(output);
    subs_.push_back(core->create_subscription(input,
        std::bind(&Republisher::callback, this, std::placeholders::_1, input), node));
    pubs_[input] = core->get_create_publisher(output, node);
  }

  // TODO(lucasw) what is result if Republisher goes out of scope?

  bool allMessagesReceived(rclcpp::Time stamp)
  {
    if (messages_.count(stamp) < 1) {
      return false;
    }

    return (messages_[stamp].size() == input_topics_.size());
  }

  void callback(sensor_msgs::msg::Image::SharedPtr msg, const std::string& topic)
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
      for (auto pair : messages_[stamp]) {
        pubs_[pair.first]->publish(pair.second);
      }
    }
    messages_.erase(stamp);
  }

  std::vector<std::string> input_topics_;

  std::vector<std::shared_ptr<Subscriber> > subs_;
  std::map<std::string, std::shared_ptr<Publisher> > pubs_;

  std::map<rclcpp::Time,
      std::map<std::string, sensor_msgs::msg::Image::SharedPtr> > messages_;
  size_t skip_count_ = 0;
  size_t skip_max_ = 0;
};  // Republisher

}  // internal_pub_sub
#endif  // INTERNAL_PUB_SUB_INTERNAL_PUB_SUB_HPP

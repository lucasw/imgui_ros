#include <functional>
#include <list>
#include <rclcpp/rclcpp.hpp>

// Only support sensor_msgs::msg::Image initially, later make it generic
#include <sensor_msgs/msg/image.hpp>

typedef std::function<void (sensor_msgs::msg::Image::SharedPtr)> Function;


struct Subscriber
{
public:
  Subscriber(const std::string& name) : name_(name)
  {
    std::cout << this << " creating new subscriber '" << name_ << "'\n";
  }
  ~Subscriber()
  {
    std::cout << this << " shutting down subscriber '" << name_ << "'\n";
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

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr ros_sub_;
private:
  const std::string name_;
  Function callback_;
};

struct Publisher : std::enable_shared_from_this<Publisher>
{
  Publisher(const std::string& name) : name_(name)
  {
    std::cout << this << " creating new publisher '" << name_ << "'\n";
  }
  ~Publisher()
  {
    std::cout << this << " shutting down publisher '" << name_ << "'\n";
    // TODO(lucasw) delete out of core_
  }
  std::string name_;

  void publish(sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (ros_enable_ && ros_pub_) {
      ros_pub_->publish(msg);
      return;
    }

    std::vector<std::weak_ptr<Subscriber> > to_remove;

    for (auto sub_weak : subs_) {
      if (auto sub = sub_weak.lock()) {
        sub->callback(msg);
      } else {
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
  bool ros_enable_ = true;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ros_pub_;
};

class Core
{
public:
  std::shared_ptr<Subscriber> create_subscription(const std::string& topic, Function callback,
      std::shared_ptr<rclcpp::Node> node=nullptr)
  {
    // TODO(lucasw) look through topics and see if callback is already there?
    auto sub = std::make_shared<Subscriber>(topic);
    sub->bind(callback);

    auto pub = create_publisher(topic);
    if (pub) {
      std::cout << "creating new subscriber on topic :'" << topic << "'\n";
      pub->subs_.push_back(sub);
    } else {
      std::cerr << "couldn't create new subscriber on topic :'" << topic << "'\n";
    }

    if (node) {
      sub->ros_sub_ = node->create_subscription<sensor_msgs::msg::Image>(topic, callback);
    }
    return sub;
  }
  std::shared_ptr<Publisher> create_publisher(const std::string& topic,
      std::shared_ptr<rclcpp::Node> node=nullptr)
  {
    std::shared_ptr<Publisher> pub;

    if ((publishers_.count(topic) < 1)) { //  || (!(pub = publishers_[topic].lock()))) {
      std::cout << "creating new publisher on topic :'" << topic << "'\n";
      pub = std::make_shared<Publisher>(topic);
      publishers_[topic] = pub;
    } else {
      pub = publishers_[topic];
    }

    if (node) {
      pub->ros_pub_ = node->create_publisher<sensor_msgs::msg::Image>(topic);
    }
    return pub;
  }

private:
  // TODO(lucasw)
  // These can't be weak_ptrs because a subscriber without a publisher may need it
  // to stick around, so for now can't delete publishers
  // A deletion function would have to check for zero subscribers.
  std::map<std::string, std::shared_ptr<Publisher> > publishers_;
};

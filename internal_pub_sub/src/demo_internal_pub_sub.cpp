#include <internal_pub_sub/internal_pub_sub.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

struct Pub
{
  Pub(const std::string& topic,
      std::shared_ptr<internal_pub_sub::Core> core)  // : core_(core)
  {
    pub_ = core->get_create_publisher(topic);
  }

  ~Pub()
  {
    std::cout << this << " shutting down wrapper Publisher on " << pub_->topic_ << "\n";
  }

  void update(rclcpp::Time stamp)
  {
    auto msg = std::make_shared<sensor_msgs::msg::Image>();
    msg->header.frame_id = "map";
    msg->header.stamp = stamp;
    msg->width = 1024;
    msg->height = 1024;
    msg->encoding = "mono8";
    msg->step = msg->width;
    msg->data.resize(msg->step * msg->height);
    std::cout << "pub " << pub_->topic_ << " " << this << " " << msg->header.stamp.sec << "\n";
    pub_->publish(msg);
  }

  // std::shared_ptr<internal_pub_sub::Core> core_;
  std::shared_ptr<internal_pub_sub::Publisher> pub_;
  int count_ = 0;
};

struct Sub
{
  Sub(const std::string& topic, std::shared_ptr<internal_pub_sub::Core> core)  // : core_(core)
  {
    sub_ = core->create_subscription(topic,
        std::bind(&Sub::callback, this, std::placeholders::_1));
  }

  ~Sub()
  {
    std::cout << this << " shutting down wrapper Subscriber on " << sub_->topic_ << "\n";
  }

  void callback(sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::cout << "sub " << sub_->topic_ << " " << this << " new message " << msg->header.stamp.sec
        << " " << msg->data.size() << "\n";
  }
  // std::shared_ptr<internal_pub_sub::Core> core_;
  std::shared_ptr<internal_pub_sub::Subscriber> sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto core = std::make_shared<internal_pub_sub::Core>();
  std::shared_ptr<Pub> publisher_node = nullptr;  // std::make_shared<Pub>("foo2_throttled", core);
  std::shared_ptr<Pub> publisher_node2 = nullptr;
  std::cout << "------\n";
  std::shared_ptr<Sub> subscriber_node = nullptr;  // std::make_shared<Sub>("foo", core);
  std::shared_ptr<Sub> subscriber_node2 = nullptr;
  std::shared_ptr<Sub> subscriber_node3 = nullptr;
  std::cout << "------\n";
  std::cout << std::endl;

#if 0
  auto single_republisher = std::make_shared<internal_pub_sub::Republisher>(
      "foo", "foo_throttled", 4, core);
#endif
  std::vector<std::string> inputs = {"foo", "foo2"};
  std::vector<std::string> outputs = {"foo_throttled", "foo2_throttled"};
  std::shared_ptr<internal_pub_sub::Republisher> republisher = nullptr;

  int count = 0;
  while (rclcpp::ok()) {
    std::cout << "---------------- " << count << " --------------------\n";
    rclcpp::Time stamp(count, 0);
    if (publisher_node) {
      publisher_node->update(stamp);
      std::cout << "pub node use count " << publisher_node->pub_.use_count() << "\n";
    }
    if (publisher_node2) publisher_node2->update(stamp);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    // std::cout << count << "\n";

    #if 1
    if (count == 1) {
      publisher_node = nullptr;
      // core->publishers_.erase(publisher_node->pub_->topic_);
    }
    if (count == 1) {
      subscriber_node2 = std::make_shared<Sub>("foo_throttled", core);
    }
    if (count == 2) {
      // TODO(lucasw) core->create_republisher(inputs, outputs, 4);
      republisher = std::make_shared<internal_pub_sub::Republisher>(
        inputs, outputs, 4, core);
    }
    if (count == 3) {
      publisher_node2 = std::make_shared<Pub>("foo2", core);
    }
    #endif
    if (count == 4) {
      subscriber_node3 = std::make_shared<Sub>("foo2_throttled", core);
    }
    if (count == 5) {
      subscriber_node = nullptr;
      republisher = nullptr;
    }
    if (count == 6) {
      subscriber_node2 = nullptr;
      subscriber_node3 = nullptr;
    }
    if (count > 8) {
      break;
    }
    core->clean();
    ++count;
  }
  std::cout << std::endl;
#if 0
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(publisher_node);
  exec.add_node(subscriber_node);
  exec.spin();
#endif
  rclcpp::shutdown();
  return 0;
}


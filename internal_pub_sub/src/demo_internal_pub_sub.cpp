#include <internal_pub_sub/internal_pub_sub.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

struct Pub
{
  Pub(const std::string& topic,
      std::shared_ptr<internal_pub_sub::Core> core) : core_(core)
  {
    pub_ = core_->get_create_publisher(topic);
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

  std::shared_ptr<internal_pub_sub::Core> core_;
  std::shared_ptr<internal_pub_sub::Publisher> pub_;
  int count_ = 0;
};

struct Sub
{
  Sub(const std::string& topic, std::shared_ptr<internal_pub_sub::Core> core) : core_(core)
  {
    sub_ = core_->create_subscription(topic,
        std::bind(&Sub::callback, this, std::placeholders::_1));
  }

  ~Sub()
  {
    std::cout << this << " shutting down Subscriber\n";
  }

  void callback(sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::cout << "sub " << sub_->topic_ << " " << this << " new message " << msg->header.stamp.sec
        << " " << msg->data.size() << "\n";
  }
  std::shared_ptr<internal_pub_sub::Core> core_;
  std::shared_ptr<internal_pub_sub::Subscriber> sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto core = std::make_shared<internal_pub_sub::Core>();
  auto publisher_node = std::make_shared<Pub>("foo", core);
  auto publisher_node2 = std::make_shared<Pub>("foo2", core);
  std::cout << "------\n";
  auto subscriber_node = nullptr;  // std::make_shared<Sub>("foo", core);
  auto subscriber_node2 = std::make_shared<Sub>("foo_throttled", core);
  auto subscriber_node3 = std::make_shared<Sub>("foo2_throttled", core);
  std::cout << "------\n";
  std::cout << std::endl;

#if 0
  auto republisher = std::make_shared<internal_pub_sub::Republisher>(
      "foo", "foo_throttled", 4, core);
#else
  std::vector<std::string> inputs = {"foo", "foo2"};
  std::vector<std::string> outputs = {"foo_throttled", "foo2_throttled"};
  auto republisher = std::make_shared<internal_pub_sub::Republisher>(
      inputs, outputs, 4, core);

#endif

  int count = 0;
  while (rclcpp::ok()) {
    rclcpp::Time stamp(count, 0);
    publisher_node->update(stamp);
    publisher_node2->update(stamp);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    // std::cout << count << "\n";
    if (count > 10) {
      subscriber_node = nullptr;
    }

    if (count > 20) {
      break;
    }
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


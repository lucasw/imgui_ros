#include <imgui_ros/pub_sub_core.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

struct Pub
{
  Pub(std::shared_ptr<Core> core) : core_(core)
  {
    pub_ = core_->create_publisher("foo");
  }

  void update()
  {
    auto msg = std::make_shared<sensor_msgs::msg::Image>();
    msg->header.frame_id = "map";
    msg->header.stamp.sec = count_++;
    msg->width = 1024;
    msg->height = 1024;
    msg->encoding = "mono8";
    msg->step = msg->width;
    msg->data.resize(msg->step * msg->height);
    std::cout << this << " " << msg->header.stamp.sec << "\n";
    pub_->publish(msg);
  }

  std::shared_ptr<Core> core_;
  std::shared_ptr<Publisher> pub_;
  int count_ = 0;
};

struct Sub
{
  Sub(std::shared_ptr<Core> core) : core_(core)
  {
    sub_ = core_->create_subscription("foo", std::bind(&Sub::callback, this, std::placeholders::_1));
  }

  ~Sub()
  {
    std::cout << this << " shutting down Subscriber\n";
  }

  void callback(sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::cout << this << " new message " << msg->header.stamp.sec
        << " " << msg->data.size() << "\n";
  }
  std::shared_ptr<Core> core_;
  std::shared_ptr<Subscriber> sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto core = std::make_shared<Core>();
  auto publisher_node = std::make_shared<Pub>(core);
  std::cout << "------\n";
  auto subscriber_node = std::make_shared<Sub>(core);
  std::cout << "------\n";
  std::cout << std::endl;

  int count = 0;
  while (rclcpp::ok()) {
    publisher_node->update();
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


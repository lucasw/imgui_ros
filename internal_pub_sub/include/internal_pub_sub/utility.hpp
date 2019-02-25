#ifndef INTERNAL_PUB_SUB_UTILITY_HPP
#define INTERNAL_PUB_SUB_UTILITY_HPP

#include <functional>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

typedef std::function<void (sensor_msgs::msg::Image::SharedPtr)> Function;

void setFullTopic(std::shared_ptr<rclcpp::Node> node, std::string& topic);

#endif  // INTERNAL_PUB_SUB_UTILITY_HPP

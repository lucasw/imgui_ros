// Copyright 2014 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "ros/ros.h"
#include "rcutils/cmdline_parser.h"

#include "std_msgs/float32.hpp"

using namespace std::chrono_literals;

void print_usage()
{
  printf("Usage for talker app:\n");
  printf("talker [-t topic_name] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-t topic_name : Specify the topic on which to publish. Defaults to chatter.\n");
}

// Create a ParamToTopic class that subclasses the generic ros::Node base class.
// The main function below will instantiate the class as a ROS node.
class ParamToTopic : public ros::Node
{
public:
  explicit ParamToTopic()
  : Node("param_to_topic")
  {
  }

  // can't init in constructor because shared_from_this will throw bad_weak_ptr
  void init()
  {
    msg_ = std::make_shared<std_msgs::Float32>();
    const std::string topic_name = "test";

    // Typically a parameter client is created for a remote node by passing the name of the remote
    // node in the constructor; in this example we create a parameter client for this node itself.
    parameters_client_ = std::make_shared<ros::AsyncParametersClient>(this);

    auto on_parameter_event_callback =
      [this](const rcl_interfaces::ParameterEvent::SharedPtr event) -> void
      {
        // TODO(wjwwood): The message should have an operator<<, which would replace all of this.
        std::stringstream ss;
        ss << "\nParameter event:\n new parameters:";
        for (auto & new_parameter : event->new_parameters) {
          ss << "\n  " << new_parameter.name;
        }
        ss << "\n changed parameters:";
        for (auto & changed_parameter : event->changed_parameters) {
          ss << "\n  " << changed_parameter.name;
          if (changed_parameter.name == "foo") {
            float val;
            get_parameter_or("foo", val, 0.0f);
            msg_->data = val;
            ss << " " << val << " ";
          }
        }
        ss << "\n deleted parameters:";
        for (auto & deleted_parameter : event->deleted_parameters) {
          ss << "\n  " << deleted_parameter.name;
        }
        ss << "\n";
        ROS_INFO_STREAM(ss.str().c_str());
      };

    // Setup callback for changes to parameters.
    parameter_event_sub_ = parameters_client_->on_parameter_event(on_parameter_event_callback);

    // Node already has std::enable_shared_from_this
    parameters_client_ = std::make_shared<ros::AsyncParametersClient>(shared_from_this());
    while (!parameters_client_->wait_for_service(1s)) {
      if (!ros::ok()) {
        ROS_ERROR_STREAM("Interrupted while waiting for the service. Exiting.");
        return;
      }
      ROS_INFO_STREAM("service not available, waiting again...");
    }

    // TODO(lucasw) there ought to be a set_parameter for setting just one parameter
    #if 0
    auto results = parameters_client_->set_parameters({ros::Parameter("foo", 2.0)});
    for (auto result : results) {
      if (!result.successful) {
        ROS_ERROR_STREAM("Failed to set parameter: %s", result.reason.c_str());
      }
    }
    #endif

    // Create a function for when messages are to be sent.
    auto publish_message =
      [this]() -> void
      {
        #if 0
        for (auto & parameter : parameters_client_->get_parameters({"foo"})) {
          msg_->data = parameter.as_double();
        }
        #endif
        // + std::to_string(count_++);
        // ROS_INFO_STREAM("Publishing: '%s'", msg_->data.c_str());

        // Put the message into a queue to be processed by the middleware.
        // This call is non-blocking.
        pub_->publish(msg_);
      };

    // Create a publisher with a custom Quality of Service profile.
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    custom_qos_profile.depth = 7;
    pub_ = this->create_publisher<std_msgs::Float32>(topic_name, custom_qos_profile);
    // Use a timer to schedule periodic message publishing.
    timer_ = this->create_wall_timer(1s, publish_message);
  }

private:
  std::shared_ptr<ros::AsyncParametersClient> parameters_client_;
  ros::Subscription<rcl_interfaces::ParameterEvent>::SharedPtr parameter_event_sub_;
  std::shared_ptr<std_msgs::Float32> msg_;
  ros::Publisher<std_msgs::Float32>::SharedPtr pub_;
  ros::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  // Initialize any global resources needed by the middleware and the client library.
  // You must call this before using any other part of the ROS system.
  // This should be called once per process.
  ros::init(argc, argv);

  // Create a node.
  auto node = std::make_shared<ParamToTopic>();
  node->init();

  // spin will block until work comes in, execute work as it becomes available, and keep blocking.
  // It will only be interrupted by Ctrl-C.
  ros::spin(node);

  ros::shutdown();
  return 0;
}

/*
 * Copyright (c) 2017 Lucas Walter
 * June 2017
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "imgui.h"
#include <imgui_ros/viz3d.h>
#include <imgui_ros/imgui_impl_opengl3.h>
#include <imgui_ros/srv/add_tf.hpp>
#include <imgui_ros/srv/add_window.hpp>
#include <map>
#include <mutex>
#include <opencv2/core.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_listener.h>
#include <SDL.h>

namespace imgui_ros {
class ImguiRos : public rclcpp::Node {
public:
  ImguiRos();
  ~ImguiRos();

private:
  rclcpp::Service<srv::AddTf>::SharedPtr add_tf_;
  void addTf(const std::shared_ptr<imgui_ros::srv::AddTf::Request> req,
             std::shared_ptr<imgui_ros::srv::AddTf::Response> res);

  void addWindow(const std::shared_ptr<imgui_ros::srv::AddWindow::Request> req,
                 std::shared_ptr<imgui_ros::srv::AddWindow::Response> res);
  bool addWidget(const imgui_ros::msg::Widget& widget,
      std::string& message, std::shared_ptr<Widget>& imgui_widget);
  void update();

  // Need to init the opengl context in same thread as the update
  // is run in, not necessarily the same thread onInit runs in
  void glInit();
  std::mutex mutex_;
  bool init_;
  // TODO(lucasw) std::shared_ptr
  SDL_Window *window;
  SDL_GLContext gl_context;
  // ImVec4 clear_color_ = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

  std::map<std::string, std::shared_ptr<Window> > windows_;

  // TODO(lucasw) still need to update even if ros time is paused
  rclcpp::TimerBase::SharedPtr update_timer_;

  rclcpp::Service<srv::AddWindow>::SharedPtr add_window_;

  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;

  std::string name_ = "imgui_ros";
  int width_ = 1280;
  int height_ = 720;

  std::shared_ptr<ImGuiImplOpenGL3> imgui_impl_opengl3_;
  std::shared_ptr<Viz3D> viz3d;

  // this will get parameter events for all nodes in same namespace (or just root namespace?
  // namespacing seems broken in ros2 currently)
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_sub_;
  void onParameterEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);
};

}  // namespace imgui_ros

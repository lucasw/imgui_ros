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

#ifndef IMGUI_ROS_IMGUI_ROS_H
#define IMGUI_ROS_IMGUI_ROS_H

#include <imgui.h>
#include <imgui_ros_msgs/AddWindow.h>
// #include <imgui_ros/viz3d.h>
#include <imgui_ros/image_transfer.h>
#include <imgui_ros/imgui_impl_opengl3.h>
#include <imgui_ros/window.h>
#if 0
#include <imgui_ros/param.h>
#include <internal_pub_sub/internal_pub_sub.hpp>
#include <imgui_ros/AddTf.hpp>
#endif
#include <map>
#include <mutex>
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include <SDL.h>

namespace imgui_ros {

class ImguiRos {
public:
  ImguiRos();
  ~ImguiRos();
  void postInit();

private:
  ros::NodeHandle nh_;
#if 0
  void runNodeSingleThreaded(ros::NodeHandle& nh);
  ros::Service<AddTf>::SharedPtr add_tf_;
  void addTf(const std::shared_ptr<imgui_ros::AddTf::Request> req,
             std::shared_ptr<imgui_ros::AddTf::Response> res);
#endif
  bool addWindow(imgui_ros_msgs::AddWindow::Request& req,
                 imgui_ros_msgs::AddWindow::Response& res);
  bool addWidget(const imgui_ros_msgs::Widget& widget,
      std::string& message, std::shared_ptr<Widget>& imgui_widget);
  // TODO(lucasw) still need to update even if ros time is paused
  ros::Timer update_timer_;
  void update(const ros::TimerEvent& ev);

  // Need to init the opengl context in same thread as the update
  // is run in, not necessarily the same thread onInit runs in
  void glInit();
  std::mutex mutex_;
  bool init_ = false;
  // TODO(lucasw) std::shared_ptr
  SDL_Window *sdl_window_;
  SDL_GLContext gl_context;
  // ImVec4 clear_color_ = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

  std::map<std::string, std::shared_ptr<Window> > windows_;
  ros::ServiceServer add_window_;

#if 0
  ros::Clock::SharedPtr clock_;
  // std::shared_ptr<ros::Node> tf_node_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  // TODO(lucasw) maybe a non shared pointer works better
  // tf2_ros::Buffer buffer_;

  ros::Publisher<tf2_msgs::TFMessage>::SharedPtr tf_pub_;
#endif
  std::string name_ = "imgui_ros";
  int width_ = 1280;
  int height_ = 720;
  int old_width_ = 0;
  int old_height_ = 0;
  int old_x_ = 0;
  int old_y_ = 0;

  bool fullscreen_ = false;

  ros::Time start_stamp_ = ros::Time::now();
  bool stats_window_init_ = true;
  void drawStats(ros::Time stamp);

  std::shared_ptr<ImGuiImplOpenGL3> imgui_impl_opengl3_;
#if 0
  std::shared_ptr<Viz3D> viz3d;

  std::map<std::string, ros::AsyncParametersClient::SharedPtr> parameters_clients_;
  // node_name, widget_name
  std::map<std::string, std::map<std::string, std::shared_ptr<Param> > > param_widgets_;

  // thread dedicate to large ros messages
  std::thread ros_io_thread_;

  // this will get parameter events for all nodes in same namespace (or just root namespace?
  // namespacing seems broken in ros2 currently)
  // void onParameterEvent(const rcl_interfaces::ParameterEvent::SharedPtr event);
#endif
  std::shared_ptr<ImageTransfer> image_transfer_;
  // check to make sure opengl context accesses never happen in different thread
  std::thread::id thread_id_;
};

}  // namespace imgui_ros

#endif  // IMGUI_ROS_IMGUI_ROS_H

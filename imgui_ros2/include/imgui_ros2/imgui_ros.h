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
// #include "imgui_impl_opengl3.h"
// #include "imgui_impl_sdl.h"
#include <imgui_ros2/image.h>
// #include <imgui_ros/dynamic_reconfigure.h>
#include <map>
#include <mutex>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <SDL.h>


namespace imgui_ros2 {
class ImguiRos : public rclcpp::Node {
public:
  ImguiRos();
  ~ImguiRos();

private:
  // bool addWindow(imgui_ros::srv::AddWindow::Request& req,
  //                imgui_ros::srv::AddWindow::Response& res);
  void update();

  // Need to init the opengl context in same thread as the update
  // is run in, not necessarily the same thread onInit runs in
  void glInit();
  std::mutex mutex_;
  bool init_;
  // TODO(lucasw) std::shared_ptr
  SDL_Window *window;
  ImGuiIO io;
  SDL_GLContext gl_context;
  ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

  std::map<std::string, std::shared_ptr<Window> > windows_;

  // TODO(lucasw) still need to update even if ros time is paused
  rclcpp::TimerBase::SharedPtr update_timer_;

  // ros::ServiceServer add_window_;
};

}  // namespace imgui_ros

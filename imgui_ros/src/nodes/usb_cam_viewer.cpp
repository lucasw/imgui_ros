/*
 * Copyright (c) 2018 Lucas Walter
 * October 2018
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

#include <memory>
#include <imgui_ros/imgui_ros.h>
#include <usb_cam/usb_cam.hpp>
#include <internal_pub_sub/internal_pub_sub.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <thread>

void run_usb_cam(std::shared_ptr<internal_pub_sub::Core> core)
{
  // rclcpp::executors::MultiThreadedExecutor executor;
  rclcpp::executors::SingleThreadedExecutor executor;
  auto usb_cam = std::make_shared<usb_cam::UsbCam>();
  usb_cam->init("usb_cam");
  usb_cam->postInit(core);
  executor.add_node(usb_cam);
  executor.spin();
}

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);


  auto core = std::make_shared<internal_pub_sub::Core>();

  rclcpp::executors::SingleThreadedExecutor single_executor;
  // imgui_ros has to be single threaded for now to avoid context switches with opengl
  auto imgui_ros = std::make_shared<imgui_ros::ImguiRos>(core);
  single_executor.add_node(imgui_ros);

  rclcpp::WallRate rate(50);
#if 0
  // This doesn't work even though the execution time out to be in a different thread than
  // this one- need to spawn to different threads to spin each executor in.
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  while (rclcpp::ok()) {
    single_executor.spin_some();
    multi_executor.spin_some();
    rate.sleep();
  }
#else
  std::thread cam_thread(std::bind(run_usb_cam, core));
  single_executor.spin();
  cam_thread.join();
#endif

  rclcpp::shutdown();
  return 0;
}

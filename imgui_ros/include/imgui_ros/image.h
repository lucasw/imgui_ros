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

#ifndef IMGUI_ROS_IMAGE_H
#define IMGUI_ROS_IMAGE_H

#include <deque>
#include <imgui.h>
#include <imgui_ros/image_transfer.h>
#include <imgui_ros/imgui_impl_opengl3.h>
// #include <internal_pub_sub/internal_pub_sub.hpp>
#include <imgui_ros/window.h>
#include <mutex>
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

namespace imgui_ros
{
struct GlImage : public Widget {
  GlImage(const std::string name, const std::string topic);
  ~GlImage();
  virtual bool updateTexture() = 0;
  virtual void draw() = 0;

  // TODO(lucasw) or NULL or -1?
  GLuint texture_id_ = 0;
// protected:
  size_t width_ = 0;
  size_t height_ = 0;
};

// TODO(lucasw) move ros specific code out, have not ros code in common
// location that ros1 and ros2 versions can use.
// TODO(lucasw) should every window be a node?  Or less overhead to
// have a single node in the imgui parent?
struct RosImage : public GlImage {
  RosImage(const std::string name, const std::string topic = "",
           const bool sub_not_pub = false,
           const bool ros_pub = false,
           // ros::NodeHandle& nh = nullptr,
           std::shared_ptr<ImageTransfer> image_transfer = nullptr);
  RosImage(const std::string& name,
    sensor_msgs::Image::SharedPtr image);

  // void imageCallback(const sensor_msgs::Image::SharedPtr msg);

  // TODO(lucasw) factor this into a generic opengl function to put in parent class
  // if the image changes need to call this
  virtual bool updateTexture();

  // TODO(lucasw) factor out common code
  virtual void draw();

  virtual void publish(const ros::Time& stamp);

  sensor_msgs::Image::SharedPtr image_;

  int wrap_s_ind_ = 0;
  int wrap_t_ind_ = 0;
  int min_filter_ind_ = 5;
  int mag_filter_ind_ = 1;
  bool draw_texture_controls_ = false;
  bool enable_draw_image_ = false;
  bool enable_cpu_to_gpu_ = true;
  std::string header_frame_id_ = "";
  // is there a fresh image to publish?
  bool pub_dirty_ = true;
  // may want to keep the image just within imgui, don't send it anywhere
  bool enable_publish_ = true;
private:
  const bool sub_not_pub_ = false;
  std::weak_ptr<ros::Node> node_;
  std::shared_ptr<ImageTransfer> image_transfer_;

  // temp until image_transfer supports subs
  // ros::Subscription<sensor_msgs::Image>::SharedPtr sub_;

  std::vector<int> min_filter_modes_;
  std::vector<int> mag_filter_modes_;
  std::vector<int> wrap_modes_;

  // TODO(lucasw) Duration(0, 0) may have resulted in crashes?
  ros::Duration image_gap_ = ros::Duration(0);
  ros::Duration image_age_ = ros::Duration(0);
  bool enable_one_to_one_ = false;
};  // RosImage

struct CvImage : public GlImage {
  CvImage(const std::string name);
  // TODO(lucasw) instead of cv::Mat use a sensor_msgs Image pointer,
  // an convert straight from that format rather than converting to cv.
  // Or just have two implementations of Image here, the cv::Mat
  // would be good to keep as an example.
  cv::Mat image_;

  // if the image changes need to call this
  virtual bool updateTexture();
  virtual void draw();
};
}  // namespace imgui_ros
#endif  // IMGUI_ROS_IMAGE_H

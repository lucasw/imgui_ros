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
#include <imgui_ros/imgui_impl_opengl3.h>
#include <imgui_ros/window.h>
#include <mutex>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

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

class ImageTransfer;

// TODO(lucasw) move ros specific code out, have not ros code in common
// location that ros1 and ros2 versions can use.
// TODO(lucasw) should every window be a node?  Or less overhead to
// have a single node in the imgui parent?
struct RosImage : public GlImage {
  RosImage(const std::string name, const std::string topic = "",
           const bool pub_not_sub = false,
           std::shared_ptr<rclcpp::Node> node = nullptr,
           std::shared_ptr<ImageTransfer> image_transfer = nullptr);

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  // TODO(lucasw) factor this into a generic opengl function to put in parent class
  // if the image changes need to call this
  virtual bool updateTexture();

  // TODO(lucasw) factor out common code
  virtual void draw();

  virtual void publish(const rclcpp::Time& stamp);

  sensor_msgs::msg::Image::SharedPtr image_;

  int wrap_s_ind_ = 0;
  int wrap_t_ind_ = 0;
  int min_filter_ind_ = 5;
  int mag_filter_ind_ = 1;
  bool draw_texture_controls_ = false;
  bool enable_draw_image_ = false;
private:
  std::weak_ptr<rclcpp::Node> node_;
  std::shared_ptr<ImageTransfer> image_transfer_;

  // temp until image_transfer supports subs
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

  std::vector<int> min_filter_modes_;
  std::vector<int> mag_filter_modes_;
  std::vector<int> wrap_modes_;

  bool enable_info_ = true;
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

class ImageTransfer : public rclcpp::Node
{
public:
  ImageTransfer() : Node("image_transfer")
  {

  }

  // this will remove the image from the queue, so can't have
  // multiple subscriber on same message- they need to share downstream from here
  bool getSub(const std::string& topic, sensor_msgs::msg::Image::SharedPtr& image)
  {
    std::lock_guard<std::mutex> lock(sub_mutex_);
    // TODO(lucasw) if the sub doesn't exist at all need to create it
    if (from_sub_.count(topic) < 1) {
      return false;
    }
    image = from_sub_[topic];
    from_sub_.erase(topic);
    return true;
  }

  bool publish(const std::string& topic, sensor_msgs::msg::Image::SharedPtr image)
  {
    std::lock_guard<std::mutex> lock(pub_mutex_);
    to_pub_.push_back(std::pair<std::string, sensor_msgs::msg::Image::SharedPtr>(topic, image));
    return true;
  }

  // TODO(lucasw) need way to remove publisher or subscriber

  void update()
  {
    {
      std::lock_guard<std::mutex> lock(pub_mutex_);
      while (to_pub_.size() > 0) {
        const std::string topic = to_pub_.front().first;
        sensor_msgs::msg::Image::SharedPtr image = to_pub_.front().second;
        to_pub_.pop_front();

        if (pubs_.count(topic) < 1) {
          pubs_[topic] = create_publisher<sensor_msgs::msg::Image>(topic);
        }
        pubs_[topic]->publish(image);
      }
    }
  }
private:
  std::mutex sub_mutex_;
  std::map<std::string, sensor_msgs::msg::Image::SharedPtr> from_sub_;
  std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subs_;

  std::mutex pub_mutex_;
  std::deque<std::pair<std::string, sensor_msgs::msg::Image::SharedPtr> > to_pub_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> pubs_;

};
#endif  // IMGUI_ROS_IMAGE_H

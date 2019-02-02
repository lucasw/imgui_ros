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
#include <internal_pub_sub/internal_pub_sub.hpp>
#include <imgui_ros/window.h>
#include <mutex>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

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
           const bool sub_not_pub = false,
           std::shared_ptr<rclcpp::Node> node = nullptr,
           std::shared_ptr<ImageTransfer> image_transfer = nullptr);
  RosImage(const std::string& name,
    sensor_msgs::msg::Image::SharedPtr image);

  // void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

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
  bool enable_cpu_to_gpu_ = true;
  std::string header_frame_id_ = "";
  // is there a fresh image to publish?
  bool pub_dirty_ = true;
  // may want to keep the image just within imgui, don't send it anywhere
  bool enable_publish_ = true;
private:
  const bool sub_not_pub_ = false;
  std::weak_ptr<rclcpp::Node> node_;
  std::shared_ptr<ImageTransfer> image_transfer_;

  // temp until image_transfer supports subs
  // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

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

// TODO(lucasw) get rid of this and restore pub/sub to where needed
class ImageTransfer : public rclcpp::Node
{
public:
  rclcpp::TimerBase::SharedPtr update_timer_;
  ImageTransfer(std::shared_ptr<internal_pub_sub::Core> core) :
      Node("image_transfer"),
      core_(core)
  {
    if (!core_) {
      std::cout << "image_transfer making new pub sub core\n";
      core_ = std::make_shared<internal_pub_sub::Core>();
    }
    update_timer_ = this->create_wall_timer(33ms,
        std::bind(&ImageTransfer::update, this));
  }

  bool getSub(const std::string& topic, sensor_msgs::msg::Image::SharedPtr& image)
  {
    std::lock_guard<std::mutex> lock(sub_mutex_);
    if (subs_.count(topic) < 1) {
      // TODO(lucasw) is it better to create the publisher here
      // or inside the thread update is running in?
      std::function<void(std::shared_ptr<sensor_msgs::msg::Image>)> fnc;
      fnc = std::bind(&ImageTransfer::imageCallback, this, std::placeholders::_1,
              topic);
      // subs_[topic] = create_subscription<sensor_msgs::msg::Image>(topic, fnc);
      subs_[topic] = core_->create_subscription(topic, fnc, shared_from_this());
      // subs_[topic] = nullptr;
    }
    // TODO(lucasw) if the sub doesn't exist at all need to create it
    if (from_sub_.count(topic) < 1) {
      return false;
    }
    image = from_sub_[topic];
    // this will remove the image from the queue, so can't have
    // multiple subscriber on same message- they need to share downstream from here
    // So instead keep all the most recent messages on every topic
    // from_sub_.erase(topic);
    return true;
  }

  bool publish(const std::string& topic, sensor_msgs::msg::Image::SharedPtr image)
  {
    std::lock_guard<std::mutex> lock(pub_mutex_);
    to_pub_.push_back(std::pair<std::string, sensor_msgs::msg::Image::SharedPtr>(topic, image));
    return true;
  }

  // TODO(lucasw) need way to remove publisher or subscriber

  // TODO(lucasw) virtual void draw()

  void update()
  {
    // std::lock_guard<std::mutex> lock(sub_mutex_);
    if (!initted_) {
      std::cout << "image transfer " << std::this_thread::get_id() << "\n";
      initted_ = true;
    }
    {
      while (to_pub_.size() > 0) {
        std::string topic;
        sensor_msgs::msg::Image::SharedPtr image;
        {
          std::lock_guard<std::mutex> lock(pub_mutex_);
          topic = to_pub_.front().first;
          image = to_pub_.front().second;
          to_pub_.pop_front();
        }
        auto pub = core_->get_create_publisher(topic, shared_from_this());
        if (pub) {
          pub->publish(image);
        }
      }
    }  // publish all queued up messages
  }

  void draw(rclcpp::Time cur)
  {
    // auto cur = now();

    ImGui::Separator();
    ImGui::Text("enable sensor_msgs/Image publishing, otherwise in-process only");
    // TODO(lucasw) turn all the publishers on or off with a master checkbox
    // ImGui::Checkbox("multisample", &multisample_);
    ImGui::Columns(2);
    for (auto pub_pair : core_->publishers_) {
      auto pub = pub_pair.second;
      if (pub) {
        ImGui::Checkbox(pub->topic_.c_str(), &pub->ros_enable_);
        ImGui::NextColumn();
        ImGui::Text("%lu subs", pub->subs_.size());
        ImGui::NextColumn();
        float rate = 0.0;
        if (pub->stamps_.size() > 2) {
          rclcpp::Time earliest = pub->stamps_.front();
          rate = static_cast<float>(pub->stamps_.size()) /
            ((cur - earliest).nanoseconds() / 1e9);
        }
        ImGui::Text("%0.2f Hz", rate);
        ImGui::NextColumn();
        float time_since_last = 0.0;
        if (pub->stamps_.size() > 0) {
          rclcpp::Time latest = pub->stamps_.back();
          time_since_last =  (cur - latest).nanoseconds() / 1e9;
        }
        ImGui::Text("%0.2f since last", time_since_last);
        ImGui::NextColumn();
      }
    }
    ImGui::Columns(1);
  }

private:
  std::shared_ptr<internal_pub_sub::Core> core_;
  bool initted_ = false;
  std::mutex sub_mutex_;
  void imageCallback(sensor_msgs::msg::Image::SharedPtr msg, const std::string& topic)
  {
    // std::cout << "image transfer " << topic << " msg received " << msg->header.stamp.sec << "\n";
    std::lock_guard<std::mutex> lock(sub_mutex_);
    from_sub_[topic] = msg;
  }

  std::map<std::string, sensor_msgs::msg::Image::SharedPtr> from_sub_;
  std::map<std::string, std::shared_ptr<internal_pub_sub::Subscriber> > subs_;

  std::mutex pub_mutex_;
  std::deque<std::pair<std::string, sensor_msgs::msg::Image::SharedPtr> > to_pub_;
};
#endif  // IMGUI_ROS_IMAGE_H

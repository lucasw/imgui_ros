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

#include "imgui.h"
#include "imgui_impl_opengl3.h"
#include "imgui_impl_sdl.h"
// #include <imgui_ros/AddWindow.h>
#include <imgui_ros2/image.h>
// #include <opencv2/highgui.hpp>
using std::placeholders::_1;

// namespace {
  GlImage::GlImage(const std::string name, const std::string topic) :
      Window(name, topic) {
    glGenTextures(1, &texture_id_);
  }

  GlImage::~GlImage() {
    std::cout << "freeing texture " << texture_id_ << " " << name_ << "\n";
    glDeleteTextures(1, &texture_id_);
  }

  RosImage::RosImage(const std::string name, const std::string topic,
                     std::shared_ptr<rclcpp::Node> node) : GlImage(name, topic), node_(node) {
    std::cout << "subscribing to topic " << topic << "\n";
    sub_ = node->create_subscription<sensor_msgs::msg::Image>(topic,
        std::bind(&RosImage::imageCallback, this, _1));
  }

  void RosImage::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
#if 0
    ROS_DEBUG_STREAM("image callback "
        << msg->header.stamp << " "
        << msg->data.size() << " "
        << msg->width << " " << msg->height << ", "
        << sub_.getTopic());
#endif
    std::lock_guard<std::mutex> lock(mutex_);
    image_ = msg;
    dirty_ = true;
  }

  // TODO(lucasw) factor this into a generic opengl function to put in parent class
  // if the image changes need to call this
  bool RosImage::updateTexture() {
    sensor_msgs::msg::Image::SharedPtr image;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!dirty_)
        return true;
      dirty_ = false;

      if (!image_) {
        // TODO(lucasw) or make the texture 0x0 or 1x1 gray.
        return false;
      }

      image = image_;
      width_ = image->width;
      height_ = image->height;
    }

    #if 0
    if (texture_id_ != 0) {
      // if this has happened then probably a crash is going to happen,
      // the memory used to create the texture has been freed?
      ROS_ERROR_STREAM("can't update with non-zero texture_id " << texture_id_);
      return false;
    }
    #endif

    // RCLCPP_INFO("image update %d %d %d %d" << texture_id_ << " "
    //    << image->header.stamp << " "
    //    << image->data.size() << " "
    //    << image->width << " " << image->height);
    glBindTexture(GL_TEXTURE_2D, texture_id_);

    // TODO(lucasw) only need to do these once (unless altering)
    // TODO(lucasw) make these configurable live
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    // Set texture clamping method - GL_CLAMP isn't defined
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    // Copy the data to the graphics memory.
    // TODO(lucasw) actually look at the image encoding type and
    // have a big switch statement here
    // TODO(lucasw) if the old texture is the same width and height and number of channels
    // (and color format?) as the old one, use glTexSubImage2D
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                 image->width, image->height,
                 0, GL_BGR, GL_UNSIGNED_BYTE, &image->data[0]);


    // one or both of these are causing a crash
    // use fast 4-byte alignment (default anyway) if possible
    // glPixelStorei(GL_UNPACK_ALIGNMENT, (image->step & 3) ? 1 : 4);
    // set length of one complete row in data (doesn't need to equal image.cols)
    // glPixelStorei(GL_UNPACK_ROW_LENGTH, image->step / 1);  // image.elemSize()); TODO(lucasw)

    // ROS_INFO_STREAM(texture_id_ << " " << image.size());
    return true;
  }

  // TODO(lucasw) factor out common code
  void RosImage::draw() {
    // only updates if dirty
    updateTexture();
    ImGui::Begin(name_.c_str());
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (texture_id_ != 0) {
        std::stringstream ss;
        static int count = 0;
        ss << texture_id_ << " " << topic_ << " "  // << sub_.getTopic() << " "
            << width_ << " " << height_ << " " << count++;
        // const char* text = ss.str().c_str();
        std::string text = ss.str();
        ImGui::Text("%.*s", static_cast<int>(text.size()), text.data());
        ImGui::Image((void*)(intptr_t)texture_id_, ImVec2(width_, height_));
      }
    }
    ImGui::End();
  }

  CvImage::CvImage(const std::string name) : GlImage(name, "") {
  }

  // if the image changes need to call this
  bool CvImage::updateTexture() {
    if (!dirty_)
      return true;
    dirty_ = false;

    if (image_.empty()) {
      // TODO(lucasw) or make the texture 0x0 or 1x1 gray.
      return false;
    }

    glBindTexture(GL_TEXTURE_2D, texture_id_);

    // Do these know which texture to use because of the above bind?
    // TODO(lucasw) make these configurable live
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    // Set texture clamping method - GL_CLAMP isn't defined
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    // does this copy the data to the graphics memory?
    // TODO(lucasw) actually look at the image encoding type and
    // have a big switch statement here
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                 image_.cols, image_.rows,
                 0, GL_BGR, GL_UNSIGNED_BYTE, image_.ptr());
    // use fast 4-byte alignment (default anyway) if possible
    glPixelStorei(GL_UNPACK_ALIGNMENT, (image_.step & 3) ? 1 : 4);

    // set length of one complete row in data (doesn't need to equal image.cols)
    glPixelStorei(GL_UNPACK_ROW_LENGTH, image_.step / image_.elemSize());
    std::cout << texture_id_ << " " << image_.size() << "\n";
    return true;
  }

  void CvImage::draw() {
    // only updates if dirty
    updateTexture();
    // TODO(lucasw) another kind of dirty_ - don't redraw if image hasn't changed,
    // window hasn't changed?  How to detect need to redraw window?
    ImGui::Begin(name_.c_str());
    if (!image_.empty() && (texture_id_ != 0)) {
      std::stringstream ss;
      static int count = 0;
      ss << texture_id_ << " " << image_.size() << " " << count++;
      // const char* text = ss.str().c_str();
      std::string text = ss.str();
      ImGui::Text("%.*s", static_cast<int>(text.size()), text.data());
      ImGui::Image((void*)(intptr_t)texture_id_, ImVec2(image_.cols, image_.rows));
    }
    ImGui::End();
  }
// }

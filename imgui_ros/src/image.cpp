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
#include "imgui_impl_sdl.h"
// #include <imgui_ros/AddWindow.h>
#include <imgui_ros/image.h>
#include <imgui_ros/imgui_impl_opengl3.h>
// #include <opencv2/highgui.hpp>
using std::placeholders::_1;

// namespace {
  GlImage::GlImage(const std::string name, const std::string topic) :
      Widget(name, topic) {
    glGenTextures(1, &texture_id_);
  }

  GlImage::~GlImage() {
    std::cout << "freeing texture " << texture_id_ << " " << name_ << "\n";
    glDeleteTextures(1, &texture_id_);
  }

  RosImage::RosImage(const std::string name, const std::string topic, const bool sub_not_pub,
                     std::shared_ptr<rclcpp::Node> node) : GlImage(name, topic), node_(node)
  {
    wrap_modes_.push_back(GL_CLAMP_TO_EDGE);
    wrap_modes_.push_back(GL_MIRRORED_REPEAT);
    wrap_modes_.push_back(GL_REPEAT);

    min_filter_modes_.push_back(GL_NEAREST);
    min_filter_modes_.push_back(GL_LINEAR);
    // TODO(lucasw) implement mipmaps
    // filter_modes_.push_back(GL_NEAREST_MIPMAP_NEAREST);
    // filter_modes_.push_back(GL_NEAREST_MIPMAP_LINEAR);
    // filter_modes_.push_back(GL_LINEAR_MIPMAP_NEAREST);
    // filter_modes_.push_back(GL_LINEAR_MIPMAP_LINEAR);

    mag_filter_modes_.push_back(GL_NEAREST);
    mag_filter_modes_.push_back(GL_LINEAR);

    if (topic != "") {
      if (sub_not_pub) {
        RCLCPP_DEBUG(node->get_logger(), "%s subscribing to topic '%s'",
            name.c_str(), topic.c_str());
        sub_ = node->create_subscription<sensor_msgs::msg::Image>(topic,
            std::bind(&RosImage::imageCallback, this, _1));
      } else {
        pub_ = node->create_publisher<sensor_msgs::msg::Image>(topic);
      }
    }
  }

  void RosImage::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
#if 0
    std::cout << "image callback "
        << msg->header.stamp.sec << " "
        << msg->header.stamp.nanosec << " "
        << msg->data.size() << " "
        << msg->width << " " << msg->height << ", "
        << topic_
        << ", ptr " << reinterpret_cast<unsigned long int>(&msg->data[0]) << " "
        << static_cast<unsigned int>(msg->data[0]) << "\n";
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

#if 0
    std::cout << "update texture " << texture_id_ << " "
        << image->header.stamp.sec << " "
        << image->header.stamp.nanosec << " "
        << image->data.size() << " "
        << image->width << " " << image->height
        << ", ptr " << reinterpret_cast<unsigned long int>(&image->data[0]) << " "
        << static_cast<unsigned int>(image->data[0]) << "\n";
#endif
    glBindTexture(GL_TEXTURE_2D, texture_id_);

    // TODO(lucasw) only need to do these once (unless altering)
    // TODO(lucasw) make these configurable live
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    // Set texture clamping method - GL_CLAMP isn't defined
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap_modes_[wrap_s_ind_]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap_modes_[wrap_t_ind_]);

    // Copy the data to the graphics memory.
    // TODO(lucasw) actually look at the image encoding type and
    // have a big switch statement here
    // TODO(lucasw) if the old texture is the same width and height and number of channels
    // (and color format?) as the old one, use glTexSubImage2D
    if (image->encoding == "mono8") {
      // TODO(lucasw) need to use a fragment shader to copy the red channel
      // to the blue and green - there is no longer a GL_LUMINANCE
      // https://stackoverflow.com/questions/680125/can-i-use-a-grayscale-image-with-the-opengl-glteximage2d-function
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RED,
                   image->width, image->height,
                   0, GL_RED, GL_UNSIGNED_BYTE, &image->data[0]);
    } else if (image->encoding == "bgr8") {
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                   image->width, image->height,
                   0, GL_BGR, GL_UNSIGNED_BYTE, &image->data[0]);
    } else if (image->encoding == "rgb8") {
       glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                   image->width, image->height,
                   0, GL_RGB, GL_UNSIGNED_BYTE, &image->data[0]);
    // TODO(lucasw) these don't work, come out all black:
    } else if (image->encoding == "bgra8") {
       glTexImage2D(GL_TEXTURE_2D, 0, GL_BGRA,
                   image->width, image->height,
                   0, GL_BGRA, GL_UNSIGNED_BYTE, &image->data[0]);
    } else if (image->encoding == "rgba8") {
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
          image->width, image->height,
          0, GL_RGBA, GL_UNSIGNED_BYTE, &image->data[0]);
    }

    // one or both of these are causing a crash
    // use fast 4-byte alignment (default anyway) if possible
    // glPixelStorei(GL_UNPACK_ALIGNMENT, (image->step & 3) ? 1 : 4);
    // set length of one complete row in data (doesn't need to equal image.cols)
    // glPixelStorei(GL_UNPACK_ROW_LENGTH, image->step / 1);  // image.elemSize()); TODO(lucasw)

    // ROS_INFO_STREAM(texture_id_ << " " << image.size());
    // std::cout << "update texture done " << texture_id_ << "\n";
    glBindTexture(GL_TEXTURE_2D, 0);
    return true;
  }

  void RosImage::publish() {
    // std::cout << name_ << " " << pub_ << " " << image_ << " " <<  texture_id_ << "\n";
    if (!pub_) {
      return;
    }
    if (!image_) {
      // TODO(lucasw) debug message
      return;
    }
    // TODO(lucasw) check to see if there are any subscribers?  If none return.

    // TODO(lucasw) lock image_
    glBindTexture(GL_TEXTURE_2D, texture_id_);
    if (image_->encoding == "bgr8") {
      // glGetTextureImage, glGetnTexImage not available
      glGetTexImage(GL_TEXTURE_2D, 0, GL_BGR,
                     GL_UNSIGNED_BYTE,
                     // image_->width * image_->height * 3,
                     &image_->data[0]);
    } else if (image_->encoding == "rgb8") {
      glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB,
                     GL_UNSIGNED_BYTE,
                     // image_->width * image_->height * 3,
                     &image_->data[0]);
    } else {
      glBindTexture(GL_TEXTURE_2D, 0);
      // TODO(lucasw) set debug message to this
      std::cerr << name_ << " unsupported image type: " << image_->encoding << "\n";
      return;
    }
    glBindTexture(GL_TEXTURE_2D, 0);

    {
      auto node = node_.lock();
      if (node) {
        image_->header.stamp = node->now();
      }
    }
    pub_->publish(image_);
  }

  // TODO(lucasw) factor out common code
  void RosImage::draw() {
    // only updates if dirty
    updateTexture();
    {
      std::lock_guard<std::mutex> lock(mutex_);

      std::string name = name_ + " texture";
      // const std::string checkbox_text = "info##" + name;
      // ImGui::Checkbox(checkbox_text.c_str(), &enable_info_);
      if (enable_info_) {
        std::stringstream ss;
        ss << name_ << " " << texture_id_ << " " << topic_ << " "
            << width_ << " " << height_;  // << " " << count++;
        // const char* text = ss.str().c_str();
        // TODO(lucasw) write text value of 10-20 pixels values
        std::string text = ss.str();
        // std::cout << "draw " << text << "\n";
        ImGui::Text("%.*s", static_cast<int>(text.size()), text.data());
      }

      // Texture settings
      {
        // TODO(lucasw) set this up once
        std::string items_null;
        items_null += std::string("clamp_to_edge") + '\0';
        items_null += std::string("mirrored_repeat") + '\0';
        items_null += std::string("repeat") + '\0';
        const bool s_changed = ImGui::Combo(("wrap_s##" + name).c_str(), &wrap_s_ind_,
            items_null.c_str());
        const bool t_changed = ImGui::Combo(("wrap_t##" + name).c_str(), &wrap_t_ind_,
            items_null.c_str());
        if (s_changed || t_changed) {
          glBindTexture(GL_TEXTURE_2D, texture_id_);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap_modes_[wrap_s_ind_]);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap_modes_[wrap_t_ind_]);
          glBindTexture(GL_TEXTURE_2D, 0);
        }

        // TODO(lucasw) set this up once
        items_null = "";
        items_null += std::string("nearest") + '\0';
        items_null += std::string("linear") + '\0';
        const bool min_changed = ImGui::Combo(("min filter##" + name).c_str(), &min_filter_ind_,
            items_null.c_str());
        const bool mag_changed = ImGui::Combo(("mag filter##" + name).c_str(), &mag_filter_ind_,
            items_null.c_str());
        if (min_changed || mag_changed) {
          glBindTexture(GL_TEXTURE_2D, texture_id_);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, min_filter_modes_[min_filter_ind_]);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, mag_filter_modes_[mag_filter_ind_]);
          glBindTexture(GL_TEXTURE_2D, 0);
        }
      }

      const std::string checkbox_text2 = "show image##" + name;
      const std::string one_one_checkbox_text2 = "1:1##" + name;

      ImGui::Columns(2);
      ImGui::Checkbox(checkbox_text2.c_str(), &enable_draw_image_);
      ImGui::NextColumn();
      ImGui::Checkbox(one_one_checkbox_text2.c_str(), &enable_one_to_one_);
      ImGui::Columns(1);

      if ((enable_draw_image_) && (texture_id_ != 0) && (width_ != 0) && (height_ != 0)) {
        ImVec2 image_size;
        ImVec2 win_size = ImGui::GetWindowSize();
        const double fr_x = win_size.x / width_;
        const double fr_y = win_size.y / height_;
        double fr = fr_x;
        if (fr_x > fr_y) {
          fr = fr_y;
        }

        if (enable_one_to_one_) {
          image_size.x = width_;
          image_size.y = height_;
        } else {
          // TODO(lucasw) get this vertical offset from somewhere
          // is it the height of the title bar?
          image_size.y = height_ * fr - 15;
          // have to get a new scale factor because of the manual offset
          image_size.x = width_ * (image_size.y / height_);
          const float y = ImGui::GetCursorPosY();
          ImGui::SetCursorPos(ImVec2((win_size.x - image_size.x) * 0.5f,
               y));
               // y + (win_size.y - image_size.y) * 0.5f));
        }

        ImGui::Image((void*)(intptr_t)texture_id_, image_size);
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////
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
      ImVec2 win_size = ImGui::GetWindowSize();
      // std::cout << win_size.x << " " << win_size.y << "\n";
      // ImVec2 win_size = ImVec2(image_.cols, image_.rows);
      ImGui::Image((void*)(intptr_t)texture_id_, win_size);
    }
    ImGui::End();
  }
// }

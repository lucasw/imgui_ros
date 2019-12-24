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

#include <cv_bridge/cv_bridge.h>
#include <imgui.h>
#include <imgui_impl_sdl.h>
// #include <imgui_ros/AddWindow.h>
#include <imgui_ros/image.h>
#include <imgui_ros/imgui_impl_opengl3.h>
#include <opencv2/highgui.hpp>
using std::placeholders::_1;

namespace imgui_ros
{

// TODO(lucasw) put into utility file, maybe separate library later, even separate repo
bool glTexFromMat(cv::Mat& image, GLuint& texture_id)
{
  if (image.empty()) {
    // TODO(lucasw) or make the texture 0x0 or 1x1 gray.
    return false;
  }

  glBindTexture(GL_TEXTURE_2D, texture_id);

  // Do these know which texture to use because of the above bind?
  // TODO(lucasw) make these configurable live
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

  // Set texture clamping method - GL_CLAMP isn't defined
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  // use fast 4-byte alignment (default anyway) if possible
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  // glPixelStorei(GL_UNPACK_ALIGNMENT, (image.step & 3) ? 1 : 4);

  // copy the data to the graphics memory
  // TODO(lucasw) actually look at the image encoding type and
  // have a big switch statement here
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
               image.cols, image.rows,
               0, GL_BGR, GL_UNSIGNED_BYTE, image.ptr());

  // set length of one complete row in data (doesn't need to equal image.cols)
  // glPixelStorei(GL_UNPACK_ROW_LENGTH, image.step / image_.elemSize());

  glBindTexture(GL_TEXTURE_2D, 0);

  return true;
}

  GlImage::GlImage(const std::string name, const std::string topic) :
      Widget(name, topic) {
    glGenTextures(1, &texture_id_);
    ROS_INFO_STREAM("generating texture " << texture_id_ << " '" << name_);
  }

  GlImage::~GlImage() {
    ROS_INFO_STREAM("freeing texture " << texture_id_ << " '" << name_ << "'");
    glDeleteTextures(1, &texture_id_);
  }

  RosImage::RosImage(const std::string name,
                     std::shared_ptr<ImageTransfer> image_transfer,
                     const std::string topic, const bool sub_not_pub,
                     const bool ros_pub) :
                     GlImage(name, topic),
                     sub_not_pub_(sub_not_pub),
                     image_transfer_(image_transfer)
  {
    if (!image_transfer_) {
      throw std::runtime_error("uninitialized image transfer");
    }
    wrap_modes_.push_back(GL_CLAMP_TO_EDGE);
    wrap_modes_.push_back(GL_MIRRORED_REPEAT);
    wrap_modes_.push_back(GL_REPEAT);

    min_filter_modes_.push_back(GL_NEAREST);
    min_filter_modes_.push_back(GL_LINEAR);
    min_filter_modes_.push_back(GL_NEAREST_MIPMAP_NEAREST);
    min_filter_modes_.push_back(GL_NEAREST_MIPMAP_LINEAR);
    min_filter_modes_.push_back(GL_LINEAR_MIPMAP_NEAREST);
    min_filter_modes_.push_back(GL_LINEAR_MIPMAP_LINEAR);

    mag_filter_modes_.push_back(GL_NEAREST);
    mag_filter_modes_.push_back(GL_LINEAR);

    if (topic != "") {
      if (sub_not_pub) {
      #if 0
        RCLCPP_DEBUG(node->get_logger(), "%s subscribing to topic '%s'",
            name.c_str(), topic.c_str());
        sub_ = node->create_subscription<sensor_msgs::Image>(topic,
            std::bind(&RosImage::imageCallback, this, _1));
      #endif
      } else {
        (void)ros_pub;
        // pub_ = node->create_publisher<sensor_msgs::Image>(topic);
        image_transfer_->setRosPub(topic_);  // , ros_pub);
      }
    }
  }

  RosImage::RosImage(const std::string& name,
    std::shared_ptr<ImageTransfer> image_transfer,
    sensor_msgs::ImagePtr image) :
    RosImage(name, image_transfer)
  {
    image_msg_ = image;
    dirty_ = true;
    pub_dirty_ = true;
  }

#if 0
  void RosImage::imageCallback(const sensor_msgs::ImagePtr msg) {
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
#endif

  void RosImage::update(const ros::Time& stamp, const std::string dropped_file)
  {
    (void)stamp;
    // stamp_ = stamp;
    if (is_focused_ && (dropped_file != "") && !sub_not_pub_) {
      // ImGui::Text("dropped file: %s", dropped_file.c_str());
      ROS_INFO_STREAM("'" << name_ << " dropped file " << dropped_file);
      cv_bridge::CvImage cv_image;
      cv_image.image = cv::imread(dropped_file, cv::IMREAD_COLOR);
      if (cv_image.image.empty()) {
        ROS_ERROR_STREAM("Could not load image '" + dropped_file + "'");
      } else {
        // TODO(lucasw) convert cv::Mat to image msg so it can be published
        // image_ = image;
        loaded_file_ = dropped_file;
        if (glTexFromMat(cv_image.image, texture_id_)) {
          ROS_INFO_STREAM("loaded image " << cv_image.image.cols
              << " x " << cv_image.image.rows);
          sensor_msgs::ImagePtr image_msg;
          image_msg = cv_image.toImageMsg();
          image_msg->header.frame_id = header_frame_id_;
          image_msg->header.stamp = ros::Time::now();
          // TODO(lucasw) why doesn't cv_bridge fill this in?
          image_msg->encoding = "bgr8";
          image_msg_ = image_msg;
          width_ = image_msg->width;
          height_ = image_msg->height;
          image_transfer_->publish(topic_, image_msg_);
        }
      }
    }
  }  // update

  // Transfer image data from cpu to gpu
  // TODO(lucasw) factor this into a generic opengl function to put in parent class
  // if the image changes need to call this
  bool RosImage::updateTexture() {
    // ImGui::Separator();
    auto t0 = ros::Time::now();
    // ImGui::Text("%s:", name_.c_str());
    if (!enable_cpu_to_gpu_) {
      // ImGui::Text(" no cpu to gpu");
      return true;
    }
    sensor_msgs::ImagePtr image_msg;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (sub_not_pub_) {
        // ImGui::Text(" sub not pub");
        if (!image_transfer_) {
          // ImGui::Text("  no image transfer");
          return false;
        }
        image_transfer_->getSub(topic_, image_msg);
        if (!image_msg) {
          // ImGui::Text("  no image msg");
          return false;
        }
        bool different_image = (image_msg != image_msg_);
        dirty_ |= different_image;
        if (image_msg_ && image_msg && different_image) {
          image_gap_ = ros::Time(image_msg->header.stamp) - ros::Time(image_msg_->header.stamp);
        }
        image_msg_ = image_msg;
      }
      // TODO(lucasw) updateTexture does nothing if this isn't a subscriber,
      // except the first time it is run?
      if (!dirty_) {
        // ImGui::Text(" not dirty");
        return true;
      }
      dirty_ = false;

      if (!image_msg_) {
        // TODO(lucasw) or make the texture 0x0 or 1x1 gray.
        // ImGui::Text("no image msg");
        return false;
      }

      image_msg = image_msg_;
      width_ = image_msg->width;
      height_ = image_msg->height;
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
    // TODO(lucasw) put ImagePtr to opengl texture into a utility library function
    glBindTexture(GL_TEXTURE_2D, texture_id_);
    // ImGui::Text("%d %s", texture_id_, image_msg->encoding.c_str());

    // TODO(lucasw) only need to do these once (unless altering)
    // TODO(lucasw) make these configurable live
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, min_filter_modes_[min_filter_ind_]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, mag_filter_modes_[mag_filter_ind_]);

    // Set texture clamping method - GL_CLAMP isn't defined
    if (wrap_s_ind_ >= static_cast<int>(wrap_modes_.size()))
      wrap_s_ind_ = wrap_modes_.size() - 1;
    else if (wrap_s_ind_ < 0)
      wrap_s_ind_ = 0;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrap_modes_[wrap_s_ind_]);

    if (wrap_t_ind_ >= static_cast<int>(wrap_modes_.size()))
      wrap_t_ind_ = wrap_modes_.size() - 1;
    else if (wrap_t_ind_ < 0)
      wrap_t_ind_ = 0;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrap_modes_[wrap_t_ind_]);

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);  // (image->step & 3) ? 1 : 4);

    // Copy the data to the graphics memory.
    // TODO(lucasw) actually look at the image encoding type and
    // have a big switch statement here
    // TODO(lucasw) if the old texture is the same width and height and number of channels
    // (and color format?) as the old one, use glTexSubImage2D
    if (image_msg->encoding == "mono8") {
      // TODO(lucasw) need to use a fragment shader to copy the red channel
      // to the blue and green - there is no longer a GL_LUMINANCE
      // https://stackoverflow.com/questions/680125/can-i-use-a-grayscale-image-with-the-opengl-glteximage2d-function
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RED,
                   image_msg->width, image_msg->height,
                   0, GL_RED, GL_UNSIGNED_BYTE, &image_msg->data[0]);
    } else if (image_msg->encoding == "bgr8") {
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                   image_msg->width, image_msg->height,
                   0, GL_BGR, GL_UNSIGNED_BYTE, &image_msg->data[0]);
    } else if (image_msg->encoding == "rgb8") {
       glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                   image_msg->width, image_msg->height,
                   0, GL_RGB, GL_UNSIGNED_BYTE, &image_msg->data[0]);
    } else if (image_msg->encoding == "bgra8") {
       glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                   image_msg->width, image_msg->height,
                   0, GL_BGRA, GL_UNSIGNED_BYTE, &image_msg->data[0]);
    } else if (image_msg->encoding == "rgba8") {
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
          image_msg->width, image_msg->height,
          0, GL_RGBA, GL_UNSIGNED_BYTE, &image_msg->data[0]);
    } else {
      // TODO(lucasw) throw
    }
    glGenerateMipmap(GL_TEXTURE_2D);

    // one or both of these are causing a crash
    // use fast 4-byte alignment (default anyway) if possible
    // glPixelStorei(GL_UNPACK_ALIGNMENT, (image->step & 3) ? 1 : 4);
    // set length of one complete row in data (doesn't need to equal image.cols)
    // glPixelStorei(GL_UNPACK_ROW_LENGTH, image->step / 1);  // image.elemSize()); TODO(lucasw)

    // ROS_INFO_STREAM(texture_id_ << " " << image.size());
    // std::cout << "update texture done " << texture_id_ << "\n";
    glBindTexture(GL_TEXTURE_2D, 0);
    auto cur = ros::Time::now();
    update_duration_ = cur - t0;
    // The age when updated, not age for every draw after of the same data
    image_age_ = cur - image_msg_->header.stamp;
    return true;
  }

  void RosImage::publish(const ros::Time& stamp) {
    // std::cout << name_ << " " << pub_ << " " << image_ << " " <<  texture_id_ << "\n";
    if (!enable_publish_) {
      return;
    }
    if (!pub_dirty_) {
      return;
    }
    pub_dirty_ = false;

    if (!image_transfer_) {
      std::cerr << "No image transfer object\n";
      return;
    }

    auto image_msg = boost::make_shared<sensor_msgs::Image>();
    {
      // Need ability to report a different frame than the sim is using internally-
      // this allows for calibration error simulation
      image_msg->header.frame_id = header_frame_id_;
      image_msg->width = width_;
      image_msg->height = height_;
      image_msg->encoding = "bgr8";
      image_msg->step = width_ * 3;
      // TODO(lucasw) this step may be expensive
      image_msg->data.resize(image_msg->step * height_);
    }

    // TODO(lucasw) check to see if there are any subscribers?  If none return.

    // Copy image from gpu for sending- but don't do this if the image hasn't changed
    // TODO(lucasw) lock image
    glBindTexture(GL_TEXTURE_2D, texture_id_);
    if (image_msg->encoding == "bgr8") {
      // glGetTextureImage, glGetnTexImage not available
      glGetTexImage(GL_TEXTURE_2D, 0, GL_BGR,
                     GL_UNSIGNED_BYTE,
                     // image_msg->width * image_msg->height * 3,
                     &image_msg->data[0]);
    } else if (image_msg->encoding == "rgb8") {
      glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB,
                     GL_UNSIGNED_BYTE,
                     // image_msg->width * image_msg->height * 3,
                     &image_msg->data[0]);
    } else {
      glBindTexture(GL_TEXTURE_2D, 0);
      // TODO(lucasw) set debug message to this
      std::cerr << name_ << " unsupported image type: " << image_msg->encoding << "\n";
      return;
    }
    glBindTexture(GL_TEXTURE_2D, 0);

    image_msg->header.stamp = stamp;
    // pub_->publish(image_);
    image_transfer_->publish(topic_, image_msg);
    image_msg_ = image_msg;
  }

  // TODO(lucasw) factor out common code
  void RosImage::draw() {
    // ImGui::Text("RosImage %s", name_.c_str());
    is_focused_ = ImGui::IsWindowFocused();
    // only updates if dirty
    updateTexture();
    {
      std::lock_guard<std::mutex> lock(mutex_);

      std::string name = name_ + " texture";

      // auto t0 = clock_->now();
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
      // auto draw_duration = clock_->now() - t0;

      const std::string checkbox_text = "info##" + name;
      ImGui::Checkbox(checkbox_text.c_str(), &enable_info_);
      if (enable_info_) {
        #if 0
        std::stringstream ss;
        ss << name_ << " " << texture_id_ << " " << topic_ << " "
            << width_ << " " << height_;  // << " " << count++;
        std::string text = ss.str();
        ImGui::Text("%s %d %s %d %d", static_cast<int>(text.size()), text.data());
        #else
        ImGui::Text("%s %d %lu %lu", name_.c_str(), texture_id_,
            width_, height_);
        ImGui::Text("topic: '%s',\nloaded file: '%s'", topic_.c_str(), loaded_file_.c_str());
        #endif
        if (image_msg_) {
          ImGui::Columns(2);
          ImGui::Text("%0.3f",
              image_msg_->header.stamp.toSec());
          ImGui::NextColumn();
          ImGui::Text("age %0.5f", image_age_.toSec());
          ImGui::NextColumn();
          ImGui::Text("gap %0.5f", image_gap_.toSec() / 1e9);
          ImGui::NextColumn();
          ImGui::Text("tex update %0.5f", update_duration_.toSec());
              // draw_duration.nanoseconds() / 1e6);
          ImGui::Columns(1);
        }

        ImGui::Columns(2);
        const std::string show_image_text = "show##" + name;
        ImGui::Checkbox(show_image_text.c_str(), &enable_draw_image_);
        ImGui::NextColumn();
        const std::string cpu_to_gpu_text = "cpu->gpu##" + name;
        ImGui::Checkbox(cpu_to_gpu_text.c_str(), &enable_cpu_to_gpu_);
        ImGui::NextColumn();
        if (!sub_not_pub_) {
          const std::string show_image_text = "publish##" + name;
          ImGui::Checkbox(show_image_text.c_str(), &enable_publish_);
        }
        ImGui::NextColumn();
        const std::string one_one_checkbox_text2 = "1:1##" + name;
        ImGui::Checkbox(one_one_checkbox_text2.c_str(), &enable_one_to_one_);
        ImGui::NextColumn();
        ImGui::Columns(1);
      }

      // Texture settings
      if (draw_texture_controls_) {
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
        items_null += std::string("nearest_mipmap_nearest") + '\0';
        items_null += std::string("nearest_mipmap_linear") + '\0';
        items_null += std::string("linear_mipmap_nearest") + '\0';
        items_null += std::string("linear_mipmap_linear") + '\0';
        const bool min_changed = ImGui::Combo(("min filter##" + name).c_str(), &min_filter_ind_,
            items_null.c_str());
        items_null = "";
        items_null += std::string("nearest") + '\0';
        items_null += std::string("linear") + '\0';
        const bool mag_changed = ImGui::Combo(("mag filter##" + name).c_str(), &mag_filter_ind_,
            items_null.c_str());
        if (min_changed || mag_changed) {
          glBindTexture(GL_TEXTURE_2D, texture_id_);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, min_filter_modes_[min_filter_ind_]);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, mag_filter_modes_[mag_filter_ind_]);
          glBindTexture(GL_TEXTURE_2D, 0);
        }
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
    ROS_INFO_STREAM(texture_id_ << " " << image_.size());
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
}  // namespace imgui_ros

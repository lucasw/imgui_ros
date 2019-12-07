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
 * SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <imgui.h>
// #include "imgui_impl_sdl.h"
#include <imgui_ros/camera.h>
#include <imgui_ros/utility.h>
#include <iomanip>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
using std::placeholders::_1;
using std::placeholders::_2;

namespace imgui_ros
{
// TODO(lucasw) 'Camera' -> 'TextureCamera'
// This renders a view from a given tf frame in the viz3d world
// a copies it to a texture (which then can be separately published as a
// ROS Image, or used as a texture on 3d objects.
Camera::Camera(const std::string name,
    const std::string frame_id,
    const std::string header_frame_id,
    const double aov_y,
    const double aov_x,
    ros::NodeHandle* nh) :
    name_(name),
    frame_id_(frame_id),
    header_frame_id_(header_frame_id),
    aov_y_(aov_y),
    aov_x_(aov_x)
{
  if (header_frame_id_ == "") {
    header_frame_id_ = frame_id_;
  }
  ROS_INFO("creating camera %s, frame %s, aov y %0.1f, x %0.1f",
      name.c_str(), frame_id_.c_str(), aov_y, aov_x);
}

void Camera::init(const size_t width, const size_t height,
    const std::string& texture_name, const std::string& topic,
    const bool ros_pub,
    ros::NodeHandle* nh,
    std::shared_ptr<ImageTransfer> image_transfer)
{
  ROS_DEBUG("regular camera");
  const bool sub_not_pub = false;
  image_ = std::make_shared<RosImage>(texture_name, topic, sub_not_pub, ros_pub,
      image_transfer);
  {
    // node is bad
    // ROS_INFO("creating camera %s %d %d", name, width, height);
    image_->width_ = width;
    image_->height_ = height;
    image_->header_frame_id_ = header_frame_id_;

    // TODO(lucasw) is this needed here if same thing is being done in RosImage::publish?
    #if 0
    image_->image_ = std::make_shared<sensor_msgs::Image>();
    // Need ability to report a different frame than the sim is using internally-
    // this allows for calibration error simulation
    image_->image_->header.frame_id = header_frame_id_;
    image_->image_->width = width;
    image_->image_->height = height;
    image_->image_->encoding = "bgr8";
    image_->image_->step = width * 3;
    image_->image_->data.resize(width * height * 3);
    #endif

    image_->min_filter_ind_ = 0;
    image_->mag_filter_ind_ = 0;
  }

  {
    cv::Mat tmp(cv::Size(image_->width_, image_->height_), CV_8UC4, cv::Scalar(100, 50, 20, 255));
    glGenTextures(1, &image_->texture_id_);
    glBindTexture(GL_TEXTURE_2D, image_->texture_id_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image_->width_, image_->height_, 0, GL_RGBA,
        GL_UNSIGNED_BYTE, &tmp.data[0]);
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    // unbind - TODO(lucasw) needed?
    glBindTexture(GL_TEXTURE_2D, 0);
  }

  {
    glGenRenderbuffers(1, &depth_buffer_);
    glBindRenderbuffer(GL_RENDERBUFFER, depth_buffer_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT,
        image_->width_, image_->height_);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
  }

  {
    glGenFramebuffers(1, &frame_buffer_);
    glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
        GL_TEXTURE_2D, image_->texture_id_, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
        GL_RENDERBUFFER, depth_buffer_);

    glDrawBuffers(1, DrawBuffers);
    // OpenGL 4?
    // glNamedFramebufferDrawBuffers(frame_buffer_, 1, DrawBuffers);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
      std::stringstream ss;
      ss << name_ << " framebuffer is not complete " << glGetError();
      throw std::runtime_error(ss.str());
    } else {
      ROS_INFO("camera '%s' framebuffer setup complete, fb %d, depth %d, tex id %d",
          name_.c_str(), frame_buffer_, depth_buffer_, image_->texture_id_);
    }

    // restore default frame buffer
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
  }

  std::string msg;
  if (checkGLError2(msg)) {
    std::cerr << msg << std::endl;
    throw std::runtime_error(msg);
  }

  camera_info_pub_ = nh->advertise<sensor_msgs::CameraInfo>(topic + "/camera_info", 3);
}

Camera::~Camera()
{
  glDeleteRenderbuffers(1, &depth_buffer_);
  glDeleteFramebuffers(1, &frame_buffer_);
}

bool Camera::isReadyToRender()
{
  if (!enable_) {
    return false;
  }
  if (skip_count_ > skip_max_) {
    skip_count_ = 0;
  }
  if (skip_max_ == 0) {
    return true;
  }
  if (skip_count_ == 0) {
    return true;
  }
  return false;
}

void Camera::setFrameRate(const float target_frame_rate, const float update_rate)
{
  (void)target_frame_rate;
  (void)update_rate;
}

void Camera::publishCameraInfo(const ros::Time& stamp)
{
  if (!isReadyToRender()) {
    return;
  }
  sensor_msgs::CameraInfo camera_info_msg;
  camera_info_msg.header.frame_id = header_frame_id_;
  camera_info_msg.header.stamp = stamp;

  const int width = image_->width_;
  const int height = image_->height_;
  camera_info_msg.width = width;
  camera_info_msg.height = height;
  camera_info_msg.distortion_model = "plumb_bob";

  // TODO(lucasw) later provide parameters for this
  float aov_x = aov_x_;
  if (aov_x == 0.0)
    aov_x = aov_y_;
  const double fx = width * 0.5 / tan(aov_x * (M_PI / 180.0f) / 2.0);
  const double fy = height * 0.5 / tan(aov_y_ * (M_PI / 180.0f) / 2.0);
  camera_info_msg.K[0] = fx;
  camera_info_msg.K[2] = width * 0.5;
  camera_info_msg.K[4] = fy;
  camera_info_msg.K[5] = height * 0.5;
  camera_info_msg.K[8] = 1.0;
  camera_info_msg.r = {1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0,
                         0.0, 0.0, 1.0};
  camera_info_msg.p = {fx, 0.0, width * 0.5, 0.0,
                         0.0, fy, height * 0.5, 0.0,
                         0.0, 0.0, 1.0, 0.0};

  camera_info_pub_->publish(camera_info_msg);
}

void Camera::draw()
{
  std::string name = name_ + " camera";
  ImGui::PushID(name.c_str());
  // ImGui::Begin(name.c_str());
  // TODO(lucasw) later re-use code in RosImage
  ImGui::Checkbox("render to texture", &enable_);
  imgui_ros::inputText("frame id", frame_id_);
  imgui_ros::inputText("header frame id", header_frame_id_);
  double min = 1.0;
  double max = 170.0;
  ImGui::SliderScalar("aov y", ImGuiDataType_Double,
      &aov_y_, &min, &max, "%lf", 2);

  min = 0.0;
  ImGui::SliderScalar("aov x", ImGuiDataType_Double,
      &aov_x_, &min, &max, "%lf", 2);

  {
    double min = 0.01;
    double max = far_;
    ImGui::SliderScalar("near clip", ImGuiDataType_Double,
          &near_, &min, &max, "%lf", 3);
    min = near_;
    max = 1000.0;
    ImGui::SliderScalar("far clip", ImGuiDataType_Double,
          &far_, &min, &max, "%lf", 3);
  }

  {
    int skip = skip_max_;
    ImGui::SliderInt("skip", &skip, 0, 30);
    skip_max_ = skip;
  }

  if (enable_) {
    image_->draw();
  }
  ImGui::ColorEdit4("clear color", (float*)&clear_color_);

  // this does nothing if the image doesn't have a publisher set up
  // image_->publish();

  ImGui::PopID();
}

// Camera::render()
// {
// }
}  // namespace imgui_ros

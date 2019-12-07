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
#include <imgui_ros/cube_camera.h>
#include <iomanip>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
using std::placeholders::_1;
using std::placeholders::_2;

namespace imgui_ros
{
void drawGrid(cv::Mat& image, const size_t spacing=32)
{
  auto color = cv::Scalar(255, 255, 255, 255);
  for (int i = 0; i < image.rows; i += spacing) {
    cv::line(image, cv::Point(0, i), cv::Point(image.cols, i), color, 1);
  }
  for (int j = 0; j < image.cols; j += spacing) {
    cv::line(image, cv::Point(j, 0), cv::Point(j, image.rows), color, 1);
  }
}

// TODO(lucasw) 'CubeCamera' -> 'TextureCubeCamera'
// Similar to a regular texture Camera, this renders from a given tf frame
// onto six faces of a cube, and then complex projections can be done
// for panoramic or omnidirectional cameras, or arbitrary lense maps
// with a grid of normals facing in any direction.
CubeCamera::CubeCamera(const std::string& name,
    const std::string& frame_id,
    const std::string& header_frame_id,
    const float aov_y, const float aov_x,
    ros::NodeHandle* nh) :
    Camera(name, frame_id, header_frame_id, aov_y, aov_x, nh)
{
}

CubeCamera::~CubeCamera()
{
}

// TODO(lucasw) could put init back into constructor
void CubeCamera::init(
    const size_t width, const size_t height,
    const size_t face_width,
    const std::string& texture_name, const std::string& topic,
    const bool ros_pub,
    ros::NodeHandle* nh,
    std::shared_ptr<ImageTransfer> image_transfer)
{
  const bool sub_not_pub = false;

  ROS_INFO(
      "init cube camera '%s', texture '%s', topic '%s', faces %zu, %zu x %zu, %zu",
      name_.c_str(), texture_name.c_str(), topic.c_str(),
      faces_.size(), width, height, face_width);

  Camera::init(width, height, texture_name, topic, ros_pub, nh, image_transfer);

  glGenTextures(1, &cube_texture_id_);
  glBindTexture(GL_TEXTURE_CUBE_MAP, cube_texture_id_);
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_BASE_LEVEL, 0);
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAX_LEVEL, 0);
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

  checkGLError(__FILE__, __LINE__);

  std::cout << "cube texture id: " << cube_texture_id_ << "\n";

  // TODO(lucasw) maybe a cube camera should be 6 regular camera
  // or at least factor out common code.
  for (size_t i = 0; i < faces_.size(); ++i) {
    faces_[i] = std::make_shared<CubeFace>();
  }

  auto mat3x3 = tf2::Matrix3x3();
  faces_[0]->dir_ = GL_TEXTURE_CUBE_MAP_POSITIVE_X;
  mat3x3.setRPY(0, -M_PI * 0.5, 0);
  faces_[0]->transform_.setBasis(mat3x3);

  faces_[1]->dir_ = GL_TEXTURE_CUBE_MAP_NEGATIVE_X;
  mat3x3.setRPY(0, M_PI * 0.5, 0);
  faces_[1]->transform_.setBasis(mat3x3);

  faces_[2]->dir_ = GL_TEXTURE_CUBE_MAP_POSITIVE_Y;
  mat3x3.setRPY(M_PI * 0.5, 0, 0);
  faces_[2]->transform_.setBasis(mat3x3);

  faces_[3]->dir_ = GL_TEXTURE_CUBE_MAP_NEGATIVE_Y;
  mat3x3.setRPY(-M_PI * 0.5, 0, 0);
  faces_[3]->transform_.setBasis(mat3x3);

  faces_[4]->dir_ = GL_TEXTURE_CUBE_MAP_POSITIVE_Z;

  faces_[5]->dir_ = GL_TEXTURE_CUBE_MAP_NEGATIVE_Z;
  mat3x3.setRPY(M_PI, 0, M_PI);
  faces_[5]->transform_.setBasis(mat3x3);

  int ind = 0;
  for (auto face : faces_) {
    std::cout << "face dir " << face->dir_ << "\n";

    ind += 1;
    // TODO(lucasw) texture_name + std::to_string(face->dir_);
    std::shared_ptr<RosImage> image = std::make_shared<RosImage>(
        texture_name + "_face" + std::to_string(ind),
        "", sub_not_pub, false);
    image->min_filter_ind_ = 0;
    image->mag_filter_ind_ = 0;
    face->image_ = image;
    // TODO(lucasw) is this useful?  No don't do it, it'll be freed 6 times
    // (though that isn't that bad)
    // image->texture_id_ = cube_texture_id_;
    // TODO(lucasw) move into image class
    {
      // ROS_INFO("creating camera %s %d %d", name, width, height);
      image->width_ = face_width;
      image->height_ = face_width;

      image->image_ = boost::make_shared<sensor_msgs::Image>();
      // TODO(lucasw) there need to be frames for all directions of the cube
      image->image_->header.frame_id = frame_id_;
      image->image_->width = face_width;
      image->image_->height = face_width;
      image->image_->encoding = "bgr8";
      image->image_->step = face_width * 3;
      image->image_->data.resize(image->image_->height * image->image_->step);

      // allocate the image for later copying out of the cubemap
      glBindTexture(GL_TEXTURE_2D, image->texture_id_);
      glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_REPEAT);
      glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_REPEAT);
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, face_width, face_width,
          0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
      glGenerateMipmap(GL_TEXTURE_2D);
      std::cout << "data size " << image->image_->data.size() << "\n";
    }

    checkGLError(__FILE__, __LINE__);
    {
      // unique per face color
      auto color = cv::Scalar(255/6 * ind, 50 + ind * 10, 128 - ind * 5, 255);
      cv::Mat tmp(cv::Size(image->width_, image->height_), CV_8UC4, color);
      drawGrid(tmp, 32);
      // std::cout << "copying data to portion of cube map "
      //     << tmp.cols << " " << tmp.rows << " " << image->width_ << " " << image->height_
      //     << " " << int(tmp.data[0]) << "\n";
      glBindTexture(GL_TEXTURE_CUBE_MAP, cube_texture_id_);
      glTexImage2D(face->dir_, 0, GL_RGBA,
          image->width_, image->height_, 0, GL_RGBA,
          GL_UNSIGNED_BYTE, &tmp.data[0]);
      glGenerateMipmap(GL_TEXTURE_2D);
      // std::cout << "copied data to portion of cube map " << face->dir_ << "\n";
    }
    checkGLError(__FILE__, __LINE__);

    {
      glGenRenderbuffers(1, &face->depth_buffer_);
      glBindRenderbuffer(GL_RENDERBUFFER, face->depth_buffer_);
      glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT,
          image->width_, image->height_);
      glBindRenderbuffer(GL_RENDERBUFFER, 0);
      // std::cout << "depth buffer " << face->depth_buffer_ << "\n";
    }
    checkGLError(__FILE__, __LINE__);

    {
      glGenFramebuffers(1, &face->frame_buffer_);
      // std::cout << "frame buffer " << face->frame_buffer_ << "\n";
      glBindFramebuffer(GL_FRAMEBUFFER, face->frame_buffer_);
      glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
          face->dir_, cube_texture_id_, 0);
      glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
          GL_RENDERBUFFER, face->depth_buffer_);

      glDrawBuffer(GL_COLOR_ATTACHMENT0);
      // OpenGL 4?
      // glNamedFramebufferDrawBuffers(frame_buffer_, 1, DrawBuffers);

      checkGLError(__FILE__, __LINE__);

      auto fb_status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
      if (fb_status != GL_FRAMEBUFFER_COMPLETE) {
        ROS_ERROR(
            "framebuffer incomplete '%s', face dir %d, fb %d, db %d, cube tex id %d, fb status %d, gl error %X",
            name_.c_str(),
            face->dir_, face->frame_buffer_, face->depth_buffer_,
            cube_texture_id_,
            fb_status, glGetError());
        // TODO(lucasw) put above text into throw message
        throw std::runtime_error("incomplete frame buffer");
      } else {
        ROS_INFO("cube camera '%s' dir %d framebuffer setup complete, fb %d, depth %d, tex id %d",
            name_.c_str(), face->dir_,
            face->frame_buffer_, face->depth_buffer_, cube_texture_id_);
      }

      // restore default frame buffer
      glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }
  }  // end looping through faces

  glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
}

void CubeCamera::draw()
{
  Camera::draw();
  std::string name = name_ + " cube camera";
  // ImGui::Begin(name.c_str());
  // TODO(lucasw) later re-use code in RosImage
  // ImGui::Checkbox(("enable##" + name).c_str(), &enable_);


  glBindTexture(GL_TEXTURE_CUBE_MAP, cube_texture_id_);
  int width = faces_[0]->image_->width_;
  int height = faces_[0]->image_->height_;
  ImGui::Text("Face width %d, height %d", width, height);
  ImVec2 image_size;
  int miplevel = 0;
  #if 0
  // these come up zero
  glGetTexLevelParameteriv(GL_TEXTURE_2D, miplevel, GL_TEXTURE_WIDTH, &width);
  glGetTexLevelParameteriv(GL_TEXTURE_2D, miplevel, GL_TEXTURE_HEIGHT, &height);
  ImGui::Text("%d %d", width, height);
  glGetTexLevelParameteriv(GL_TEXTURE_CUBE_MAP, miplevel, GL_TEXTURE_WIDTH, &width);
  glGetTexLevelParameteriv(GL_TEXTURE_CUBE_MAP, miplevel, GL_TEXTURE_HEIGHT, &height);
  ImGui::Text("%d %d", width, height);
  #endif
  // this works
  for (auto face : faces_) {
    glGetTexLevelParameteriv(face->dir_, miplevel, GL_TEXTURE_WIDTH, &width);
    glGetTexLevelParameteriv(face->dir_, miplevel, GL_TEXTURE_HEIGHT, &height);
    ImGui::Text("face %d: %d x %d", face->dir_, width, height);

    image_size.x = width;
    image_size.y = height;
    // ImGui::Image((void*)(intptr_t)face->image_->texture_id_, image_size);
  }
  glBindTexture(GL_TEXTURE_CUBE_MAP, 0);

  image_size.x = width;
  image_size.y = height;

  // this doesn't work, need to copy data to a regular texture
  // ImGui::Image((void*)(intptr_t)cube_texture_id_, image_size);

  // ImGui::End();

  // this does nothing if the image doesn't have a publisher set up
  for (auto face : faces_) {
    // TODO(lucasw) how to publish the
    // image->publish();
  }
}

// CubeCamera::render()
// {
// }
}  // namespace imgui_ros

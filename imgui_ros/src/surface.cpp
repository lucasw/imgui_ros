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

#include <SDL2/SDL.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <imgui.h>
// #include "imgui_impl_sdl.h"
#include <imgui_ros/viz3d.h>
#include <iomanip>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
using std::placeholders::_1;
using std::placeholders::_2;

Shape::Shape(const std::string name, const std::string frame_id,
    const std::string texture_name,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer) :
    name_(name),
    frame_id_(frame_id),
    texture_(texture_name)
{
}

Shape::~Shape()
{
  glDeleteBuffers(1, &elements_handle_);
  glDeleteBuffers(1, &vbo_handle_);
  glDeleteVertexArrays(1, &vao_handle_);
}

void Shape::init()
{
  glGenVertexArrays(1, &vao_handle_);
  glBindVertexArray(vao_handle_);

  glGenBuffers(1, &vbo_handle_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_handle_);
  // copy data to gpu
  glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)vertices_.Size * sizeof(DrawVert),
      (const GLvoid*)vertices_.Data, GL_STREAM_DRAW);

  glGenBuffers(1, &elements_handle_);

  checkGLError(__FILE__, __LINE__);
  std::cout << name_ << " init vao " << vao_handle_ << ", "
      << "vbo " << vbo_handle_ << ", elements " << elements_handle_ << ", "
      << "vertices size " << vertices_.Size << ", "
      << "indices size " << indices_.Size << "\n";

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elements_handle_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)indices_.Size * sizeof(ImDrawIdx),
      (const GLvoid*)indices_.Data, GL_STREAM_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

#if 0
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
#endif
  checkGLError(__FILE__, __LINE__);
}

void Shape::draw()
{
  std::stringstream ss;
  ImGui::Checkbox((name_ + "##shape").c_str(), &enable_);
  // TODO(lucasw) put the frame in a combo box that allows it to be changed
  // to any other existing frame (or typed in?)
  // use tf buffer getFrames()
  ss << "frame: " << frame_id_ << ", texture: "  << texture_ << "\n";
  ss << " `- vertices: " << vertices_.size() << ", indices " << indices_.size() << "\n";
  ss << " `- vao: " << vao_handle_ << ", vbo " << vbo_handle_
    << ", elements " << elements_handle_ << "\n";
  // ss << msg_;
  // TODO(lucasw) add interactive way to browse vertices and indices
  // maybe highlight each graphically as it is selected
  ImGui::Text("%s", ss.str().c_str());
}

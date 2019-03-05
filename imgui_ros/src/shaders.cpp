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
#include <imgui_ros/shaders.h>
#include <iomanip>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
using std::placeholders::_1;
using std::placeholders::_2;

namespace imgui_ros
{
ShaderSet::~ShaderSet()
{
  remove();
}

void ShaderSet::remove()
{
  std::cout << "Deleting shader set " << name_ << " "
      << vert_handle_ << " " << frag_handle_ << " " << shader_handle_ << "\n";
  glDeleteShader(vert_handle_);
  glDeleteShader(frag_handle_);
  glDeleteProgram(shader_handle_);
}

bool ShaderSet::init(const std::string& glsl_version, std::string& message)
{
  message += "shader init: '" + name_ + "'\n";
  if ((vert_handle_ != 0) || (frag_handle_ != 0) || (shader_handle_ != 0)) {
    std::stringstream ss;
    ss << "already initialized, deleting old versions "
        << vert_handle_ << " " << frag_handle_ << " " << shader_handle_ << "\n";
    message += ss.str();
    remove();
  }
  // Create shaders
  const GLchar* vertex_shader_with_version[2] = {glsl_version.c_str(),
      vertex_code_.c_str()};
  vert_handle_ = glCreateShader(GL_VERTEX_SHADER);
  if (!vert_handle_)
  {
    message = "vertex shader failed to create " + glGetError();
    return false;
  }
  else
  {
    glShaderSource(vert_handle_, 2, vertex_shader_with_version, NULL);
    glCompileShader(vert_handle_);
    if (!CheckShader(vert_handle_, "vertex shader", message)) {
      // std::cout << vertex_code_ << std::endl;
      return false;
    }
  }

  const GLchar* fragment_shader_with_version[2] = {glsl_version.c_str(), fragment_code_.c_str() };
  frag_handle_ = glCreateShader(GL_FRAGMENT_SHADER);
  if (!frag_handle_)
  {
    message = "fragment shader failed to create " + glGetError();
    return false;
  }
  else
  {
    glShaderSource(frag_handle_, 2, fragment_shader_with_version, NULL);
    glCompileShader(frag_handle_);
    if (!CheckShader(frag_handle_, "fragment shader", message)) {
      // std::cout << fragment_code_ << std::endl;
      return false;
    }
  }

  shader_handle_ = glCreateProgram();
  glAttachShader(shader_handle_, vert_handle_);
  glAttachShader(shader_handle_, frag_handle_);
  glLinkProgram(shader_handle_);
  if (!CheckProgram(shader_handle_, "shader program", message)) {
    return false;
  }

  GLint size;
  GLenum type;
  const GLsizei buf_size = 64;
  GLchar name_char[buf_size];
  GLsizei length;

  GLint num_attribs = 0;
  glGetProgramiv(shader_handle_, GL_ACTIVE_ATTRIBUTES, &num_attribs);
  std::cout << "shader '" << name_ << "' has " << num_attribs << " attribs\n";

  for (int i = 0; i < num_attribs; ++i) {
    glGetActiveAttrib(shader_handle_, (GLuint)i, buf_size, &length, &size, &type, name_char);
    std::string name(name_char);
    attrib_locations_[name] = glGetAttribLocation(shader_handle_, name.c_str());
    std::cout << "  '" << name << "' attrib location: " << attrib_locations_[name]
        << ", size " << size << ", type " << type << "\n";
  }

  GLint num_uniforms = 0;
  glGetProgramiv(shader_handle_, GL_ACTIVE_UNIFORMS, &num_uniforms);
  std::cout << "shader '" << name_ << "' has " << num_uniforms << " uniforms\n";

  for (int i = 0; i < num_uniforms; ++i) {
    glGetActiveUniform(shader_handle_, (GLuint)i, buf_size, &length, &size, &type, name_char);
    std::string name(name_char);

    // removing [0] makes the uniforms work properly
    std::string remove_str = "[0]";
    size_t num = remove_str.size();
    if (name.size() >= num) {
      if (name.substr(name.size() - num) == "[0]") {
        name = name.substr(0, name.size() - num);
      }
    }

    uniform_locations_[name] = glGetUniformLocation(shader_handle_, name.c_str());
    // TODO(lucasw) ambient is -1, but still seems to work
    std::cout << "  '" << name << "' uniform location: " << uniform_locations_[name]
        << ", size " << size << ", type " << type << "\n";
  }

  std::string msg;
  if (checkGLError2(msg)) {
    return false;
  }

  return true;
}

void ShaderSet::draw()
{
  std::stringstream ss;
  ss << "shader: " << name_ << " " << shader_handle_ << ", vert " << vert_handle_
      << ", geometry " << geometry_handle_ << ", frag " << frag_handle_;
  ss << "\n";

  for (auto name_pair : attrib_locations_) {
    std::string name = name_pair.first;
    ss << name << ": " << attrib_locations_[name] << ", ";
  }

  if (false) {
    ss << "\n--------------------------";
    ss << vertex_code_ << "\n";
    ss << "--------------------------";
    ss << geometry_code_ << "\n";
    ss << "--------------------------";
    ss << fragment_code_ << "\n";
    ss << "--------------------------";
  }

  ImGui::Text("%s", ss.str().c_str());
}
}  // namespace imgui_ros

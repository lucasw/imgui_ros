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
// #include "imgui_impl_sdl.h"
#include <imgui_ros/viz3d.h>
using std::placeholders::_1;

// render the entire background
// this probably will be split out into a widget also.
Viz3D::Viz3D(const std::string name,
    std::shared_ptr<ImGuiImplOpenGL3> renderer) : name_(name),
    renderer_(renderer)
{
  glGenVertexArrays(1, &vao_handle_);
  glGenBuffers(1, &vertex_buffer_);
}

void Viz3D::render(const int fb_width, const int fb_height)
{
  if (fb_width <= 0 || fb_height <= 0)
      return;

#if 0
  glViewport(0, 0, (GLsizei)fb_width, (GLsizei)fb_height);
  glUseProgram(g_ShaderHandle);

  glBindVertexArray(vao_handle_);
  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(getVertexBuffer()),
      getVertexBuffer(), GL_STATIC_DRAW);
  glEnableVertexAttribArray(g_attrib_location_position_);
  glVertexAttribPointer(g_attrib_location_position_, 3, GL_FLOAT,
    GL_FALSE, 0, (void*)0);
  glDrawArrays(GL_TRIANGLES, 0, 3);
  glDisableVertexAttribArray(g_attrib_location_position_);
#endif
}

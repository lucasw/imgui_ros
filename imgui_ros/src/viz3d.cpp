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

#include "imgui.h"
// #include "imgui_impl_sdl.h"
#include <imgui_ros/viz3d.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using std::placeholders::_1;

// render the entire background
// this probably will be split out into a widget also.
Viz3D::Viz3D(const std::string name,
    std::shared_ptr<ImGuiImplOpenGL3> renderer) : name_(name),
    renderer_(renderer)
{
  glGenTextures(1, &texture_id_);
  std::cout << "viz3d texture id " << texture_id_ << "\n";
  test_ = cv::Mat(128, 128, CV_8UC3, cv::Scalar::all(128));
  cv::circle(test_, cv::Point(40, 40), 40, cv::Scalar(0, 255, 255, 0), -1);
  // cv::imshow("test", test_);
  // cv::waitKey(0);

  glBindTexture(GL_TEXTURE_2D, texture_id_);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  // Set texture clamping method - GL_CLAMP isn't defined
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  glPixelStorei(GL_UNPACK_ALIGNMENT, (test_.step & 3) ? 1 : 4);
  // set length of one complete row in data (doesn't need to equal image.cols)
  glPixelStorei(GL_UNPACK_ROW_LENGTH, test_.step / test_.elemSize());

  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, test_.cols, test_.rows,
      0, GL_RGB, GL_UNSIGNED_BYTE, &test_.data[0]);
  checkGLError(__FILE__, __LINE__);
}

Viz3D::~Viz3D()
{
  glDeleteTextures(1, &texture_id_);
}

void Viz3D::render(const int fb_width, const int fb_height,
  const int display_pos_x, const int display_pos_y,
  const int display_size_x, const int display_size_y)
{
  if (fb_width <= 0 || fb_height <= 0) {
    std::cerr << "bad width height " << fb_width << " " << fb_height << "\n";
    return;
  }

  std::shared_ptr<ImGuiImplOpenGL3> renderer = renderer_.lock();
  if (!renderer) {
    std::cerr << "no renderer\n";
    return;
  }
    checkGLError(__FILE__, __LINE__);

    GLState gl_state;
    gl_state.backup();

    // Setup render state: alpha-blending enabled, no face culling, no depth testing, scissor enabled, polygon fill
    glEnable(GL_BLEND);
    glBlendEquation(GL_FUNC_ADD);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    // TODO(lucasw) later enable
    glDisable(GL_CULL_FACE);
    // TODO(lucasw) later enable
    glDisable(GL_DEPTH_TEST);
    // Want to draw to whole window
    glDisable(GL_SCISSOR_TEST);
#ifdef GL_POLYGON_MODE
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
#endif
    checkGLError(__FILE__, __LINE__);

    // Setup viewport, orthographic projection matrix
    // Our visible imgui space lies from draw_data->DisplayPos (top left) to draw_data->DisplayPos+data_data->DisplaySize (bottom right). DisplayMin is typically (0,0) for single viewport apps.
    glViewport(0, 0, (GLsizei)fb_width, (GLsizei)fb_height);
    #if 1
    float L = display_pos_x;
    float R = display_pos_x + display_size_x;
    float T = display_pos_y;
    float B = display_pos_y + display_size_y;
    // later do non-ortho projection
    const float ortho_projection[4][4] =
    {
        { 2.0f/(R-L),   0.0f,         0.0f,   0.0f },
        { 0.0f,         2.0f/(T-B),   0.0f,   0.0f },
        { 0.0f,         0.0f,        -1.0f,   0.0f },
        { (R+L)/(L-R),  (T+B)/(B-T),  0.0f,   1.0f },
    };
    #else
    const float ortho_projection[4][4] =
    {
        { 1.0f,   0.0f,   0.0f,   0.0f },
        { 0.0f,   1.0f,   0.0f,   0.0f },
        { 0.0f,   0.0f,   1.0f,   0.0f },
        { 0.0f,   0.0f,   0.0f,   1.0f },
    };
    #endif
    glUseProgram(renderer->shader_handle_);
    glUniform1i(renderer->attrib_location_tex_, 0);
    glUniformMatrix4fv(renderer->attrib_location_proj_mtx_, 1, GL_FALSE, &ortho_projection[0][0]);
#ifdef GL_SAMPLER_BINDING
    glBindSampler(0, 0);
    // We use combined texture/sampler state. Applications using GL 3.3 may set that otherwise.
#endif
    // Recreate the VAO every time
    // (This is to easily allow multiple GL contexts. VAO are not shared among GL contexts, and we don't track creation/deletion of windows so we don't have an obvious key to use to cache them.)
    checkGLError(__FILE__, __LINE__);
    GLuint vao_handle = 0;
    glGenVertexArrays(1, &vao_handle);
    glBindVertexArray(vao_handle);
    glBindBuffer(GL_ARRAY_BUFFER, renderer->vbo_handle_);
    checkGLError(__FILE__, __LINE__);
    glEnableVertexAttribArray(renderer->attrib_location_position_);
    glEnableVertexAttribArray(renderer->attrib_location_uv_);
    glEnableVertexAttribArray(renderer->attrib_location_color_);
    checkGLError(__FILE__, __LINE__);
    // This is for 2D data, need to be 3D
    glVertexAttribPointer(renderer->attrib_location_position_, 2, GL_FLOAT, GL_FALSE,
        sizeof(ImDrawVert), (GLvoid*)IM_OFFSETOF(ImDrawVert, pos));
    glVertexAttribPointer(renderer->attrib_location_uv_, 2, GL_FLOAT, GL_FALSE,
        sizeof(ImDrawVert), (GLvoid*)IM_OFFSETOF(ImDrawVert, uv));
    glVertexAttribPointer(renderer->attrib_location_color_, 4, GL_UNSIGNED_BYTE, GL_TRUE,
        sizeof(ImDrawVert), (GLvoid*)IM_OFFSETOF(ImDrawVert, col));

    {
      ImVector<ImDrawVert> VtxBuffer;
      const float sc = 50.0;
      const float off_y = 150.0;
      const int num = 16;
      for (int i = 0; i < num; ++i) {
        float uv_x = float(i) / float(num);
        {
          ImDrawVert p1;
          p1.pos.x = i * sc;
          p1.pos.y = off_y;
          p1.uv.x = uv_x;
          p1.uv.y = 0;
          // These colors multiply with the texture color
          p1.col = IM_COL32(255, 255, 255, 255);
          VtxBuffer.push_back(p1);
        }
        {
          ImDrawVert p2;
          p2.pos.x = i * sc;
          p2.pos.y = off_y + sc;
          p2.uv.x = uv_x;
          p2.uv.y = 1.0;
          p2.col = IM_COL32(255, 255, 255, 255);
          VtxBuffer.push_back(p2);
        }
      }

      ImVector<ImDrawIdx> IdxBuffer;
      for (int i = 0; i < VtxBuffer.Size - 3; i += 2) {
        IdxBuffer.push_back(i);
        IdxBuffer.push_back(i + 3);
        IdxBuffer.push_back(i + 2);
      }

      ImVector<ImDrawCmd> CmdBuffer;
      ImDrawCmd cmd;
      cmd.ElemCount = IdxBuffer.Size;
      CmdBuffer.push_back(cmd);

      checkGLError(__FILE__, __LINE__);
      const ImDrawIdx* idx_buffer_offset = 0;

      glBindBuffer(GL_ARRAY_BUFFER, renderer->vbo_handle_);
      glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)VtxBuffer.Size * sizeof(ImDrawVert),
          (const GLvoid*)VtxBuffer.Data, GL_STREAM_DRAW);

      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, renderer->elements_handle_);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)IdxBuffer.Size * sizeof(ImDrawIdx),
          (const GLvoid*)IdxBuffer.Data, GL_STREAM_DRAW);
      checkGLError(__FILE__, __LINE__);

      ImVec4 clip_rect = ImVec4(0, 0, fb_width, fb_height);
      glScissor((int)clip_rect.x, (int)(fb_height - clip_rect.w),
          (int)(clip_rect.z - clip_rect.x), (int)(clip_rect.w - clip_rect.y));

      for (int cmd_i = 0; cmd_i < CmdBuffer.Size; cmd_i++)
      {
        const ImDrawCmd* pcmd = &CmdBuffer[cmd_i];
        {
          // Bind texture- if it is null then the color is black
          // if (texture_id_ != nullptr)
          glBindTexture(GL_TEXTURE_2D, (GLuint)(intptr_t)texture_id_);
          glDrawElements(GL_TRIANGLES, (GLsizei)pcmd->ElemCount,
              sizeof(ImDrawIdx) == 2 ? GL_UNSIGNED_SHORT : GL_UNSIGNED_INT, idx_buffer_offset);
          // std::cout << cmd_i << " " << tex_id << " " << idx_buffer_offset << "\n";
          checkGLError(__FILE__, __LINE__);
        }
        idx_buffer_offset += pcmd->ElemCount;
      }
    }  // test draw

    glDeleteVertexArrays(1, &vao_handle);

    checkGLError(__FILE__, __LINE__);
    gl_state.backup();
    checkGLError(__FILE__, __LINE__);
}

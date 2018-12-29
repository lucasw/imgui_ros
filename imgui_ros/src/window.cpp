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

// #include "imgui.h"
// #include "imgui_impl_opengl3.h"
// #include "imgui_impl_sdl.h"
// #include <imgui_ros/AddWindow.h>
#include <algorithm>
#include <imgui_ros/window.h>
// #include <opencv2/highgui.hpp>

void Window::draw() {
  ImGui::Begin(name_.c_str());
  // for (auto& widget : widgets_) {
  for (auto& name : widget_order_) {
    if (widgets_[name]) {
      if (ImGui::CollapsingHeader((name + "##header").c_str())) {
        widgets_[name]->draw();
      }
    }
  }
  ImGui::End();
}

// TODO(lucasw) add ability to insert earlier into list, or if already exists
// then keep current position, currently only adds to end
void Window::add(std::shared_ptr<Widget> widget) {
  const std::string name = widget->name_;
  remove(name);
  widgets_[name] = widget;
  widget_order_.push_back(name);
}

void Window::remove(const std::string& name) {
  if (widgets_.count(name) > 0) {
    widget_order_.erase(std::remove(widget_order_.begin(),
        widget_order_.end(), name),
        widget_order_.end());
  }
  widgets_.erase(name);
}

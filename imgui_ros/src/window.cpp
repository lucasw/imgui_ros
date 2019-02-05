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

Window::~Window()
{
  std::cout << "freeing window " << this << "\n";
}

void Window::draw() {

  if (init_) {
    ImGui::SetNextWindowPos(pos_);
    ImGui::SetNextWindowSize(size_);
    ImGui::SetNextWindowCollapsed(collapsed_);
  }

  ImGui::Begin(name_.c_str(), NULL, window_flags_);

  // std::stringstream ss;
  // ss << tab_groups_.size() << " ";

  ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_None;
  if (tab_groups_.size() > 1) {
    if (ImGui::BeginTabBar(name_.c_str(), tab_bar_flags)) {
      for (auto tab_pair : tab_groups_) {
        const auto tab_name = tab_pair.first;
        const auto tab_group = tab_pair.second;
        // std::cout << "'" << name_ << "' " << tab_name << "\n";
        // ss << tab_name << " ";
        if (tab_group) {
          if (ImGui::BeginTabItem(tab_name.c_str())) {
            tab_group->draw();
            ImGui::EndTabItem();
          }
        }
      }
    }
    ImGui::EndTabBar();
  } else if (tab_groups_.size() == 1) {
    for (auto tab_pair : tab_groups_) {
      const auto tab_name = tab_pair.first;
      const auto tab_group = tab_pair.second;
      tab_group->draw();
    }
  }
  // ImGui::Text("%s", ss.str().c_str());

  // won't know scroll max y until after drawing window?
  if (init_) {
    const float scroll_y_adj = scroll_y_ * ImGui::GetScrollMaxY();
    ImGui::SetScrollY(scroll_y_adj);
    init_ = false;
  }
  // ImGui::Text("%0.2f %0.2f %0.2f", ImGui::GetScrollY(),
  //       ImGui::GetScrollMaxY(), scroll_y_);
  ImGui::End();
}

void Window::TabGroup::draw() {
  for (auto& name : widget_order_) {
    if (widgets_[name]) {
      // TODO(lucasw) make collapsing header optional
      // if (ImGui::CollapsingHeader((name + "##header").c_str())) {
      {
        widgets_[name]->draw();
      }
    }
  }
}

void Window::add(std::shared_ptr<Widget> widget, const std::string& tab_name) {
  if (tab_groups_.count(tab_name) < 1) {
    std::cout << "'" << name_ << "' new tab group '" << tab_name << "' "
        << tab_groups_.size() << " " << this << "\n";
    tab_groups_[tab_name] = std::make_shared<Window::TabGroup>(tab_name);
  }
  tab_groups_[tab_name]->add(widget);
}

void Window::remove(const std::string& name, const std::string& tab_name) {
  if (tab_groups_.count(tab_name) > 0) {
    std::cout << "'" << name_ << "' removing tab group '" << tab_name << "' widget '"
        << name << "'\n";
    tab_groups_[tab_name]->remove(name);
  }
  // TODO(lucasw) if no widgets in current tab name, remove it
}

// TODO(lucasw) add ability to insert earlier into list, or if already exists
// then keep current position, currently only adds to end
void Window::TabGroup::add(std::shared_ptr<Widget> widget) {
  const std::string name = widget->name_;
  remove(name);
  widgets_[name] = widget;
  widget_order_.push_back(name);
}

void Window::TabGroup::remove(const std::string& name) {
  if (widgets_.count(name) > 0) {
    widget_order_.erase(std::remove(widget_order_.begin(),
        widget_order_.end(), name),
        widget_order_.end());
  }
  widgets_.erase(name);
}

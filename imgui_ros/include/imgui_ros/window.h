/*
 * Copyright (c) 2017 Lucas Walter
 * June 2017
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

#ifndef IMGUI_ROS2_WINDOW_H
#define IMGUI_ROS2_WINDOW_H

#include <imgui.h>
#include <map>
#include <memory>
#include <mutex>
#include <tf2_msgs/msg/tf_message.hpp>
#include <vector>

struct Widget {
  Widget(const std::string name, const std::string topic) :
      name_(name), topic_(topic) {}
  ~Widget() {}
  virtual void draw() = 0;
  std::string name_ = "";

  virtual void addTF(tf2_msgs::msg::TFMessage& tfm)
  {
    (void)tfm;
  }
protected:
  bool dirty_ = true;
  std::string topic_ = "";
  std::mutex mutex_;
};

struct Window {
  Window(const std::string name) :
      name_(name) {}
  ~Window() {}
  virtual void draw();
  void add(std::shared_ptr<Widget> widget);
  virtual void addTF(tf2_msgs::msg::TFMessage& tfm) {
    for (auto& name : widget_order_) {
      if (widgets_[name]) {
        widgets_[name]->addTF(tfm);
      }
    }
  }
protected:
  // TODO(lucasw) this sorts into alphabetical order, but want to preserve insertion order
  std::map<std::string, std::shared_ptr<Widget> > widgets_;
  std::vector<std::string> widget_order_;
  bool dirty_ = true;
  std::string name_ = "";
  std::mutex mutex_;
};

#endif  // IMGUI_ROS2_IMAGE_H

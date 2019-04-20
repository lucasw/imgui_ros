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

#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
// #include <dynamic_reconfigure/GroupState.h>
#include <imgui.h>
#include <imgui_ros/dynamic_reconfigure.h>

namespace imgui_ros
{
DynamicReconfigure::DynamicReconfigure(const std::string name, const std::string topic,
    ros::NodeHandle& nh) : Widget(name, topic) {
  const std::string desc_topic = topic + "/parameter_descriptions";
  descriptions_sub_ = nh.subscribe(desc_topic, 10,
      &DynamicReconfigure::descriptionCallback, this);
  const std::string updates_topic = topic + "/parameter_updates";
  updates_sub_ = nh.subscribe(updates_topic, 10,
      &DynamicReconfigure::updatesCallback, this);
  client_ = nh.serviceClient<dynamic_reconfigure::Reconfigure>(topic + "/set_parameters");
  // TODO(lucasw) make this configurable - through optional control?
  timer_ = nh.createTimer(ros::Duration(0.1), &DynamicReconfigure::updateParameters, this);
}

void DynamicReconfigure::descriptionCallback(
    const dynamic_reconfigure::ConfigDescriptionConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_description_ = *msg;
}

void DynamicReconfigure::updatesCallback(
    const dynamic_reconfigure::ConfigConstPtr& msg) {
  // TODO(lucasw) if this arrives before the updateParameters has run with changes those will
  // get overwritten- is that good or bad?
  std::lock_guard<std::mutex> lock(mutex_);
  config_description_.dflt = *msg;
}

void DynamicReconfigure::draw() {
  std::stringstream ss;
  ss << name_ << " - " << topic_;
  // ImGui::Begin(ss.str().c_str());
  const std::string text = topic_;
  // ImGui::Text("%.*s", static_cast<int>(text.size()), text.data());
  std::lock_guard<std::mutex> lock(mutex_);
  dynamic_reconfigure::ConfigDescription& cd = config_description_;
  {
    // std::lock_guard<std::mutex> lock(mutex_);
  }
  ROS_DEBUG_STREAM("bools "
      << " " << cd.dflt.bools.size()
      << " " << cd.min.bools.size()
      << " " << cd.max.bools.size());
  ROS_DEBUG_STREAM("doubles "
      << " " << cd.dflt.doubles.size()
      << " " << cd.min.doubles.size()
      << " " << cd.max.doubles.size());

  // TODO(lucasw) assume config description is properly formed for now
  for (size_t i = 0; i < cd.dflt.bools.size(); ++i) {
    const std::string name = cd.dflt.bools[i].name;
    ROS_DEBUG_STREAM(name << " checkbox");
    bool new_value = cd.dflt.bools[i].value;
    const bool changed = ImGui::Checkbox(name.c_str(), &new_value);
    cd.dflt.bools[i].value = new_value;
    if (changed) {
      do_reconfigure_ = true;
    }
  }
  for (size_t i = 0; i < cd.dflt.doubles.size(); ++i) {
    const std::string name = cd.dflt.doubles[i].name;
    if (i >= cd.min.doubles.size()) {
      ROS_ERROR_STREAM("short min " << name << " " << i
          << " " << cd.min.doubles.size());
      break;
    }
    if (i >= cd.max.doubles.size()) {
      ROS_ERROR_STREAM("short min " << name << " " << i
          << " " << cd.max.doubles.size());
      break;
    }
    const double min = cd.min.doubles[i].value;
    const double max = cd.max.doubles[i].value;
    ROS_DEBUG_STREAM(name << " " << i << " double " << min << " " << max);
    double new_value = cd.dflt.doubles[i].value;
    const bool changed = ImGui::SliderScalar(name.c_str(), ImGuiDataType_Double,
        (void *)&new_value, (void*)&min, (void*)&max, "%f");
    if (changed) {
      cd.dflt.doubles[i].value = new_value;
      do_reconfigure_ = true;
    }
  }
  // ImGui::End();
}

void DynamicReconfigure::updateParameters(const ros::TimerEvent& e)
{
  (void)e;
  dynamic_reconfigure::Reconfigure rec;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    rec.request.config = config_description_.dflt;
    if (!do_reconfigure_)
      return;
    do_reconfigure_ = false;
  }
  // TODO(lucasw) it's possible there are multiples of the same name in reconfigure_
  // which one will take precedence- the last one?
  // would be better to use a map then only fill out a Reconfigure here.
  if (!client_.call(rec)) {
  }
}
}  // namespace imgui_ros

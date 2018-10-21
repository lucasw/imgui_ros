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
// #include <dynamic_reconfigure/GroupState.h>
#include <imgui.h>
#include <imgui_ros/dynamic_reconfigure.h>

DynamicReconfigure::DynamicReconfigure(const std::string name, const std::string topic,
    ros::NodeHandle& nh) : Window(name, topic) {
  const std::string desc_topic = topic + "/parameter_descriptions";
  descriptions_sub_ = nh.subscribe(desc_topic, 10,
      &DynamicReconfigure::descriptionCallback, this);
}

void DynamicReconfigure::descriptionCallback(
    const dynamic_reconfigure::ConfigDescriptionConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_description_ = msg;
  // TODO(lucasw) clear out all the maps
}

void DynamicReconfigure::draw() {
  std::stringstream ss;
  ss << name_ << " - " << topic_;
  ImGui::Begin(ss.str().c_str());
  const std::string text = topic_;
  // ImGui::Text("%.*s", static_cast<int>(text.size()), text.data());
  dynamic_reconfigure::ConfigDescriptionConstPtr cd;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cd = config_description_;
  }
  if (!cd) {
    ImGui::End();
    return;
  }
  ROS_DEBUG_STREAM("bools "
      << " " << cd->dflt.bools.size()
      << " " << cd->min.bools.size()
      << " " << cd->max.bools.size());
  ROS_DEBUG_STREAM("doubles "
      << " " << cd->dflt.doubles.size()
      << " " << cd->min.doubles.size()
      << " " << cd->max.doubles.size());

  // TODO(lucasw) assume config description is properly formed for now
  for (size_t i = 0; i < cd->dflt.bools.size(); ++i) {
    const std::string name = cd->dflt.bools[i].name;
    ROS_DEBUG_STREAM(name << " checkbox");
    bools_[name] = false;
    ImGui::Checkbox(name.c_str(), &bools_[name]);
  }
  for (size_t i = 0; i < cd->dflt.doubles.size(); ++i) {
    const std::string name = cd->dflt.doubles[i].name;
    if (i >= cd->min.doubles.size()) {
      ROS_ERROR_STREAM("short min " << name << " " << i
          << " " << cd->min.doubles.size());
      break;
    }
    if (i >= cd->max.doubles.size()) {
      ROS_ERROR_STREAM("short min " << name << " " << i
          << " " << cd->max.doubles.size());
      break;
    }
    const double min = cd->min.doubles[i].value;
    const double max = cd->max.doubles[i].value;
    ROS_DEBUG_STREAM(name << " " << i << " double " << min << " " << max);
    doubles_[name] = cd->dflt.doubles[i].value;
    ImGui::SliderScalar(name.c_str(), ImGuiDataType_Double,
        (void *)&doubles_[name], (void*)&min, (void*)&max, "%f");
  }

  ImGui::End();
}

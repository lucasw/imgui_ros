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
#include <yaml-cpp/yaml.h>

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


  // Temp debug to look for enum edit_method
  // it looks like the edit method is a yaml string that is not broken out
  // into a class member, will have to parse it.
  // ImGui::Text("groups size %lu", cd.groups.size());
  dr_enums_.clear();
  dr_enums_combo_text_.clear();
  dynamic_reconfigure::ConfigDescription& cd = config_description_;
  for (size_t i = 0; i < cd.groups.size(); ++i) {
    const auto& group = cd.groups[i];
    // ImGui::Text("%lu '%s' '%s' %lu",
    //     i, group.name.c_str(), group.type.c_str(),
    //     group.parameters.size());
    for (size_t j = 0; j < group.parameters.size(); ++j) {
      const auto& parameter = group.parameters[j];
      if (parameter.edit_method == "") {
        continue;
      }

/**
{
  'enum_description': 'An enum to set size',
  'enum': [
    {
      'srcline': 17,
      'description':
      'A small constant',
      'srcfile': '/home/lucasw/catkin_ws/src/dynamic_reconfigure_tools/dynamic_reconfigure_example/cfg/Example.cfg',
      'cconsttype': 'const int',
      'value': 0,
      'ctype': 'int',
      'type': 'int',
      'name': 'Small'
    },
*/
      YAML::Node node = YAML::Load(parameter.edit_method);
      auto enum_list = node["enum"];
      // for (auto& item : enum_list) {
      if (enum_list.IsSequence()) {
        ROS_INFO_STREAM("parameter with enum " << parameter.name);
        dr_enums_[parameter.name].clear();
        dr_enums_combo_text_[parameter.name] = "";
        for (auto it = enum_list.begin(); it != enum_list.end(); ++it) {
          DrEnum dr_enum;
          dr_enum.name_ = (*it)["name"].as<std::string>();
          dr_enum.value_ = (*it)["value"].as<std::string>();
          dr_enum.type_ = (*it)["type"].as<std::string>();
          dr_enum.description_ = (*it)["description"].as<std::string>();
          dr_enums_combo_text_[parameter.name] += dr_enum.name_ +
              " (" + dr_enum.value_ + ")" + '\0';
          ROS_INFO_STREAM(dr_enum.name_ << " "
              << dr_enum.value_ << " "
              << dr_enum.type_ << " "
              << dr_enum.description_);
          dr_enums_[parameter.name].push_back(dr_enum);
        }
      }
      // ImGui::Text("%lu '%s' '%s' '%s' '%s'", j,
      //     parameter.name.c_str(),
      //     parameter.type.c_str(),
      //     parameter.description.c_str(),
      //    parameter.edit_method.c_str());
    }
  }
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
  auto& dflt = cd.dflt;
  auto& bools = dflt.bools;
  ROS_DEBUG_STREAM("bools "
      << " " << bools.size()
      << " " << cd.min.bools.size()
      << " " << cd.max.bools.size());
  ROS_DEBUG_STREAM("doubles "
      << " " << dflt.doubles.size()
      << " " << cd.min.doubles.size()
      << " " << cd.max.doubles.size());

  // TODO(lucasw) assume config description is properly formed for now
  for (size_t i = 0; i < bools.size(); ++i) {
    const std::string& name = bools[i].name;
    ROS_DEBUG_STREAM(name << " checkbox");
    bool new_value = bools[i].value;
    const bool changed = ImGui::Checkbox(name.c_str(), &new_value);
    bools[i].value = new_value;
    if (changed) {
      do_reconfigure_ = true;
    }
  }
  auto& doubles = dflt.doubles;
  for (size_t i = 0; i < doubles.size(); ++i) {
    const std::string& name = doubles[i].name;
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
    double new_value = doubles[i].value;
    const bool changed = ImGui::SliderScalar(name.c_str(), ImGuiDataType_Double,
        (void *)&new_value, (void*)&min, (void*)&max, "%f");
    if (changed) {
      doubles[i].value = new_value;
      do_reconfigure_ = true;
    }
  }
  auto& ints = dflt.ints;
  for (size_t i = 0; i < ints.size(); ++i) {
    const std::string& name = ints[i].name;
    if (i >= cd.min.ints.size()) {
      ROS_ERROR_STREAM("short min " << name << " " << i
          << " " << cd.min.ints.size());
      break;
    }
    if (i >= cd.max.ints.size()) {
      ROS_ERROR_STREAM("short min " << name << " " << i
          << " " << cd.max.ints.size());
      break;
    }
    int new_value = ints[i].value;
    bool changed = false;
    // check if enum
    if (dr_enums_.count(name) < 1) {
      const int min = cd.min.ints[i].value;
      const int max = cd.max.ints[i].value;
      ROS_DEBUG_STREAM(name << " " << i << " int " << min << " " << max);
      changed = ImGui::SliderInt(name.c_str(),
          &new_value, min, max);
    } else {
      // TODO(lucasw) if enums don't start at 0 and go to n-1 for n items
      // in the combo box this is going to fail
      changed = ImGui::Combo(name.c_str(), &new_value,
          dr_enums_combo_text_[name].c_str());
    }
    if (changed) {
      ints[i].value = new_value;
      do_reconfigure_ = true;
    }
  }
  auto& strs = dflt.strs;
  for (size_t i = 0; i < strs.size(); ++i) {
    const auto& str = strs[i];
    ROS_DEBUG_STREAM(str.name << " " << str.value);
    // ImGui::Text("parameters size %d", cd.parameters.size());
    const size_t max_string_size = 128;
    char new_value[max_string_size];
    sprintf(new_value, "%s", str.value.substr(0, max_string_size - 1).c_str());
    const bool changed = ImGui::InputText(str.name.c_str(),
        &new_value[0], IM_ARRAYSIZE(new_value), ImGuiInputTextFlags_EnterReturnsTrue);
    if (changed) {
      strs[i].value = new_value;
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

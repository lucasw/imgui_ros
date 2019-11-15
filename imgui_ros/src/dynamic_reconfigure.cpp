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

  // The edit method is a yaml string that is not broken out
  // into a class member, have to parse it.
  // ImGui::Text("groups size %lu", cd.groups.size());
  dr_enums_.clear();
  dr_enums_combo_text_.clear();
  groups_of_parameters_.clear();
  parameters_to_groups_.clear();
  dynamic_reconfigure::ConfigDescription& cd = config_description_;
  for (size_t i = 0; i < cd.groups.size(); ++i) {
    const auto& group = cd.groups[i];
    // ImGui::Text("%lu '%s' '%s' %lu",
    //     i, group.name.c_str(), group.type.c_str(),
    //     group.parameters.size());

    // TODO(lucasw) need to store which parameter names go in which groups using group.name-
    // maybe want both directions, a map of groups with lists of names,
    // and a map of names with which group they are in.
    for (size_t j = 0; j < group.parameters.size(); ++j) {
      const auto& parameter = group.parameters[j];
      // bi-directional maps
      groups_of_parameters_[group.name].push_back(parameter.name);
      parameters_to_groups_[parameter.name] = group.name;
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
              << dr_enum.type_ << " '"
              << dr_enum.description_ << "'");
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
  // TODO(lucasw) instead of assuming the update has every value,
  // maybe loop through everything in the msg and find what it needs
  // to update in dflt.
  config_description_.dflt = *msg;
}

void DynamicReconfigure::draw() {
  std::stringstream ss;
  ss << name_ << " - " << topic_;
  // ImGui::Begin(ss.str().c_str());
  const std::string text = topic_;
  // ImGui::Text("%.*s", static_cast<int>(text.size()), text.data());
  std::lock_guard<std::mutex> lock(mutex_);
  const auto& cd_min = config_description_.min;
  const auto& cd_max = config_description_.max;

/**
  ROS_DEBUG_STREAM("bools "
      << " " << bools.size()
      << " " << cd_min.bools.size()
      << " " << cd_max.bools.size());
  ROS_DEBUG_STREAM("doubles "
      << " " << dflt.doubles.size()
      << " " << cd_min.doubles.size()
      << " " << cd_max.doubles.size());
*/

  // need to organize all the parameters into their proper groups
  std::map<std::string, std::map<std::string, std::pair<std::string, size_t>>>
      group_parameter_type_ind;

  auto& bools = config_description_.dflt.bools;
  for (size_t i = 0; i < bools.size(); ++i) {
    const std::string& name = bools[i].name;
    const std::string& group = parameters_to_groups_[name];
    group_parameter_type_ind[group][name] = std::make_pair("bool", i);
  }
  auto& doubles = config_description_.dflt.doubles;
  for (size_t i = 0; i < doubles.size(); ++i) {
    const std::string& name = doubles[i].name;
    const std::string& group = parameters_to_groups_[name];
    group_parameter_type_ind[group][name] = std::make_pair("double", i);
  }
  auto& ints = config_description_.dflt.ints;
  for (size_t i = 0; i < ints.size(); ++i) {
    const std::string& name = ints[i].name;
    const std::string& group = parameters_to_groups_[name];
    group_parameter_type_ind[group][name] = std::make_pair("int", i);
  }
  auto& strs = config_description_.dflt.strs;
  for (size_t i = 0; i < strs.size(); ++i) {
    const std::string& name = strs[i].name;
    const std::string& group = parameters_to_groups_[name];
    group_parameter_type_ind[group][name] = std::make_pair("string", i);
  }

  ImGui::Text("%s", topic_.c_str());
  // TODO(lucasw) assume config description is properly formed for now
  for (const auto& group_pair : groups_of_parameters_) {  // group_parameter_type_ind) {
    const std::string& group_name = group_pair.first;
    // TODO(lucasw) maybe make the groups expandable, or indent them?


    if (group_name != "Default") {
      ImGui::Text("%s", group_name.c_str());
    }
    const auto& parameters = group_pair.second;
    for (const std::string& parameter_name : parameters) {
      const auto& type_ind_pair = group_parameter_type_ind[group_name][parameter_name];
      const std::string& dr_type = type_ind_pair.first;
      const size_t ind = type_ind_pair.second;

      // TODO(lucasw) replace with switch, enum for type
      if (dr_type == "bool") {
        // could match this name against parameter_name
        const std::string& name = bools[ind].name;
        ROS_DEBUG_STREAM(name << " checkbox");
        bool new_value = bools[ind].value;
        // This doesn't work
        // const char* widget_name = (name + "##" + name_).c_str();
        // const bool changed = ImGui::Checkbox(widget_name, &new_value);
        const bool changed = ImGui::Checkbox((name + "##" + name_).c_str(), &new_value);
        if (changed) {
          bools[ind].value = new_value;
          bools_[name] = bools[ind];
          do_reconfigure_ = true;
        }
      } else if (dr_type == "double") {
        const std::string& name = doubles[ind].name;
        if (ind >= cd_min.doubles.size()) {
          ROS_ERROR_STREAM("short min " << name << " " << ind
              << " " << cd_min.doubles.size());
          continue;
        }
        if (ind >= cd_max.doubles.size()) {
          ROS_ERROR_STREAM("short min " << name << " " << ind
              << " " << cd_max.doubles.size());
          continue;
        }
        const double min = cd_min.doubles[ind].value;
        const double max = cd_max.doubles[ind].value;
        ROS_DEBUG_STREAM(name << " " << ind << " double " << min << " " << max);
        double new_value = doubles[ind].value;
        const bool changed = ImGui::SliderScalar((name + "##" + name_).c_str(),
        // this is producing garbage text
        // const auto widget_name = (name + "##" + name_).c_str();
        // const bool changed = ImGui::SliderScalar(widget_name,
            ImGuiDataType_Double,
            (void *)&new_value, (void*)&min, (void*)&max, "%f");
        if (changed) {
          doubles[ind].value = new_value;
          doubles_[name] = doubles[ind];
          do_reconfigure_ = true;
        }
      } else if (dr_type == "int") {
        const std::string& name = ints[ind].name;
        const auto widget_name = (name + "##" + name_).c_str();
        if (ind >= cd_min.ints.size()) {
          ROS_ERROR_STREAM("short min " << name << " " << ind
              << " " << cd_min.ints.size());
          continue;
        }
        if (ind >= cd_max.ints.size()) {
          ROS_ERROR_STREAM("short min " << name << " " << ind
              << " " << cd_max.ints.size());
          continue;
        }
        int new_value = ints[ind].value;
        bool changed = false;
        // check if enum
        if (dr_enums_.count(name) < 1) {
          const int min = cd_min.ints[ind].value;
          const int max = cd_max.ints[ind].value;
          ROS_DEBUG_STREAM(name << " " << ind << " int " << min << " " << max);
          changed = ImGui::SliderInt(widget_name,
              &new_value, min, max);
        } else {
          // TODO(lucasw) if enums don't start at 0 and go to n-1 for n items
          // in the combo box this is going to fail
          // ROS_INFO_STREAM_THROTTLE(4.0, dr_enums_combo_text_[name]);
          changed = ImGui::Combo(widget_name, &new_value,
              dr_enums_combo_text_[name].c_str());
        }
        if (changed) {
          ints[ind].value = new_value;
          ints_[name] = ints[ind];
          do_reconfigure_ = true;
        }
      } else if (dr_type == "string") {
        const std::string& name = strs[ind].name;
        const auto widget_name = (name + "##" + name_).c_str();
        if (ind >= strs.size()) {
          ROS_ERROR_STREAM("bad ind " << group_name << " " << parameter_name << " " << ind
              << " " << strs.size());
          continue;
        }
        const auto& str = strs[ind];
        ROS_DEBUG_STREAM(str.name << " " << str.value << " " << ind << " " << strs.size());
        // ImGui::Text("parameters size %d", cd.parameters.size());
        if ((dr_enums_.count(name) < 1) || (dr_enums_[name].size() < 1)) {
          const size_t max_string_size = 128;
          char new_value[max_string_size];
          sprintf(new_value, "%s", str.value.substr(0, max_string_size - 1).c_str());
          const bool changed = ImGui::InputText(widget_name,
              &new_value[0], IM_ARRAYSIZE(new_value), ImGuiInputTextFlags_EnterReturnsTrue);

          if (changed) {
            ROS_INFO_STREAM(new_value);
            strs[ind].value = new_value;
            strs_[str.name] = strs[ind];
            do_reconfigure_ = true;
          }
        } else {
          ImGuiComboFlags flags = 0;
          auto& item_current = dr_enums_[name][0];
#if 0
          if (ImGui::BeginCombo(widget_name, item_current.name_.c_str(), flags)) {
            for (const auto& dr_enum : dr_enums_[name]) {
              bool is_selected = (item_current.name_ == dr_enum.name_);
              if (ImGui::Selectable(dr_enum.name_.c_str(), is_selected)) {
                item_current = dr_enum;
              }
              if (is_selected) {
                ROS_INFO_STREAM(dr_enum.name_);
                ImGui::SetItemDefaultFocus();
              }
            }
            ImGui::EndCombo();
          }
#endif
        }
      } else {
        ROS_ERROR_STREAM("unknown parameter type '" << dr_type << "' '"
            << parameter_name << "'");
      } // switch
    }  // loop through parameters in this group
  }  // loop through groups

  // ImGui::End();
}

void DynamicReconfigure::updateParameters(const ros::TimerEvent& e)
{
  (void)e;
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!do_reconfigure_)
      return;

    // TODO(lucasw) Need to populate group https://github.com/lucasw/imgui_ros/issues/9#issuecomment-485308307
    // for min, max, and dflt when dealing with a rospy server- look to see if the group fields
    // are empty, then populate them with the group information from config_description_.groups
    // TODO(lucasw), the problem is likely in the rospy server code,
    // doesn't matter if this is set here
    if (false) {
    for (const auto& group : config_description_.groups) {
      bool found_group = false;
      for (const auto& dflt_group : config_description_.dflt.groups) {
        if (dflt_group.name == group.name) {
          found_group = true;
          break;
        }
      }
      if (!found_group) {
        ROS_INFO_STREAM("adding missing group state " << group.name << " "
            << group.id << " " << group.parent);
        dynamic_reconfigure::GroupState group_state;
        group_state.name = group.name;
        group_state.state = true;
        group_state.id = group.id;
        group_state.parent = group.parent;
        config_description_.dflt.groups.push_back(group_state);
      }
    }
    }

    // TODO(lucasw) request changes only to the values that actually changed, don't send all of dflt
    // rec.request.config = config_description_.dflt;
    do_reconfigure_ = false;
  }

  dynamic_reconfigure::Reconfigure rec;
  for (const auto& pair : bools_) {
    rec.request.config.bools.push_back(pair.second);
  }
  bools_.clear();
  for (const auto& pair : ints_) {
    rec.request.config.ints.push_back(pair.second);
  }
  ints_.clear();
  for (const auto& pair : strs_) {
    rec.request.config.strs.push_back(pair.second);
  }
  strs_.clear();
  for (const auto& pair : doubles_) {
    rec.request.config.doubles.push_back(pair.second);
  }
  doubles_.clear();

  if (!client_.call(rec)) {
    ROS_ERROR("bad reconfigure");
  }
}
}  // namespace imgui_ros

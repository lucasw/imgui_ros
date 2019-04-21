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

#ifndef IMGGUI_ROS_DYNAMIC_RECONFIGURE_H
#define IMGGUI_ROS_DYNAMIC_RECONFIGURE_H

#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <imgui.h>
#include <imgui_ros/window.h>
#include <map>
#include <mutex>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

namespace imgui_ros
{
struct DynamicReconfigure : public Widget {
  DynamicReconfigure(const std::string name, const std::string topic,
                     ros::NodeHandle& nh);
  ~DynamicReconfigure() {}
  void descriptionCallback(const dynamic_reconfigure::ConfigDescriptionConstPtr& msg);
  void updatesCallback(const dynamic_reconfigure::ConfigConstPtr& msg);
  virtual void draw() override;
private:
  ros::Subscriber descriptions_sub_;
  dynamic_reconfigure::ConfigDescription config_description_;
  ros::Subscriber updates_sub_;
  dynamic_reconfigure::ConfigConstPtr config_;
  ros::ServiceClient client_;

  struct DrEnum
  {
    std::string name_;
    std::string value_;
    std::string type_;
    std::string description_;
  };
  std::map<std::string, std::vector<DrEnum> > dr_enums_;
  std::map<std::string, std::string> dr_enums_combo_text_;

  bool do_reconfigure_ = false;
  ros::Timer timer_;
  void updateParameters(const ros::TimerEvent& e);
};  // DynamicReconfigure
}  // namespae imgui_ros
#endif  // IMGGUI_ROS_DYNAMIC_RECONFIGURE_H

/*
 * Copyright (c) 2019 Lucas Walter
 * February 2019
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

#ifndef INTERNAL_PUB_SUB_NODE_LOADER_HPP
#define INTERNAL_PUB_SUB_NODE_LOADER_HPP

#include <class_loader/class_loader.hpp>
#include <internal_pub_sub/internal_pub_sub.hpp>
#include <internal_pub_sub/srv/add_node.hpp>
#include <rclcpp/rclcpp.hpp>

namespace internal_pub_sub
{

void run_node(std::shared_ptr<rclcpp::Node> node);

struct NodeInfo
{
  // NodeInfo()
  // {
  // }

  NodeInfo(const std::string& node_namespace,
      const std::string& node_name,
      const std::string& package_name,
      const std::string& plugin_name,
      std::shared_ptr<rclcpp::Node> node,
      std::shared_ptr<internal_pub_sub::Node> ips_node,
      std::shared_ptr<class_loader::ClassLoader> loader
      ) :
      node_namespace_(node_namespace),
      node_name_(node_name),
      package_name_(package_name),
      plugin_name_(plugin_name),
      node_(node),
      ips_node_(ips_node),
      loader_(loader)
  {
    RCLCPP_INFO(node->get_logger(), "executing node %s %s %s %s",
        node_namespace_.c_str(), node_name_.c_str(),
        package_name_.c_str(), plugin_name_.c_str());

    executor_.add_node(node);
    // TODO(lucasw) want multi threaded option
    // TODO(lucasw) maybe want start stop controls instead of just run
    // at construction and stop and destruction
    thread_ = std::thread(std::bind(&NodeInfo::run, this));
  }

  void run()
  {
    executor_.spin();
  }

  ~NodeInfo()
  {
    std::cout << "unloading node " << node_namespace_ << " " << node_name_ << " "
        << package_name_ << " " << plugin_name_ << "\n";
    executor_.cancel();
    thread_.join();
    node_ = nullptr;
    ips_node_ = nullptr;
    // need to unload this last, otherwise get error message
    loader_ = nullptr;
  }

  std::string node_namespace_;
  std::string node_name_;
  std::string package_name_;
  std::string plugin_name_;

  std::shared_ptr<rclcpp::Node> node_;
  // TODO(lucasw) or shove these into nodes above?
  std::shared_ptr<internal_pub_sub::Node> ips_node_;

  std::thread thread_;
  // if the loader is destructed then the node/s it loaded go away with it
  // TODO(lucasw) should there be a map of package_name loaders?
  // and not keep this here?
  std::shared_ptr<class_loader::ClassLoader> loader_;

  // rclcpp::executors::MultiThreadedExecutor executor_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

struct NodeLoader : public internal_pub_sub::Node
{
  NodeLoader();
  ~NodeLoader();
  virtual void postInit(std::shared_ptr<Core> core);

  rclcpp::Service<srv::AddNode>::SharedPtr add_node_;
  void addNode(const std::shared_ptr<srv::AddNode::Request> req,
      std::shared_ptr<srv::AddNode::Response> res);

  std::shared_ptr<class_loader::ClassLoader> getLoader(const std::string& package_name);
  //    const std::string& plugin_name);

  bool load(
      std::shared_ptr<class_loader::ClassLoader> loader,
      const std::string& package_name, const std::string& plugin_name,
      const std::string& node_name, const std::string& node_namespace,
      const std::vector<std::string>& arguments,
      const std::map<std::string, std::string>& remappings,
      const std::vector<rclcpp::Parameter>& parameters,
      const bool internal_pub_sub);

  // namespace and node name as keys, any identically named node will
  // unload the older node.
  std::map<std::string, std::map<std::string, std::shared_ptr<NodeInfo> > > node_infos_;
  std::map<std::string, std::shared_ptr<class_loader::ClassLoader> > loaders_;
  rclcpp::TimerBase::SharedPtr timer_;
  void update();
};

}  // namespace internal_pub_sub

#endif  // INTERNAL_PUB_SUB_NODE_LOADER_HPP

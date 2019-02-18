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

struct NodeLoader : public internal_pub_sub::Node
{
  NodeLoader();
  virtual void postInit();

  rclcpp::Service<srv::AddNode>::SharedPtr add_node_;
  void addNode(const std::shared_ptr<srv::AddNode::Request> req,
      std::shared_ptr<srv::AddNode::Response> res);

  std::shared_ptr<class_loader::ClassLoader> getLoader(const std::string& package_name,
      const std::string& plugin_name);

  bool load(
      std::shared_ptr<class_loader::ClassLoader> loader,
      const std::string& package_name, const std::string& plugin_name,
      const std::string& node_name, const std::string& node_namespace,
      const bool internal_pub_sub);

  std::vector<std::shared_ptr<class_loader::ClassLoader> > loaders_;
  std::vector<std::shared_ptr<rclcpp::Node> > nodes_;
  // TODO(lucasw) or shove these into nodes above?
  std::vector<std::shared_ptr<internal_pub_sub::Node> > ips_nodes_;

  std::shared_ptr<Core> core_;
};

}  // namespace internal_pub_sub

#endif  // INTERNAL_PUB_SUB_NODE_LOADER_HPP

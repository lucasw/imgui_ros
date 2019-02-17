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

#include <ament_index_cpp/get_packages_with_prefixes.hpp>
#include <ament_index_cpp/get_resource.hpp>
#include <ament_index_cpp/get_search_paths.hpp>
#include <internal_pub_sub/node_loader.hpp>

#ifdef __clang__
// TODO(dirk-thomas) custom implementation until we can use libc++ 3.9
#include <string>
namespace fs
{
class path
{
public:
  explicit path(const std::string & p)
  : path_(p)
  {}
  bool is_absolute()
  {
    return path_[0] == '/';
  }

private:
  std::string path_;
};
}  // namespace fs
#else
# include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif


// based on demos/composition/api_composition.cpp 
std::vector<std::string> split(
  const std::string & s, char delim, bool skip_empty = false)
{
  std::vector<std::string> result;
  std::stringstream ss;
  ss.str(s);
  std::string item;
  while (std::getline(ss, item, delim)) {
    if (skip_empty && item == "") {
      continue;
    }
    result.push_back(item);
  }
  return result;
}

namespace internal_pub_sub
{

NodeLoader::NodeLoader() : Node()
{
#if 0
  auto packages = ament_index_cpp::get_packages_with_prefixes();
  for (auto pair : packages) {
    std::cout << pair.first << " " << pair.second << "\n";
  }
#endif
#if 0
  auto paths = ament_index_cpp::get_search_paths();
  for (auto path : paths) {
    std::cout << path << "\n";
  }
#endif
}

void NodeLoader::postInit()
{
  Node::postInit();
  add_node_ = create_service<srv::AddNode>("add_node",
      std::bind(&NodeLoader::addNode, this, std::placeholders::_1, std::placeholders::_2));
}

void NodeLoader::addNode(const std::shared_ptr<srv::AddNode::Request> req,
    std::shared_ptr<srv::AddNode::Response> res)
{

  res->success = true;
  for (auto node_to_add : req->nodes) {
    auto loader = getLoader(node_to_add.package_name);
    if (loader == nullptr) {
      return;
    }
    const bool rv = load(
        loader,
        node_to_add.package_name,
        node_to_add.plugin_name,
        node_to_add.node_name,
        node_to_add.node_namespace,
        node_to_add.internal_pub_sub);
    res->success &= rv;
  }
  return;
}

bool NodeLoader::load(std::shared_ptr<class_loader::ClassLoader> loader,
    const std::string& package_name, const std::string& plugin_name,
    const std::string& node_name, const std::string& node_namespace,
    const bool internal_pub_sub)
{
  const std::string full_node_plugin_name = package_name + "::" + plugin_name;

  try {
    if (internal_pub_sub) {
      auto node = loader->createInstance<internal_pub_sub::Node>(full_node_plugin_name);
      node->setCore(core_);
      node->init(node_name, node_namespace);
      ips_nodes_.push_back(node);
    } else {
      auto node = loader->createInstance<rclcpp::Node>(full_node_plugin_name);
      // TODO(lucasw) parameters
      node->init(node_name, node_namespace);
      nodes_.push_back(node);
    }
  } catch (class_loader::CreateClassException & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to load node: %s", ex.what());
    return false;
  }

  // TODO(lucasw) does this really need to be kept?
  loaders_.push_back(loader);

  return true;
}

std::shared_ptr<class_loader::ClassLoader> NodeLoader::getLoader(const std::string& package_name)
{
  std::shared_ptr<class_loader::ClassLoader> loader = nullptr;
  // get node plugin resource from package
  std::string content;
  std::string base_path;
  if (!ament_index_cpp::get_resource("node_plugin", package_name, content, &base_path)) {
    RCLCPP_ERROR(get_logger(), "Could not find requested resource in ament index: %s",
        package_name.c_str());
    return loader;
  }

  RCLCPP_INFO(get_logger(), "%s %s", content.c_str(), base_path.c_str());
  /** example output for package_name 'image_manip':
image_manip::IIRImage;lib/libimagemanip.so
image_manip::ImageDeque;lib/libimagemanip.so
image_manip::SaveImage;lib/libimagemanip.so
/home/lucasw/colcon_ws/install/image_manip
  */

  std::vector<std::string> lines = split(content, '\n', true);
  for (auto line : lines) {
    std::vector<std::string> parts = split(line, ';');
    if (parts.size() != 2) {
      RCLCPP_ERROR(get_logger(), "Invalid resource entry %s", line.c_str());
      continue;
    }

    std::string class_name = parts[0];

    // load node plugin
    std::string library_path = parts[1];

    if (!fs::path(library_path).is_absolute()) {
      library_path = base_path + "/" + library_path;
    }
    /**
/home/lucasw/colcon_ws/install/image_manip/lib/libimagemanip.so image_manip::IIRImage
/home/lucasw/colcon_ws/install/image_manip/lib/libimagemanip.so image_manip::ImageDeque
/home/lucasw/colcon_ws/install/image_manip/lib/libimagemanip.so image_manip::SaveImage
    */
    std::cout << library_path << " " << class_name << "\n";

    try {
      loader = std::make_shared<class_loader::ClassLoader>(library_path);
      return loader;
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to load library: %s", ex.what());
      return loader;
    }
  }
  return loader;
}

}  // namespace internal_pub_sub

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;

  auto node_loader = std::make_shared<internal_pub_sub::NodeLoader>();

  node_loader->init("node_loader", "management");
  node_loader->postInit();

  exec.add_node(node_loader);
  exec.spin();
  rclcpp::shutdown();
}

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
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>

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

NodeLoader::NodeLoader()
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

NodeLoader::~NodeLoader()
{
}

void NodeLoader::postInit(std::shared_ptr<Core> core)
{
  Node::postInit(core);
  add_node_ = create_service<srv::AddNode>("add_node",
      std::bind(&NodeLoader::addNode, this, std::placeholders::_1, std::placeholders::_2));

  timer_ = create_wall_timer(std::chrono::milliseconds(1000),
      std::bind(&NodeLoader::update, this));

  RCLCPP_INFO(get_logger(), "ready to load nodes");
}

void NodeLoader::addNode(const std::shared_ptr<srv::AddNode::Request> req,
    std::shared_ptr<srv::AddNode::Response> res)
{
  res->success = true;

  if (req->node_settings.size() == 0) {
    res->message = "no nodes to load";
    return;
  }
  res->message = "attempting to load nodes " + std::to_string(req->node_settings.size());

  for (auto node_to_add : req->node_settings) {
    // unload possibly existing node before loading a new one
    node_infos_[node_to_add.node_namespace][node_to_add.node_name] = nullptr;
    RCLCPP_INFO(get_logger(), "unloaded node");
    core_->clean();
    RCLCPP_INFO(get_logger(), "cleaned core");
    if (node_to_add.remove) {
      // TODO(lucasw) message differently if it didn't already exist?
      res->message += ", removed " + node_to_add.node_namespace + " " + node_to_add.node_name;
      continue;
    }

    auto loader = getLoader(node_to_add.package_name, node_to_add.plugin_name);
    if (loader == nullptr) {
      res->message += ", " + node_to_add.plugin_name + " loader is null";
      res->success = false;
      continue;
    }

    std::vector<rclcpp::Parameter> parameters;
    for (auto& param : node_to_add.parameters) {
      rclcpp::Parameter parameter = rclcpp::Parameter::from_parameter_msg(param);
      RCLCPP_INFO(get_logger(), "parameter %s %s %s",
          parameter.get_name().c_str(),
          parameter.get_type_name().c_str(),
          parameter.value_to_string().c_str());
      parameters.push_back(parameter);  // rclcpp::Parameter(param.name, rclcpp::ParameterValue(param.value)));
    }

    std::vector<std::string> arguments = node_to_add.arguments;
    std::map<std::string, std::string> remappings;
    for (auto& remapping : node_to_add.remappings) {
      arguments.push_back(remapping.from_topic + ":=" + remapping.to_topic);
      remappings[remapping.from_topic] = remapping.to_topic;
    }

    const bool rv = load(
        loader,
        node_to_add.package_name,
        node_to_add.plugin_name,
        node_to_add.node_name,
        node_to_add.node_namespace,
        arguments,
        remappings,
        parameters,
        node_to_add.internal_pub_sub);
    if (!rv) {
      res->message += ", " + node_to_add.plugin_name + " load failed";
    }
    res->success &= rv;
  }
  return;
}

bool NodeLoader::load(std::shared_ptr<class_loader::ClassLoader> loader,
    const std::string& package_name, const std::string& plugin_name,
    const std::string& node_name, const std::string& node_namespace,
    const std::vector<std::string>& arguments,
    const std::map<std::string, std::string>& remappings,
    const std::vector<rclcpp::Parameter>& parameters,
    const bool internal_pub_sub)
{
  // TODO(lucasw) could move this into constructor for NodeInfo
  const std::string full_node_plugin_name = package_name + "::" + plugin_name;

  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<internal_pub_sub::Node> ips_node;

  try {
    if (internal_pub_sub) {
      if (core_ == nullptr) {
        RCLCPP_ERROR(get_logger(), "core is uininitialized");
        return false;
      }
      ips_node = loader->createInstance<internal_pub_sub::Node>(full_node_plugin_name);
      ips_node->remappings_ = remappings;
      node = ips_node;
    } else {
      node = loader->createInstance<rclcpp::Node>(full_node_plugin_name);
    }
  } catch (class_loader::CreateClassException & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to load node: %s", ex.what());
    return false;
  }
  if (node == nullptr) {
    RCLCPP_ERROR(get_logger(), "Failed to create node: %s", full_node_plugin_name.c_str());
    return false;
  }

  // TODO(lucasw) arguments -> remappings?
  node->init(node_name, node_namespace,
      rclcpp::contexts::default_context::get_global_default_context(),
      arguments, parameters);

  if (ips_node) {
    ips_node->postInit(core_);
  }

  node_infos_[node_namespace][node_name] = std::make_shared<NodeInfo>(
      node_namespace, node_name, package_name, plugin_name,
      node,
      ips_node,
      loader);
  return true;
}

std::shared_ptr<class_loader::ClassLoader> NodeLoader::getLoader(
    const std::string& package_name,
    const std::string& plugin_name)
{
  const std::string full_node_plugin_name = package_name + "::" + plugin_name;
  RCLCPP_INFO(get_logger(), "want to load %s", full_node_plugin_name.c_str());
  std::shared_ptr<class_loader::ClassLoader> loader = nullptr;
  // get node plugin resource from package
  std::string content;
  std::string base_path;
  if (!ament_index_cpp::get_resource("node_plugin", package_name, content, &base_path)) {
    RCLCPP_ERROR(get_logger(), "Could not find requested resource in ament index: %s",
        package_name.c_str());
    return loader;
  }

  RCLCPP_INFO(get_logger(), "resources:\n%sbase path: %s", content.c_str(), base_path.c_str());
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
    RCLCPP_INFO(get_logger(), "candidate loader %s %s = %s",
        library_path.c_str(), class_name.c_str(), full_node_plugin_name.c_str());
    if (class_name != full_node_plugin_name) {
      continue;
    }
    RCLCPP_INFO(get_logger(), "found matching loader %s %s",  library_path.c_str(), class_name.c_str());

    try {
      loader = std::make_shared<class_loader::ClassLoader>(library_path);
      return loader;
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to load library: %s", ex.what());
      return loader;
    }
  }
  RCLCPP_ERROR(get_logger(), "Found no matching libraries: %s", full_node_plugin_name.c_str());
  return loader;
}

void NodeLoader::update()
{
  // it's ugly to have to call this here
  // core_->clean();
}

}  // namespace internal_pub_sub

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;

  auto node_loader = std::make_shared<internal_pub_sub::NodeLoader>();
  auto core = std::make_shared<internal_pub_sub::Core>();
  // TODO(lucasw) take __name & __ns parameters from command line
  node_loader->init("node_loader", "");
  node_loader->postInit(core);

  exec.add_node(node_loader);
  exec.spin();
  rclcpp::shutdown();
}

string package_name ""
string plugin_name ""
string node_name ""
string node_namespace ""
# fill in new_parameters with all the parameters
rcl_interfaces/Parameter[] parameters
internal_pub_sub/Remapping[] remappings
# TODO(lucasw) remappings
# if false then this is a regular rclcpp::Node, otherwise a internal_pub_sub::Node
bool internal_pub_sub False

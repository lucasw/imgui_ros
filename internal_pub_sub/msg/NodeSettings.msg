string package_name ""
string plugin_name ""
string node_name ""
string node_namespace ""
# fill in new_parameters with all the parameters
rcl_interfaces/Parameter[] parameters
internal_pub_sub/Remapping[] remappings
string[] arguments
# if false then this is a regular rclcpp::Node, otherwise a internal_pub_sub::Node
# TODO(lucasw) make this a uint8 with enumerated types instead.
bool internal_pub_sub False

std_msgs/Header header
# unique name, will replace any textured shape already received
string name
# needs to be unique out of all textures for cameras or shapes
string texture_name
bool add True
int32 width 128
int32 height 128
float32 aov_y 90.0  # aov_x is width / height * aov_y
# TODO(lucasw) not supported - set topic to anything to publish the
# rendered image out on that topic.
string topic ''
# TODO(lucasw) reduce update rate down from full frame rate
# int32 update_skip


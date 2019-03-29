# this frame id in header is in gl frame, used internally for tf lookups
std_msgs/Header header
# the optical frame reported on published messages
string header_frame_id
# unique name, will replace any Camera received
string name
# needs to be unique out of all textures for cameras or shapes
string texture_name
bool add  # True
# publish onto ros2 dds
bool ros_pub  # False
int32 width  # 128
int32 height  # 128
float32 aov_y  # 90.0
float32 aov_x  # 0.0  # aov_x is width / height * aov_y if <= 0.0
float32 near  # 0.1
float32 far  # 100.0
# publish the rendered image out on that topic.
string topic  # ''
# TODO(lucasw) reduce update rate down from full frame rate
# int32 update_skip
# TODO(lucasw) focal distance and aperture to model depth of field
# TODO(lucasw) later allow actual framerate, for now have to specify
# number to skip before next render.
uint8 skip  # 0

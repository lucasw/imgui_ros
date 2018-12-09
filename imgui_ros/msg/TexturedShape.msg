std_msgs/Header header
# unique name, will replace any textured shape already received
string name
# where the shape is relative to the header frame_id
# TODO(lucasw) or make this a TransformStamped, and have the position computed
# relative to the header frame id at the time in the TransformStamped,
# but then at later timesteps it remains fixed relative to the header frame_id?
# No should just have the caller figure that out.
geometry_msgs/Transform transform
shape_msgs/Mesh mesh
# colors for each vertice, needs to have 0, 1 (for all vertices same color),
# or same number as mesh has vertices
std_msgs/ColorRGBA[] colors
# the name of the texture, which needs to be stored in the receiver unless
# it is a topic or supplied below
string texture
# get the texture from an image topic specified in texture
bool is_topic false
# alternatively supply the texture directly, store with name above
sensor_msgs/Image image

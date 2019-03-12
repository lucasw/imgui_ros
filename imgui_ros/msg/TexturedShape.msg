std_msgs/Header header
# unique name, will replace any textured shape already received
string name
bool add  # True
# whether to draw this shape
bool enable  # True
# where the shape is relative to the header frame_id
# TODO(lucasw) or make this a TransformStamped, and have the position computed
# relative to the header frame id at the time in the TransformStamped,
# but then at later timesteps it remains fixed relative to the header frame_id?
# No should just have the caller figure that out.
# TODO(lucasw) this belongs in a ShapeInstance.msg, this msg should be
# just for the geometry to be stored but doesn't actually create an instance to be rendered.
imgui_ros/Vertex[] vertices
# List of triangles; the index values refer to positions in vertices[].
shape_msgs/MeshTriangle[] triangles
# the name of the texture, which needs to be stored in the receiver unless
# it is a topic or supplied below.
# If blank use solid colors.
string texture  # ""
# this texture corresponds to the shininess of the specular reflection
# at the same uv coordinates of the texture above.
string shininess_texture  # ""
string emission_texture # ""
# get the texture from an image topic specified in texture
bool is_topic  # false
# specify which set of already loaded shaders to use-
# This should map to consistent pair of vertex and fragment shaders
# (though could maybe specify those independently here)
# If blank use default shader 
string shader  # ""

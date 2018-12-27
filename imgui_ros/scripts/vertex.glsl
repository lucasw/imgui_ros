uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;
// TODO(lucasw) need multiples of these for each projector
// this is the model matrix of the object being drawn
// uniform mat4 projector_model_matrix;
uniform mat4 projector_view_matrix;
uniform mat4 projector_projection_matrix;

in vec3 Position;
in vec3 Normal;
in vec2 UV;
in vec4 Color;

out vec2 FraUV;
smooth out vec3 FraNormal;
out vec4 FraColor;
out vec4 ProjectedTexturePosition;
// The coordinate frame of this direction needs to be the same as the output normal
out vec3 projector_dir;

void main()
{
  FraUV = UV;
  FraColor = Color;
  // put normal into world frame
  FraNormal = (model_matrix * vec4(Normal, 1.0)).xyz -
        (model_matrix * vec4(0.0, 0.0, 0.0, 1.0)).xyz;
  // FraProjectorPosition =

  mat4 mvp = projection_matrix * view_matrix * model_matrix;
  gl_Position = mvp * vec4(Position.xyz, 1.0);

  mat4 projector_mvp = projector_projection_matrix * projector_view_matrix * model_matrix;
  ProjectedTexturePosition = projector_mvp * vec4(Position.xyz, 1.0);

  // put projector into world frame
  projector_dir = (transpose(projector_view_matrix) * vec4(0.0, 0.0, 1.0, 1.0) -
      transpose(projector_view_matrix) * vec4(0.0, 0.0, 0.0, 1.0)).xyz;
  // TODO(lucasw) in order to get the proper direction of the projector
  // this needs to be done in the fragment shader, where a per fragment
  // position is needed to get the direction from the projector origin.
  //projector_dir = -normalize((model_matrix * vec4(Position.xyz, 1.0) -
  //    transpose(projector_view_matrix) * vec4(0.0, 0.0, 0.0, 1.0)
  //    ).xyz);
}

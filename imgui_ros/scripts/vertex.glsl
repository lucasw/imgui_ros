const int MAX_PROJECTORS = 6;

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;
// the transpose of the projector view matrix times an xyz position relative
// to the projector should yield world coordinates
uniform mat4 projector_view_matrix[MAX_PROJECTORS];
uniform mat4 projector_view_matrix_inverse[MAX_PROJECTORS];
uniform mat4 projector_projection_matrix[MAX_PROJECTORS];

in vec3 Position;
in vec3 Normal;
in vec2 UV;
in vec4 Color;
in float shininess;

out vec2 FraUV;

smooth out vec3 FraNormal;
smooth out vec3 fragment_pos;

out vec4 FraColor;
out vec4 ProjectedTexturePosition[MAX_PROJECTORS];
// The coordinate frame of this direction needs to be the same as the output normal
out vec3 projector_pos[MAX_PROJECTORS];
out vec3 projector_dir[MAX_PROJECTORS];

void main()
{
  FraUV = UV;
  FraColor = Color;
  // put normal into world frame
  FraNormal = (model_matrix * vec4(Normal, 1.0)).xyz -
        (model_matrix * vec4(0.0, 0.0, 0.0, 1.0)).xyz;
  fragment_pos = (model_matrix * vec4(Position.xyz, 1.0)).xyz;

  mat4 mvp = projection_matrix * view_matrix * model_matrix;
  gl_Position = mvp * vec4(Position.xyz, 1.0);

  for (int i = 0; i < MAX_PROJECTORS; ++i) {
  // for (int i = 0; i < 1; ++i) {
    // TODO(lucasw) this is per-model so perhaps would be better in cpu
    mat4 projector_mvp = projector_projection_matrix[i] * projector_view_matrix[i] * model_matrix;
    // the vertex does get used here, so this needs to be in this shader
    ProjectedTexturePosition[i] = projector_mvp * vec4(Position.xyz, 1.0);

    // TODO(lucasw) this could be done outside of the vertex shader entirely,
    // it is redundant to calculate it over and over per vertex.
    // put projector into world frame
    vec4 origin = vec4(0.0, 0.0, 0.0, 1.0);
    vec4 z_axis = vec4(0.0, 0.0, 1.0, 1.0);

    // TODO(lucasw) instead pass in position and direction as calculated on cpu?
    // The inverse isn't used for anything else
    projector_pos[i] = (projector_view_matrix_inverse[i] * origin).xyz;
    projector_dir[i] = (projector_view_matrix_inverse[i] * z_axis).xyz - projector_pos[i];
  }
}

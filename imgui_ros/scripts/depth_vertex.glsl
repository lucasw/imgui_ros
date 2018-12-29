const int MAX_PROJECTORS = 4;

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

out vec2 FraUV;
smooth out vec3 FraNormal;
smooth out vec3 fragment_pos;
out vec4 FraColor;
out vec4 ProjectedTexturePosition[MAX_PROJECTORS];

void main()
{
  mat4 mvp = projection_matrix * view_matrix * model_matrix;
  gl_Position = mvp * vec4(Position.xyz, 1.0);
}

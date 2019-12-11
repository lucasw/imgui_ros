uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;

in vec3 Position;
in vec3 Normal;
// in vec2 UV;
// in vec4 Color;

// out vec2 FraUV;

smooth out vec3 fragment_normal;
// out vec4 fragment_color;

void main()
{
  // FraUV = UV;
  // fragment_color = Color;
  // put normal into world frame
  fragment_normal = (model_matrix * vec4(Normal, 1.0)).xyz -
        (model_matrix * vec4(0.0, 0.0, 0.0, 1.0)).xyz;

  mat4 mvp = projection_matrix * view_matrix * model_matrix;
  gl_Position = mvp * vec4(Position.xyz, 1.0);
}

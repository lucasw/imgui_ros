uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;
in vec3 Position;

void main()
{
  mat4 mvp = projection_matrix * view_matrix * model_matrix;
  gl_Position = mvp * vec4(Position.xyz, 1.0);
}

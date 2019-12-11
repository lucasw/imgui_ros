// OpenGL makes the first vec4 `out` the fragment color by default
// but should be explicit.
// 130
uniform samplerCube cube_map;

// TODO(lucasw)
// uniform sampler2D lens_normal_texture;
// in vec2 fragment_uv;

// TODO(lucasw) chromatic aberration?

// TODO(lucasw) could use color to have the edges get dim
// in vec4 fragment_color;
smooth in vec3 fragment_normal;

out vec4 fragment_color;

void main()
{
  // simple geometric normals to cube map lookup
  fragment_color.rgb = texture(cube_map, -fragment_normal).rgb;
}

// OpenGL makes the first vec4 `out` the fragment color by default
// but should be explicit.
// 130
const int MAX_PROJECTORS = 4;
uniform vec3 eye_pos;
uniform sampler2D Texture;
uniform int num_projectors;
uniform sampler2D ProjectedTexture[MAX_PROJECTORS];
// TODO(lucasw) use a struct?
uniform float projector_max_range[MAX_PROJECTORS];
uniform float projector_constant_attenuation[MAX_PROJECTORS];
// TODO(lucasw) linear attenuation seems the least physical
uniform float projector_linear_attenuation[MAX_PROJECTORS];
uniform float projector_quadratic_attenuation[MAX_PROJECTORS];

// TODO(lucasw) or go volumetric
uniform vec3 ambient;

// disable the projector entirely by setting this to zero
uniform float projected_texture_scale[MAX_PROJECTORS];

void main()
{
  // gl_FragDepth = gl_FragCoord.z;
}

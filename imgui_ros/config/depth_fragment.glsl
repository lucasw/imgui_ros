uniform sampler2D Texture;
uniform sampler2D projector_shadow_map;
out vec4 Out_Color;

void main()
{
  // TODO(lucasw) in nvidia need to access these texture otherwise too much optimization
  // happens and the depth fragment doesn't work?
  // Somewhere there are probably bound textures being used here meant for the default
  // shader.
  Out_Color = texture(Texture, vec2(0,0)) + texture(projector_shadow_map, vec2(0,0));
}

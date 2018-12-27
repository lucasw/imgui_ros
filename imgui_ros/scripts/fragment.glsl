// OpenGL makes the first vec4 `out` the fragment color by default
// but should be explicit.
// 130
const int MAX_PROJECTORS = 4;
uniform sampler2D Texture;
uniform sampler2D ProjectedTexture[MAX_PROJECTORS];
// disable the projector entirely by setting this to zero
uniform float projected_texture_scale[MAX_PROJECTORS];
in vec2 FraUV;
in vec4 FraColor;
smooth in vec3 FraNormal;
in vec3 projector_dir[4];
in vec4 ProjectedTexturePosition[4];
out vec4 Out_Color;
void main()
{
    Out_Color = FraColor * texture(Texture, FraUV.st);

    float enable_proj[MAX_PROJECTORS];
    vec2 uv[MAX_PROJECTORS];

    for (int i = 0; i < MAX_PROJECTORS; ++i) {
      vec4 projected_texture_position = ProjectedTexturePosition[i];
      // transform to clip space
      projected_texture_position.xyz /= projected_texture_position.w;
      projected_texture_position.xy += 0.5;
      projected_texture_position.z -= 1.0;
      enable_proj[i] = step(0.0, projected_texture_position.z);
      enable_proj[i] = enable_proj[i] * step(0.0, projected_texture_position.z);
      // this is a manual clamp to prevent any projection outside
      // of the view of the projector
      // TODO(lucasw) specify this elsewhere and
      // later make it changeable live.  May want to be able to disable this.
      enable_proj[i] *=
          (1.0 - step(1.0, projected_texture_position.x)) *
          step(0.0, projected_texture_position.x) *
          (1.0 - step(1.0, projected_texture_position.y)) *
          step(0.0, projected_texture_position.y);

      // TODO(lucasw) pass in max range and attenuation parameters

      // if normal is facing away from projector disable projection,
      // also dim the projection with diffuse reflection model.
      float projector_intensity = -dot(FraNormal, projector_dir[i]);
      enable_proj[i] = enable_proj[i] * projector_intensity * step(0.0, projector_intensity);

      // TODO(lwalter) can skip this if always border textures with alpha 0.0
      uv[i] = projected_texture_position.xy;
      // TODO(lucasw) need opengl 4.0 to do this within for loop
      // error: sampler arrays indexed with non-constant expressions are forbidden in GLSL 1.30 and later
 
      // OutColor += enable_proj[i] * projected_texture_scale[i] * texture(ProjectedTexture[i], uv[i].st);
   }
   Out_Color +=
      enable_proj[0] * projected_texture_scale[0] * texture(ProjectedTexture[0], uv[0].st) +
      enable_proj[1] * projected_texture_scale[1] * texture(ProjectedTexture[1], uv[1].st) +
      enable_proj[2] * projected_texture_scale[2] * texture(ProjectedTexture[2], uv[2].st) +
      enable_proj[3] * projected_texture_scale[3] * texture(ProjectedTexture[3], uv[3].st);
}

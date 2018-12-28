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
in vec2 FraUV;
in vec4 FraColor;
smooth in vec3 FraNormal;
smooth in vec3 fragment_pos;
// TODO(lucasw) use a struct?
in vec3 projector_pos[MAX_PROJECTORS];
in vec3 projector_dir[MAX_PROJECTORS];
in vec4 ProjectedTexturePosition[MAX_PROJECTORS];
out vec4 Out_Color;
void main()
{
    vec3 view_ray = normalize(fragment_pos - eye_pos);

    Out_Color = FraColor * texture(Texture, FraUV.st);

    float enable_proj[MAX_PROJECTORS];
    vec2 uv[MAX_PROJECTORS];

    float luminosity[MAX_PROJECTORS];
    float specular[MAX_PROJECTORS];

    for (int i = 0; i < MAX_PROJECTORS; ++i) {
    // for (int i = 0; i < 1; ++i) {
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
          (step(projected_texture_position.x, 1.0)) *
          step(0.0, projected_texture_position.x) *
          (step(projected_texture_position.y, 1.0)) *
          step(0.0, projected_texture_position.y);

      // modify projected image based on distance to projector
      vec3 proj_ray = fragment_pos - projector_pos[i];
      float dist = length(proj_ray);
      proj_ray /= dist;
      // TODO(lucasw) pass in max range and attenuation parameters
      enable_proj[i] *= (projector_max_range[i] == 0.0) ? 1.0 : step(dist, projector_max_range[i]);

      // if normal is facing away from projector disable projection,
      // also dim the projection with diffuse reflection model.
      // float projector_intensity = -dot(FraNormal, projector_dir[i]);
      float normal_light_alignment = -dot(FraNormal, proj_ray);
      // float clipped_alignment *= step(0.0, alignment);

      // TODO(lwalter) can skip this if always border textures with alpha 0.0
      uv[i] = projected_texture_position.xy;

      // the base color
      // then attenuate
      float attenuation = projector_constant_attenuation[i] +
          projector_linear_attenuation[i] * dist +
          projector_quadratic_attenuation[i] * dist * dist;
      // set luminosity to 1.0 if attenuation is 0.0
      attenuation = attenuation == 0.0 ? 1.0 : attenuation;
      float scaled_attenuated = enable_proj[i] * projected_texture_scale[i] * 1.0 / attenuation;
      luminosity[i] = normal_light_alignment * scaled_attenuated;

      //////////////////////////////
      // specular
      // the surface (diffuse) texture doesn't matter at all for specular
      // TODO(lucasw) normalize is redundant?
      vec3 reflected_proj_ray = normalize(proj_ray + 2.0 * normal_light_alignment * FraNormal);
      float specular_intensity = dot(reflected_proj_ray, -view_ray);
      specular_intensity *= step(0.0, specular_intensity);
      // TODO(lucasw) later per-vertex and specular maps,
      // also try uniform connected to gui slider
      float shininess = 25.5;
      specular_intensity = pow(specular_intensity, shininess);
      specular[i] = specular_intensity * scaled_attenuated * step(0.0, normal_light_alignment);
      // specular[i] *= step(0.0, specular[i]);
      // TEMP debug
      // Out_Color.rgb = view_ray;
      // Out_Color.rgb += specular[i] * enable_proj[i] * step(0.0, alignment);
      // Out_Color.rgb += reflected_proj_ray * enable_proj[i] * step(0.0, alignment);
      // Out_Color.rgb += proj_ray * enable_proj[i] * step(0.0, alignment);
   }

   // TODO(lucasw) need opengl 4.0 to do this within for loop
   // error: sampler arrays indexed with non-constant expressions are forbidden in GLSL 1.30 and later
   vec3 proj_light[4];
   proj_light[0] = texture(ProjectedTexture[0], uv[0].st).rgb;
   proj_light[1] = texture(ProjectedTexture[1], uv[1].st).rgb;
   proj_light[2] = texture(ProjectedTexture[2], uv[2].st).rgb;
   proj_light[3] = texture(ProjectedTexture[3], uv[3].st).rgb;

   // pure white for now
   vec3 specular_color = vec3(1.0, 1.0, 1.0);

   vec3 total_luminosity = vec3(0.0, 0.0, 0.0);
   vec3 total_specular = vec3(0.0, 0.0, 0.0);
   // TODO(lucasw) having problems with adding invalid projector light,
   // everything turns black
   for (int i = 0; i < MAX_PROJECTORS && i < num_projectors; ++i) {
   // for (int i = 0; i < MAX_PROJECTORS; ++i) {
      vec3 diffuse_light = luminosity[i] * proj_light[i];
      // diffuse_light *= step(0.0, diffuse_light);
      total_luminosity += diffuse_light;

      vec3 specular_light = specular[i] * specular_color[i] * proj_light[i];
      // specular_light *= step(0.0, specular_light);
      total_specular += specular_light;
   }
   // add a little luminosity regardless of surface color, a bright enough light
   // ought to turn white on any surface.
   Out_Color.rgb = Out_Color.rgb * (ambient + total_luminosity) +
      total_specular + total_luminosity * 0.01;

  // debug
  // Out_Color.rgb = abs(eye_pos) * 1.0;
}

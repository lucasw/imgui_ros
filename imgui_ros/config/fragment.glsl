// OpenGL makes the first vec4 `out` the fragment color by default
// but should be explicit.
// 130

// TODO(lucasw) ought to be able to eliminate these with the right vertex shader
uniform float near_clip;
uniform float far_clip;

// TEMP debug
uniform samplerCube cube_map;

const int MAX_PROJECTORS = 6;
uniform vec3 eye_pos;
uniform sampler2D Texture;
uniform sampler2D shininess_texture;
uniform sampler2D emission_texture;
uniform int num_projectors;
uniform sampler2D ProjectedTexture[MAX_PROJECTORS];
uniform sampler2D projector_shadow_map[MAX_PROJECTORS];
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

float color_to_gray(vec4 color)
{
  return dot(vec3(0.3, 0.59, 0.11), color.rgb);
  // return dot(vec3(1.0/3.0, 1.0/3.0, 1.0/3.0), color.rgb);
  // return max(color.r, max(color.g, color.b));
}
vec4 color_to_gray_color(vec4 color)
{
  float gray = color_to_gray(color);
  return vec4(gray, gray, gray, color.a);
}

void main()
{
    vec3 view_ray = normalize(fragment_pos - eye_pos);

    Out_Color = FraColor * texture(Texture, FraUV.st);

    // the lookup will produce 0,1.0 value, but 0,100.0 ought to be good
    // TODO(lucasw) later scale the shininess value (which probably
    // originated from 8-bit texture 0 - 255, want to map those numbers
    // nonlinearly to get more resolution at the lower end?
    const float shiny_scale = 512.0;
    // TODO(lucasw) use a GL_LUMINANCE texture
    // the shininess_texture
    float shininess = (1.0 - color_to_gray(texture(shininess_texture, FraUV.st)));

    float enable_proj[MAX_PROJECTORS];
    vec2 uv[MAX_PROJECTORS];

    float luminosity[MAX_PROJECTORS];
    float specular[MAX_PROJECTORS];

    float scaled_dist[MAX_PROJECTORS];

    // TODO(lucasw) is it easier to always loop MAX_PROJECTORS times?
    for (int i = 0; i < MAX_PROJECTORS && i < num_projectors; ++i) {
      // the position of the fragment in the projector projection space
      vec4 pos_in_projector_space = ProjectedTexturePosition[i];

      // TODO(lucasw) want to convert either the depth texture value (which
      // has the depth compressed into 0.0 - 1.0) to camera space range,
      // or convert this distance here to the compressed depth range,
      // so they can be compared.
      // z = -1.0 is 1.0 units away, it is linear
      // scaled_dist[i] = (pos_in_projector_space.z);  // / pos_in_projector_space.w;
      // z and w seem to be equal, meaning the division by w has already occurred for z?
      // scaled_dist[i] = 0.1 * (pos_in_projector_space.z / pos_in_projector_space.w);
      // scaled_dist[i] *= step(0.0, scaled_dist[i]);

      // transform to clip space / normalized device coordinates
      pos_in_projector_space.xyz /= pos_in_projector_space.w;
      // then transform from -1.0 to 1.0 to 0 to 1.0 to use in texture lookup
      pos_in_projector_space.xyz += 1.0;
      pos_in_projector_space.xyz *= 0.5;
      // need to reverse z direction?
      // pos_in_projector_space.z = 1.0 - pos_in_projector_space.z;

      float enable_z = (step(pos_in_projector_space.z, 1.0)) *
                        step(0.0, pos_in_projector_space.z);
      float nonlinear_z = pos_in_projector_space.z;
      scaled_dist[i] = nonlinear_z;

      // is the fragment in front of the light?  Otherwise disable
      enable_proj[i] = 1.0;  // step(0.0, pos_in_projector_space.z);
      // enable_proj[i] = enable_proj[i] * step(0.0, pos_in_projector_space.z);
      // this is a manual clamp to prevent any projection outside
      // of the view of the projector
      // TODO(lucasw) specify this elsewhere and
      // later make it changeable live.  May want to be able to disable this.
      enable_proj[i] *=
          (step(pos_in_projector_space.x, 1.0)) *
          step(0.0, pos_in_projector_space.x) *
          (step(pos_in_projector_space.y, 1.0)) *
          step(0.0, pos_in_projector_space.y) *
          enable_z;

      // TODO(lucasw) proj_ray.z is not the same as the distance in the shadow map
      // modify projected image based on distance to projector
      vec3 proj_ray = fragment_pos - projector_pos[i];
      float dist = length(proj_ray);
      // TEMP debug
      // scaled_dist[i] = dist;

      proj_ray /= dist;
      // TODO(lucasw) pass in max range and attenuation parameters
      // enable_proj[i] *= (projector_max_range[i] == 0.0) ? 1.0 : step(dist, projector_max_range[i]);

      // if normal is facing away from projector disable projection,
      // also dim the projection with diffuse reflection model.
      float normal_light_alignment = -dot(FraNormal, proj_ray);

      // TODO(lwalter) can skip this if always border textures with alpha 0.0
      uv[i] = pos_in_projector_space.xy;

      // the base color
      // then attenuate
      float attenuation = projector_constant_attenuation[i] +
          projector_linear_attenuation[i] * dist +
          projector_quadratic_attenuation[i] * dist * dist;
      // set luminosity to 1.0 if attenuation is 0.0
      attenuation = attenuation == 0.0 ? 1.0 : attenuation;
      float scaled_attenuated = enable_proj[i] * projected_texture_scale[i] * 1.0 / attenuation;
      scaled_attenuated *= step(0.0, normal_light_alignment);
      luminosity[i] = normal_light_alignment * scaled_attenuated;  // * clip_light;

      //////////////////////////////
      // specular
      // the surface (diffuse) texture doesn't matter at all for specular
      // TODO(lucasw) normalize is redundant?
      vec3 reflected_proj_ray = normalize(proj_ray + 2.0 * normal_light_alignment * FraNormal);
      float specular_intensity = dot(reflected_proj_ray, -view_ray);
      specular_intensity *= step(0.0, specular_intensity);
      specular_intensity *= step(0.0, normal_light_alignment);
      // TODO(lucasw) later per-vertex and specular maps,
      // also try uniform connected to gui slider
      specular_intensity = pow(specular_intensity, 1.0 + shininess * shiny_scale);
      specular[i] = specular_intensity * scaled_attenuated;  // * clip_light;
      // specular[i] *= step(0.0, specular[i]);
      // TEMP debug
      // Out_Color.rgb = view_ray;
      // Out_Color.rgb += specular[i] * enable_proj[i] * step(0.0, alignment);
      // Out_Color.rgb += reflected_proj_ray * enable_proj[i] * step(0.0, alignment);
      // Out_Color.rgb += proj_ray * enable_proj[i] * step(0.0, alignment);
   }

   // TODO(lucasw) need opengl 4.0 to do this within for loop
   // error: sampler arrays indexed with non-constant expressions are forbidden in GLSL 1.30 and later
   vec3 proj_light[MAX_PROJECTORS];
   proj_light[0] = texture(ProjectedTexture[0], uv[0].st).rgb;
   proj_light[1] = texture(ProjectedTexture[1], uv[1].st).rgb;
   proj_light[2] = texture(ProjectedTexture[2], uv[2].st).rgb;
   proj_light[3] = texture(ProjectedTexture[3], uv[3].st).rgb;
   proj_light[4] = texture(ProjectedTexture[4], uv[4].st).rgb;
   proj_light[5] = texture(ProjectedTexture[5], uv[5].st).rgb;

   float shadow_dist[MAX_PROJECTORS];
   shadow_dist[0] = texture(projector_shadow_map[0], uv[0].st).r;
   shadow_dist[1] = texture(projector_shadow_map[1], uv[1].st).r;
   shadow_dist[2] = texture(projector_shadow_map[2], uv[2].st).r;
   shadow_dist[3] = texture(projector_shadow_map[3], uv[3].st).r;
   shadow_dist[4] = texture(projector_shadow_map[4], uv[4].st).r;
   shadow_dist[5] = texture(projector_shadow_map[5], uv[5].st).r;

   // pure white for now
   vec3 specular_color = vec3(1.0, 1.0, 1.0);

   vec3 total_luminosity = vec3(0.0, 0.0, 0.0);
   vec3 total_specular = vec3(0.0, 0.0, 0.0);
   // TODO(lucasw) having problems with adding invalid projector light,
   // everything turns black
   for (int i = 0; i < MAX_PROJECTORS && i < num_projectors; ++i) {
      // TODO(lucasw) These need to be gotten from glRange(), not the same as perspective near far
      // (do they map to min and max in depth texture?)
      float near = 0.0;
      float far = 1.0;
      float z = (2.0 * shadow_dist[i] - near - far) / (far - near);
      float linear_shadow_dist = (2.0 * near_clip * far_clip) /
          (far_clip + near_clip - z * (far_clip - near_clip));

      // float not_shadowed = step(scaled_dist[i], linear_shadow_dist);  //  * 0.999);
      float not_shadowed = step(scaled_dist[i], shadow_dist[i]);  //  * 0.999);

      vec3 diffuse_light = luminosity[i] * proj_light[i] * not_shadowed;
      // // diffuse_light *= step(0.0, diffuse_light);
      total_luminosity += diffuse_light;
      // debug
      // total_luminosity += linear_dist * enable_proj[i] * vec3(1.0, 1.0, 1.0);

      // specular[] has enable_proj applied to it above.
      vec3 specular_light = specular[i] * proj_light[i] * not_shadowed;
      // specular_light.r = shadow_dist[i] * 1.0 * enable_proj[i];  // 1.0 - not_shadowed;
      // specular_light.g = 0.25 * enable_proj[i] * (1.0 - not_shadowed);
      // specular_light *= step(0.0, specular_light);
      total_specular += specular_light;
   }
   // add a little luminosity regardless of surface color, a bright enough light
   // ought to turn white on any surface.
   Out_Color.rgb = Out_Color.rgb * (ambient + total_luminosity) +
       total_specular + total_luminosity * 0.01;

   Out_Color.rgb = clamp(Out_Color.rgb, 0.0, 1.0);

   Out_Color.rgb += (FraColor * texture(emission_texture, FraUV.st)).rgb;
   // TEMP debug
   // Out_Color.rgb = texture(ProjectedTexture[0], uv[0].st).rgb;
   // Out_Color.r = (vec3(1.0, 1.0, 1.0) * shininess).r;
   // Out_Color.rgb = vec3(1.0, 1.0, 1.0) * total_luminosity;
  // Out_Color.rgb = abs(eye_pos) * 1.0;
  // Out_Color.rgb = texture(Texture, FraNormal.xy).rgb;
  // Out_Color.rgb += texture(cube_map, FraNormal).rgb;
  // Out_Color.rgb = texture(cube_map, vec3(0.0, 0.0, 1.0)).rgb;
  // Out_Color.rgb += texture(cube_map, FraNormal).rgb;
}

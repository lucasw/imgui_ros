// OpenGL makes the first vec4 `out` the fragment color by default
// but should be explicit.
// 130
uniform sampler2D Texture;
uniform sampler2D ProjectedTexture;
uniform float projected_texture_scale;
in vec2 FraUV;
in vec4 FraColor;
smooth in vec3 FraNormal;
in vec3 projector_dir;
in vec4 ProjectedTexturePosition;
out vec4 Out_Color;
void main()
{
    vec4 projected_texture_position = ProjectedTexturePosition;
    // transform to clip space
    projected_texture_position.xyz /= projected_texture_position.w;
    projected_texture_position.xy += 0.5;
    projected_texture_position.z -= 1.0;
    float enable_proj = step(0.0, projected_texture_position.z);
    // TODO(lucasw) this is a manual clamp, can specify this elsewhere and
    // later make it changeable live.
    enable_proj = enable_proj * step(0.0, projected_texture_position.z); /* *
        (1.0 - step(1.0, projected_texture_position.x)) *
        step(0.0, projected_texture_position.x) *
        (1.0 - step(1.0, projected_texture_position.y)) *
        step(0.0, projected_texture_position.y);
    */

    // if normal is facing away from projector disable projection

    // TODO
    float projector_intensity = -dot(FraNormal, projector_dir);
    enable_proj = enable_proj * projector_intensity * step(0.0, projector_intensity);

    // vec3 in_proj_vec = step(0.0, ProjectedTexturePosition.xyz) * (1.0 - step(1.0, ProjectedTexturePosition.xyz));
    // TODO(lwalter) can skip this if always border textures with alpha 0.0
    // float in_proj_bounds = normalize(dot(in_proj_vec, in_proj_vec));
    // Out_Color = FraColor * texture(Texture, FraUV.st) + in_proj_bounds * texture(ProjectedTexture, ProjectedTexturePosition.xy);
    vec2 uv = projected_texture_position.xy;
    // uv.t = 1.0 - uv.t;
    Out_Color = FraColor * texture(Texture, FraUV.st) + enable_proj * projected_texture_scale * texture(ProjectedTexture, uv.st);
}

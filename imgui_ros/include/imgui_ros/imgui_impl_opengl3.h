// dear imgui: Renderer for OpenGL3 / OpenGL ES2 / OpenGL ES3 (modern OpenGL with shaders / programmatic pipeline)
// This needs to be used along with a Platform Binding (e.g. GLFW, SDL, Win32, custom..)
// (Note: We are using GL3W as a helper library to access OpenGL functions since there is no standard header to access modern OpenGL functions easily. Alternatives are GLEW, Glad, etc..)

// Implemented features:
//  [X] Renderer: User texture binding. Use 'GLuint' OpenGL texture identifier as void*/ImTextureID. Read the FAQ about ImTextureID in imgui.cpp.

// You can copy and use unmodified imgui_impl_* files in your project. See main.cpp for an example of using this.
// If you are new to dear imgui, read examples/README.txt and read the documentation at the top of imgui.cpp.
// https://github.com/ocornut/imgui

// About OpenGL function loaders: 
// About OpenGL function loaders: modern OpenGL doesn't have a standard header file and requires individual function pointers to be loaded manually. 
// Helper libraries are often used for this purpose! Here we are supporting a few common ones: gl3w, glew, glad. 
// You may use another loader/header of your choice (glext, glLoadGen, etc.), or chose to manually implement your own.

// About GLSL version:
// The 'glsl_version' initialization parameter should be NULL (default) or a "#version XXX" string.
// On computer platform the GLSL version default to "#version 130". On OpenGL ES 3 platform it defaults to "#version 300 es"
// Only override if your GL version doesn't handle this GLSL version. See GLSL version table at the top of imgui_impl_opengl3.cpp.

#ifndef IMGUI_IMPL_OPENGL_H
#define IMGUI_IMPL_OPENGL_H

// Set default OpenGL loader to be gl3w
#if !defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)     \
 && !defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)     \
 && !defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)     \
 && !defined(IMGUI_IMPL_OPENGL_LOADER_CUSTOM)
#define IMGUI_IMPL_OPENGL_LOADER_GL3W
#endif

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"

// iOS, Android and Emscripten can use GL ES 3
// Call ImGuiImplOpenGL3::Init() with "#version 300 es"
#if (defined(__APPLE__) && TARGET_OS_IOS) || (defined(__ANDROID__)) || (defined(__EMSCRIPTEN__))
#define USE_GL_ES3
#endif

#ifdef USE_GL_ES3
// OpenGL ES 3
#include <GLES3/gl3.h>  // Use GL ES 3
#else
// Regular OpenGL
// About OpenGL function loaders: modern OpenGL doesn't have a standard header file and requires individual function pointers to be loaded manually. 
// Helper libraries are often used for this purpose! Here we are supporting a few common ones: gl3w, glew, glad.
// You may use another loader/header of your choice (glext, glLoadGen, etc.), or chose to manually implement your own.
#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
#include <GL/gl3w.h>
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
#include <GL/glew.h>
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
#include <glad/glad.h>
#else
#include IMGUI_IMPL_OPENGL_LOADER_CUSTOM
#endif
#endif

#pragma GCC diagnostic pop

#include <string>

bool CheckShader(GLuint handle, const char* desc);
bool CheckProgram(GLuint handle, const char* desc);

void checkGLError(const std::string file, const int line);

struct GLState
{
  GLenum last_active_texture;
  GLint last_program;
  GLint last_texture;
  GLint last_sampler;
  GLint last_array_buffer;
  GLint last_vertex_array;
  GLint last_polygon_mode[2];
  GLint last_viewport[4];
  GLint last_scissor_box[4];
  GLenum last_blend_src_rgb;
  GLenum last_blend_dst_rgb;
  GLenum last_blend_src_alpha;
  GLenum last_blend_dst_alpha;
  GLenum last_blend_equation_rgb;
  GLenum last_blend_equation_alpha;
  GLboolean last_enable_blend;
  GLboolean last_enable_cull_face;
  GLboolean last_enable_depth_test;
  GLboolean last_enable_scissor_test;

  void backup()
  {
    glGetIntegerv(GL_ACTIVE_TEXTURE, (GLint*)&last_active_texture);
    glActiveTexture(GL_TEXTURE0);
    glGetIntegerv(GL_CURRENT_PROGRAM, &last_program);
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture);
#ifdef GL_SAMPLER_BINDING
    glGetIntegerv(GL_SAMPLER_BINDING, &last_sampler);
#endif
    glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &last_array_buffer);
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &last_vertex_array);
#ifdef GL_POLYGON_MODE
    glGetIntegerv(GL_POLYGON_MODE, last_polygon_mode);
#endif
    glGetIntegerv(GL_VIEWPORT, last_viewport);

    glGetIntegerv(GL_SCISSOR_BOX, last_scissor_box);
    glGetIntegerv(GL_BLEND_SRC_RGB, (GLint*)&last_blend_src_rgb);
    glGetIntegerv(GL_BLEND_DST_RGB, (GLint*)&last_blend_dst_rgb);
    glGetIntegerv(GL_BLEND_SRC_ALPHA, (GLint*)&last_blend_src_alpha);
    glGetIntegerv(GL_BLEND_DST_ALPHA, (GLint*)&last_blend_dst_alpha);
    glGetIntegerv(GL_BLEND_EQUATION_RGB, (GLint*)&last_blend_equation_rgb);
    glGetIntegerv(GL_BLEND_EQUATION_ALPHA, (GLint*)&last_blend_equation_alpha);
    last_enable_blend = glIsEnabled(GL_BLEND);
    last_enable_cull_face = glIsEnabled(GL_CULL_FACE);
    last_enable_depth_test = glIsEnabled(GL_DEPTH_TEST);
    last_enable_scissor_test = glIsEnabled(GL_SCISSOR_TEST);
  }

  void restore()
  {
    glUseProgram(last_program);
    glBindTexture(GL_TEXTURE_2D, last_texture);
#ifdef GL_SAMPLER_BINDING
    glBindSampler(0, last_sampler);
#endif
    glActiveTexture(last_active_texture);
    glBindVertexArray(last_vertex_array);
    glBindBuffer(GL_ARRAY_BUFFER, last_array_buffer);
    glBlendEquationSeparate(last_blend_equation_rgb, last_blend_equation_alpha);
    glBlendFuncSeparate(last_blend_src_rgb, last_blend_dst_rgb, last_blend_src_alpha, last_blend_dst_alpha);
    if (last_enable_blend) glEnable(GL_BLEND); else glDisable(GL_BLEND);
    if (last_enable_cull_face) glEnable(GL_CULL_FACE); else glDisable(GL_CULL_FACE);
    if (last_enable_depth_test) glEnable(GL_DEPTH_TEST); else glDisable(GL_DEPTH_TEST);
    if (last_enable_scissor_test) glEnable(GL_SCISSOR_TEST); else glDisable(GL_SCISSOR_TEST);
#ifdef GL_POLYGON_MODE
    glPolygonMode(GL_FRONT_AND_BACK, (GLenum)last_polygon_mode[0]);
#endif
    glViewport(last_viewport[0], last_viewport[1], (GLsizei)last_viewport[2], (GLsizei)last_viewport[3]);
    glScissor(last_scissor_box[0], last_scissor_box[1], (GLsizei)last_scissor_box[2], (GLsizei)last_scissor_box[3]);
  }
};
class ImGuiImplOpenGL3
{
public:
  // static bool CheckShader(GLuint handle, const char* desc);

  bool Init(const char* glsl_version = NULL);
  void Shutdown();
  void NewFrame();
  void RenderDrawData(ImDrawData* draw_data);

  // Called by Init/NewFrame/Shutdown
  bool CreateFontsTexture();
  void DestroyFontsTexture();
  bool CreateDeviceObjects();
  void DestroyDeviceObjects();

  char GlslVersionString[32] = "";
  GLuint FontTexture = 0;
  GLuint shader_handle_ = 0;
  GLuint vert_handle_ = 0;
  GLuint frag_handle_ = 0;
  int attrib_location_tex_ = 0;
  int attrib_location_proj_mtx_ = 0;
  int attrib_location_position_ = 0;
  int attrib_location_uv_ = 0;
  int attrib_location_color_ = 0;
  unsigned int vbo_handle_ = 0;
  unsigned int elements_handle_ = 0;
};

#endif  // IMGUI_IMPL_OPENGL_H

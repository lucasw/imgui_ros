// dear imgui: Renderer for OpenGL3 / OpenGL ES2 / OpenGL ES3 (modern OpenGL with shaders / programmatic pipeline)
// This needs to be used along with a Platform Binding (e.g. GLFW, SDL, Win32, custom..)
// (Note: We are using GL3W as a helper library to access OpenGL functions since there is no standard header to access modern OpenGL functions easily. Alternatives are GLEW, Glad, etc..)

// Implemented features:
//  [X] Renderer: User texture binding. Use 'GLuint' OpenGL texture identifier as void*/ImTextureID. Read the FAQ about ImTextureID in imgui.cpp.

// You can copy and use unmodified imgui_impl_* files in your project. See main.cpp for an example of using this.
// If you are new to dear imgui, read examples/README.txt and read the documentation at the top of imgui.cpp.
// https://github.com/ocornut/imgui

// CHANGELOG
// (minor and older changes stripped away, please see git history for details)
//  2018-08-29: OpenGL: Added support for more OpenGL loaders: glew and glad, with comments indicative that any loader can be used.
//  2018-08-09: OpenGL: Default to OpenGL ES 3 on iOS and Android. GLSL version default to "#version 300 ES".
//  2018-07-30: OpenGL: Support for GLSL 300 ES and 410 core. Fixes for Emscripten compilation.
//  2018-07-10: OpenGL: Support for more GLSL versions (based on the GLSL version string). Added error output when shaders fail to compile/link.
//  2018-06-08: Misc: Extracted imgui_impl_opengl3.cpp/.h away from the old combined GLFW/SDL+OpenGL3 examples.
//  2018-06-08: OpenGL: Use draw_data->DisplayPos and draw_data->DisplaySize to setup projection matrix and clipping rectangle.
//  2018-05-25: OpenGL: Removed unnecessary backup/restore of GL_ELEMENT_ARRAY_BUFFER_BINDING since this is part of the VAO state.
//  2018-05-14: OpenGL: Making the call to glBindSampler() optional so 3.2 context won't fail if the function is a NULL pointer.
//  2018-03-06: OpenGL: Added const char* glsl_version parameter to ImGuiImplOpenGL3::Init() so user can override the GLSL version e.g. "#version 150".
//  2018-02-23: OpenGL: Create the VAO in the render function so the setup can more easily be used with multiple shared GL context.
//  2018-02-16: Misc: Obsoleted the io.RenderDrawListsFn callback and exposed ImGui_ImplSdlGL3_RenderDrawData() in the .h file so you can call it yourself.
//  2018-01-07: OpenGL: Changed GLSL shader version from 330 to 150.
//  2017-09-01: OpenGL: Save and restore current bound sampler. Save and restore current polygon mode.
//  2017-05-01: OpenGL: Fixed save and restore of current blend func state.
//  2017-05-01: OpenGL: Fixed save and restore of current GL_ACTIVE_TEXTURE.
//  2016-09-05: OpenGL: Fixed save and restore of current scissor rectangle.
//  2016-07-29: OpenGL: Explicitly setting GL_UNPACK_ROW_LENGTH to reduce issues because SDL changes it. (#752)

//----------------------------------------
// OpenGL    GLSL      GLSL
// version   version   string
//----------------------------------------
//  2.0       110       "#version 110"
//  2.1       120
//  3.0       130
//  3.1       140
//  3.2       150       "#version 150"
//  3.3       330
//  4.0       400
//  4.1       410       "#version 410 core"
//  4.2       420
//  4.3       430
//  ES 2.0    100       "#version 100"
//  ES 3.0    300       "#version 300 es"
//----------------------------------------

#if defined(_MSC_VER) && !defined(_CRT_SECURE_NO_WARNINGS)
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <iostream>
#include "imgui.h"
#include <imgui_ros/imgui_impl_opengl3.h>
#include <stdio.h>
#if defined(_MSC_VER) && _MSC_VER <= 1500 // MSVC 2008 or earlier
#include <stddef.h>     // intptr_t
#else
#include <stdint.h>     // intptr_t
#endif
#if defined(__APPLE__)
#include "TargetConditionals.h"
#endif

#include <iostream>

void checkGLError(const std::string file, const int line)
{
  while (true) {
    GLenum error = glGetError();
    std::string msg;
    if (error == GL_NO_ERROR)
      return;
    else if (error == GL_INVALID_ENUM)
      msg = "invalid enum";
    else if (error == GL_INVALID_ENUM)
      msg = "invalid enum";
    else if (error == GL_INVALID_VALUE)
      msg = "invalid value";
    else if (error == GL_INVALID_OPERATION)
      msg = "invalid operation";
    else if (error == GL_INVALID_FRAMEBUFFER_OPERATION)
      msg = "invalid framebuffer operation";
    else if (error == GL_OUT_OF_MEMORY)
      msg = "out of memory";
    std::cerr << file << ":" << line << " gl error " << msg << " " << error << "\n";
  }
}


// Functions
bool    ImGuiImplOpenGL3::Init(const char* glsl_version)
{
    // Store GLSL version string so we can refer to it later in case we recreate shaders. Note: GLSL version is NOT the same as GL version. Leave this to NULL if unsure.
#ifdef USE_GL_ES3
    if (glsl_version == NULL)
        glsl_version = "#version 300 es";
#else
    if (glsl_version == NULL)
        glsl_version = "#version 130";
#endif
    IM_ASSERT((int)strlen(glsl_version) + 2 < IM_ARRAYSIZE(GlslVersionString));
    strcpy(GlslVersionString, glsl_version);
    strcat(GlslVersionString, "\n");
    return true;
}

void    ImGuiImplOpenGL3::Shutdown()
{
    ImGuiImplOpenGL3::DestroyDeviceObjects();
}

void    ImGuiImplOpenGL3::NewFrame()
{
    if (!FontTexture)
        ImGuiImplOpenGL3::CreateDeviceObjects();
}

// OpenGL3 Render function.
// (this used to be set in io.RenderDrawListsFn and called by ImGui::Render(), but you can now call this directly from your main loop)
// Note that this implementation is little overcomplicated because we are saving/setting up/restoring every OpenGL state explicitly, in order to be able to run within any OpenGL engine that doesn't do so.
void    ImGuiImplOpenGL3::RenderDrawData(ImDrawData* draw_data)
{
    // Avoid rendering when minimized, scale coordinates for retina displays (screen coordinates != framebuffer coordinates)
    ImGuiIO& io = ImGui::GetIO();
    int fb_width = (int)(draw_data->DisplaySize.x * io.DisplayFramebufferScale.x);
    int fb_height = (int)(draw_data->DisplaySize.y * io.DisplayFramebufferScale.y);
    if (fb_width <= 0 || fb_height <= 0)
        return;
    draw_data->ScaleClipRects(io.DisplayFramebufferScale);
    // Setup render state: alpha-blending enabled, no face culling, no depth testing, scissor enabled, polygon fill
    glEnable(GL_BLEND);
    glBlendEquation(GL_FUNC_ADD);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_CULL_FACE);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_SCISSOR_TEST);
#ifdef GL_POLYGON_MODE
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
#endif

    GLState gl_state;
    gl_state.backup();

    // Setup viewport, orthographic projection matrix
    // Our visible imgui space lies from draw_data->DisplayPos (top left) to draw_data->DisplayPos+data_data->DisplaySize (bottom right). DisplayMin is typically (0,0) for single viewport apps.
    glViewport(0, 0, (GLsizei)fb_width, (GLsizei)fb_height);
    float L = draw_data->DisplayPos.x;
    float R = draw_data->DisplayPos.x + draw_data->DisplaySize.x;
    float T = draw_data->DisplayPos.y;
    float B = draw_data->DisplayPos.y + draw_data->DisplaySize.y;
    const float ortho_projection[4][4] =
    {
        { 2.0f/(R-L),   0.0f,         0.0f,   0.0f },
        { 0.0f,         2.0f/(T-B),   0.0f,   0.0f },
        { 0.0f,         0.0f,        -1.0f,   0.0f },
        { (R+L)/(L-R),  (T+B)/(B-T),  0.0f,   1.0f },
    };
    glUseProgram(shader_handle_);
    glUniform1i(attrib_location_tex_, 0);
    glUniformMatrix4fv(attrib_location_proj_mtx_, 1, GL_FALSE, &ortho_projection[0][0]);
#ifdef GL_SAMPLER_BINDING
    glBindSampler(0, 0); // We use combined texture/sampler state. Applications using GL 3.3 may set that otherwise.
#endif
    // Recreate the VAO every time
    // (This is to easily allow multiple GL contexts. VAO are not shared among GL contexts, and we don't track creation/deletion of windows so we don't have an obvious key to use to cache them.)
    GLuint vao_handle = 0;
    glGenVertexArrays(1, &vao_handle);
    glBindVertexArray(vao_handle);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_handle_);
    glEnableVertexAttribArray(attrib_location_position_);
    glEnableVertexAttribArray(attrib_location_uv_);
    glEnableVertexAttribArray(attrib_location_color_);
    glVertexAttribPointer(attrib_location_position_, 2, GL_FLOAT, GL_FALSE, sizeof(ImDrawVert), (GLvoid*)IM_OFFSETOF(ImDrawVert, pos));
    glVertexAttribPointer(attrib_location_uv_, 2, GL_FLOAT, GL_FALSE, sizeof(ImDrawVert), (GLvoid*)IM_OFFSETOF(ImDrawVert, uv));
    glVertexAttribPointer(attrib_location_color_, 4, GL_UNSIGNED_BYTE, GL_TRUE, sizeof(ImDrawVert), (GLvoid*)IM_OFFSETOF(ImDrawVert, col));

    /////////////////////////////////////////////
    // Test draw
    #if 0
    {
      ImVector<ImDrawVert> VtxBuffer;
      const float sc = 50.0;
      const float off_y = 50.0;
      const int num = 16;
      for (int i = 0; i < num; ++i) {
        float uv_x = float(i) / float(num);
        {
          ImDrawVert p1;
          p1.pos.x = i * sc;
          p1.pos.y = off_y;
          p1.uv.x = uv_x;
          p1.uv.y = 0;
          p1.col = IM_COL32(255, 0, 255, 255);
          VtxBuffer.push_back(p1);
        }
        {
          ImDrawVert p2;
          p2.pos.x = i * sc;
          p2.pos.y = off_y + sc;
          p2.uv.x = uv_x;
          p2.uv.y = 0.5;
          p2.col = IM_COL32(255, 255, 0, 255);
          VtxBuffer.push_back(p2);
        }
      }

      ImVector<ImDrawIdx> IdxBuffer;
      for (int i = 0; i < VtxBuffer.Size - 3; i += 2) {
#if 0
        IdxBuffer.push_back(i);
        IdxBuffer.push_back(i + 1);
        IdxBuffer.push_back(i + 3);
#endif
        IdxBuffer.push_back(i);
        IdxBuffer.push_back(i + 3);
        IdxBuffer.push_back(i + 2);
      }

      ImVector<ImDrawCmd> CmdBuffer;
      ImDrawCmd cmd;
      cmd.ElemCount = IdxBuffer.Size;
      CmdBuffer.push_back(cmd);

      checkGLError(__FILE__, __LINE__);
      const ImDrawIdx* idx_buffer_offset = 0;

      glBindBuffer(GL_ARRAY_BUFFER, vbo_handle_);
      glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)VtxBuffer.Size * sizeof(ImDrawVert),
          (const GLvoid*)VtxBuffer.Data, GL_STREAM_DRAW);

      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elements_handle_);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)IdxBuffer.Size * sizeof(ImDrawIdx),
          (const GLvoid*)IdxBuffer.Data, GL_STREAM_DRAW);
      checkGLError(__FILE__, __LINE__);

      ImVec4 clip_rect = ImVec4(0, 0, fb_width, fb_height);
      glScissor((int)clip_rect.x, (int)(fb_height - clip_rect.w),
          (int)(clip_rect.z - clip_rect.x), (int)(clip_rect.w - clip_rect.y));

      ImTextureID tex_id = nullptr;
      #if 1
      // grab any texture at all to test with
      for (int i = 0; i < draw_data->CmdListsCount; ++i) {
        for (int j = 0; j < draw_data->CmdLists[i]->CmdBuffer.Size; ++j) {
          tex_id = draw_data->CmdLists[i]->CmdBuffer[j].TextureId;
          break;
        }
      }
      static bool has_printed = false;
      if (!has_printed) {
        std::cout << "impl test texture id " << tex_id << "\n";
        has_printed = true;
      }
      #endif

      for (int cmd_i = 0; cmd_i < CmdBuffer.Size; cmd_i++)
      {
        const ImDrawCmd* pcmd = &CmdBuffer[cmd_i];
        {
          // Bind texture- if it is null then the color is black
          if (tex_id != nullptr)
            glBindTexture(GL_TEXTURE_2D, (GLuint)(intptr_t)tex_id);
          glDrawElements(GL_TRIANGLES, (GLsizei)pcmd->ElemCount,
              sizeof(ImDrawIdx) == 2 ? GL_UNSIGNED_SHORT : GL_UNSIGNED_INT, idx_buffer_offset);
          // std::cout << cmd_i << " " << tex_id << " " << idx_buffer_offset << "\n";
          checkGLError(__FILE__, __LINE__);
        }
        idx_buffer_offset += pcmd->ElemCount;
      }
    }  // test draw
    #endif
    /////////////////////////////////////////////

    // Draw
    ImVec2 pos = draw_data->DisplayPos;
    for (int n = 0; n < draw_data->CmdListsCount; n++)
    {
        const ImDrawList* cmd_list = draw_data->CmdLists[n];
        const ImDrawIdx* idx_buffer_offset = 0;

        glBindBuffer(GL_ARRAY_BUFFER, vbo_handle_);
        glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)cmd_list->VtxBuffer.Size * sizeof(ImDrawVert), (const GLvoid*)cmd_list->VtxBuffer.Data, GL_STREAM_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elements_handle_);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)cmd_list->IdxBuffer.Size * sizeof(ImDrawIdx), (const GLvoid*)cmd_list->IdxBuffer.Data, GL_STREAM_DRAW);

#if 0
        std::cout << n << " "
            << cmd_list->VtxBuffer.Size << " "
            << cmd_list->IdxBuffer.Size << ", vertices:\n";
#endif
#if 0
        if (n < 2) {
          std::cout << n << " " << cmd_list->VtxBuffer.Size << " "
              << cmd_list->IdxBuffer.Size << " ";
          for (int i = 0; i < cmd_list->VtxBuffer.Size && i < 4; ++i) {
            std::cout << i << ", "
                << "elem " << cmd_list->CmdBuffer[i].ElemCount << " "
                << "xy " << cmd_list->VtxBuffer[i].pos.x << " "
                << cmd_list->VtxBuffer[i].pos.y << ", "
                << "uv " << cmd_list->VtxBuffer[i].uv.x << " "
                << cmd_list->VtxBuffer[i].uv.y << ", ";
          }
          std::cout << "\n";
        }
#endif
        for (int cmd_i = 0; cmd_i < cmd_list->CmdBuffer.Size; cmd_i++)
        {
            const ImDrawCmd* pcmd = &cmd_list->CmdBuffer[cmd_i];
            if (pcmd->UserCallback)
            {
                // User callback (registered via ImDrawList::AddCallback)
                pcmd->UserCallback(cmd_list, pcmd);
            }
            else
            {
                ImVec4 clip_rect = ImVec4(pcmd->ClipRect.x - pos.x, pcmd->ClipRect.y - pos.y, pcmd->ClipRect.z - pos.x, pcmd->ClipRect.w - pos.y);
                if (clip_rect.x < fb_width && clip_rect.y < fb_height && clip_rect.z >= 0.0f && clip_rect.w >= 0.0f)
                {
                    // Apply scissor/clipping rectangle
                    glScissor((int)clip_rect.x, (int)(fb_height - clip_rect.w), (int)(clip_rect.z - clip_rect.x), (int)(clip_rect.w - clip_rect.y));

                    // std::cout << clip_rect.x << " " << clip_rect.y << " "
                    //     << idx_buffer_offset << "\n";

                    // Bind texture, Draw
                    glBindTexture(GL_TEXTURE_2D, (GLuint)(intptr_t)pcmd->TextureId);
                    glDrawElements(GL_TRIANGLES, (GLsizei)pcmd->ElemCount, sizeof(ImDrawIdx) == 2 ? GL_UNSIGNED_SHORT : GL_UNSIGNED_INT, idx_buffer_offset);
                }
            }
            idx_buffer_offset += pcmd->ElemCount;
        }
    }
    glDeleteVertexArrays(1, &vao_handle);

    // Restore modified GL state
    gl_state.restore();
}

bool ImGuiImplOpenGL3::CreateFontsTexture()
{
    // Build texture atlas
    ImGuiIO& io = ImGui::GetIO();
    unsigned char* pixels;
    int width, height;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);   // Load as RGBA 32-bits (75% of the memory is wasted, but default font is so small) because it is more likely to be compatible with user's existing shaders. If your ImTextureId represent a higher-level concept than just a GL texture id, consider calling GetTexDataAsAlpha8() instead to save on GPU memory.

    // Upload texture to graphics system
    GLint last_texture;
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture);
    glGenTextures(1, &FontTexture);
    glBindTexture(GL_TEXTURE_2D, FontTexture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);

    // Store our identifier
    io.Fonts->TexID = (ImTextureID)(intptr_t)FontTexture;

    // Restore state
    glBindTexture(GL_TEXTURE_2D, last_texture);

    return true;
}

void ImGuiImplOpenGL3::DestroyFontsTexture()
{
    if (FontTexture)
    {
        ImGuiIO& io = ImGui::GetIO();
        glDeleteTextures(1, &FontTexture);
        io.Fonts->TexID = 0;
        FontTexture = 0;
    }
}

// If you get an error please report on github. You may try different GL context version or GLSL version.
bool CheckShader(GLuint handle, const char* desc)
{
    GLint status = 0, lolength = 0;
    glGetShaderiv(handle, GL_COMPILE_STATUS, &status);
    glGetShaderiv(handle, GL_INFO_LOG_LENGTH, &lolength);
    if ((GLboolean)status == GL_FALSE)
        fprintf(stderr, "ERROR: ImGuiImplOpenGL3::CreateDeviceObjects: failed to compile %d %s!\n", handle, desc);
    if (lolength > 0)
    {
        ImVector<char> buf;
        buf.resize((int)(lolength + 1));
        glGetShaderInfoLog(handle, lolength, NULL, (GLchar*)buf.begin());
        fprintf(stderr, "%s\n", buf.begin());
    }
    return (GLboolean)status == GL_TRUE;
}

// If you get an error please report on github. You may try different GL context version or GLSL version.
bool CheckProgram(GLuint handle, const char* desc)
{
    GLint status = 0, lolength = 0;
    glGetProgramiv(handle, GL_LINK_STATUS, &status);
    glGetProgramiv(handle, GL_INFO_LOG_LENGTH, &lolength);
    if ((GLboolean)status == GL_FALSE)
        fprintf(stderr, "ERROR: ImGuiImplOpenGL3::CreateDeviceObjects: failed to link %s!\n", desc);
    if (lolength > 0)
    {
        ImVector<char> buf;
        buf.resize((int)(lolength + 1));
        glGetProgramInfoLog(handle, lolength, NULL, (GLchar*)buf.begin());
        fprintf(stderr, "%s\n", buf.begin());
    }
    return (GLboolean)status == GL_TRUE;
}

bool    ImGuiImplOpenGL3::CreateDeviceObjects()
{
    // Backup GL state
    GLint last_texture, last_array_buffer, last_vertex_array;
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture);
    glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &last_array_buffer);
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &last_vertex_array);

    // Parse GLSL version string
    int glsl_version = 130;
    sscanf(GlslVersionString, "#version %d", &glsl_version);

    const GLchar* vertex_shader_glsl_120 =
        "uniform mat4 ProjMtx;\n"
        "attribute vec2 Position;\n"
        "attribute vec2 UV;\n"
        "attribute vec4 Color;\n"
        "varying vec2 FraUV;\n"
        "varying vec4 FraColor;\n"
        "void main()\n"
        "{\n"
        "    FraUV = UV;\n"
        "    FraColor = Color;\n"
        "    gl_Position = ProjMtx * vec4(Position.xy,0,1);\n"
        "}\n";

    const GLchar* vertex_shader_glsl_130 =
        "uniform mat4 ProjMtx;\n"
        "in vec2 Position;\n"
        "in vec2 UV;\n"
        "in vec4 Color;\n"
        "out vec2 FraUV;\n"
        "out vec4 FraColor;\n"
        "void main()\n"
        "{\n"
        "    FraUV = UV;\n"
        "    FraColor = Color;\n"
        "    gl_Position = ProjMtx * vec4(Position.xy,0,1);\n"
        "}\n";

    const GLchar* vertex_shader_glsl_300_es =
        "precision mediump float;\n"
        "layout (location = 0) in vec2 Position;\n"
        "layout (location = 1) in vec2 UV;\n"
        "layout (location = 2) in vec4 Color;\n"
        "uniform mat4 ProjMtx;\n"
        "out vec2 FraUV;\n"
        "out vec4 FraColor;\n"
        "void main()\n"
        "{\n"
        "    FraUV = UV;\n"
        "    FraColor = Color;\n"
        "    gl_Position = ProjMtx * vec4(Position.xy,0,1);\n"
        "}\n";

    const GLchar* vertex_shader_glsl_410_core =
        "layout (location = 0) in vec2 Position;\n"
        "layout (location = 1) in vec2 UV;\n"
        "layout (location = 2) in vec4 Color;\n"
        "uniform mat4 ProjMtx;\n"
        "out vec2 FraUV;\n"
        "out vec4 FraColor;\n"
        "void main()\n"
        "{\n"
        "    FraUV = UV;\n"
        "    FraColor = Color;\n"
        "    gl_Position = ProjMtx * vec4(Position.xy,0,1);\n"
        "}\n";

    const GLchar* fragment_shader_glsl_120 =
        "#ifdef GL_ES\n"
        "    precision mediump float;\n"
        "#endif\n"
        "uniform sampler2D Texture;\n"
        "varying vec2 FraUV;\n"
        "varying vec4 FraColor;\n"
        "void main()\n"
        "{\n"
        "    gl_FragColor = FraColor * texture2D(Texture, FraUV.st);\n"
        "}\n";

    const GLchar* fragment_shader_glsl_130 =
        "uniform sampler2D Texture;\n"
        "in vec2 FraUV;\n"
        "in vec4 FraColor;\n"
        "out vec4 Out_Color;\n"
        "void main()\n"
        "{\n"
        "    Out_Color = FraColor * texture(Texture, FraUV.st);\n"
        "}\n";

    const GLchar* fragment_shader_glsl_300_es =
        "precision mediump float;\n"
        "uniform sampler2D Texture;\n"
        "in vec2 FraUV;\n"
        "in vec4 FraColor;\n"
        "layout (location = 0) out vec4 Out_Color;\n"
        "void main()\n"
        "{\n"
        "    Out_Color = FraColor * texture(Texture, FraUV.st);\n"
        "}\n";

    const GLchar* fragment_shader_glsl_410_core =
        "in vec2 FraUV;\n"
        "in vec4 FraColor;\n"
        "uniform sampler2D Texture;\n"
        "layout (location = 0) out vec4 Out_Color;\n"
        "void main()\n"
        "{\n"
        "    Out_Color = FraColor * texture(Texture, FraUV.st);\n"
        "}\n";

    // Select shaders matching our GLSL versions
    const GLchar* vertex_shader = NULL;
    const GLchar* fragment_shader = NULL;
    std::cout << "GLSL version: " << glsl_version << "\n";
    if (glsl_version < 130)
    {
        vertex_shader = vertex_shader_glsl_120;
        fragment_shader = fragment_shader_glsl_120;
    }
    else if (glsl_version == 410)
    {
        vertex_shader = vertex_shader_glsl_410_core;
        fragment_shader = fragment_shader_glsl_410_core;
    }
    else if (glsl_version == 300)
    {
        vertex_shader = vertex_shader_glsl_300_es;
        fragment_shader = fragment_shader_glsl_300_es;
    }
    else
    {
        vertex_shader = vertex_shader_glsl_130;
        fragment_shader = fragment_shader_glsl_130;
    }
    // std::cout << "glsl vertex shader " << glsl_version << ":\n" << vertex_shader << "\n";
    // std::cout << "glsl fragment shader:\n" << fragment_shader << "\n";

    // Create shaders
    const GLchar* vertex_shader_with_version[2] = { GlslVersionString, vertex_shader };
    vert_handle_ = glCreateShader(GL_VERTEX_SHADER);
    if (!vert_handle_)
    {
      std::cerr << "vertex shader failed to create " << glGetError() << "\n";
    }
    else
    {
      glShaderSource(vert_handle_, 2, vertex_shader_with_version, NULL);
      glCompileShader(vert_handle_);
      CheckShader(vert_handle_, "vertex shader");
    }

    const GLchar* fragment_shader_with_version[2] = { GlslVersionString, fragment_shader };
    frag_handle_ = glCreateShader(GL_FRAGMENT_SHADER);
    if (!frag_handle_)
    {
      std::cerr << "fragment shader failed to create " << glGetError() << "\n";
    }
    else
    {
      glShaderSource(frag_handle_, 2, fragment_shader_with_version, NULL);
      glCompileShader(frag_handle_);
      CheckShader(frag_handle_, "fragment shader");
    }

    if (vert_handle_ && frag_handle_)
    {
      shader_handle_ = glCreateProgram();
      glAttachShader(shader_handle_, vert_handle_);
      glAttachShader(shader_handle_, frag_handle_);
      glLinkProgram(shader_handle_);
      CheckProgram(shader_handle_, "shader program");

      attrib_location_tex_ = glGetUniformLocation(shader_handle_, "Texture");
      attrib_location_proj_mtx_ = glGetUniformLocation(shader_handle_, "ProjMtx");
      attrib_location_position_ = glGetAttribLocation(shader_handle_, "Position");
      attrib_location_uv_ = glGetAttribLocation(shader_handle_, "UV");
      attrib_location_color_ = glGetAttribLocation(shader_handle_, "Color");

      // Create buffers
      glGenBuffers(1, &vbo_handle_);
      glGenBuffers(1, &elements_handle_);

      ImGuiImplOpenGL3::CreateFontsTexture();
    }

    // Restore modified GL state
    glBindTexture(GL_TEXTURE_2D, last_texture);
    glBindBuffer(GL_ARRAY_BUFFER, last_array_buffer);
    glBindVertexArray(last_vertex_array);

    return true;
}

void    ImGuiImplOpenGL3::DestroyDeviceObjects()
{
    if (vbo_handle_) glDeleteBuffers(1, &vbo_handle_);
    if (elements_handle_) glDeleteBuffers(1, &elements_handle_);
    vbo_handle_ = elements_handle_ = 0;

    if (shader_handle_ && vert_handle_) glDetachShader(shader_handle_, vert_handle_);
    if (vert_handle_) glDeleteShader(vert_handle_);
    vert_handle_ = 0;

    if (shader_handle_ && frag_handle_) glDetachShader(shader_handle_, frag_handle_);
    if (frag_handle_) glDeleteShader(frag_handle_);
    frag_handle_ = 0;

    if (shader_handle_) glDeleteProgram(shader_handle_);
    shader_handle_ = 0;

    ImGuiImplOpenGL3::DestroyFontsTexture();
}

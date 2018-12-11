/*
 * Copyright (c) 2018 Lucas Walter
 * October 2018
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <SDL2/SDL.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <imgui.h>
// #include "imgui_impl_sdl.h"
#include <imgui_ros/viz3d.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
using std::placeholders::_1;
using std::placeholders::_2;

void makeTestShape(std::shared_ptr<Shape> shape)
{
  if (!shape) {
    std::cerr << "shape needs to be already created\n";
    return;
  }

  const float sc = 0.2;
  const float off_y = 0.0;
  const int num = 16;
  const float off_x = -sc * num / 2;

  for (int j = 0; j < 5; ++j) {
    for (int i = shape->vertices_.Size; i < shape->vertices_.Size + num * 2 - 3; i += 2) {
      shape->indices_.push_back(i);
      shape->indices_.push_back(i + 3);
      shape->indices_.push_back(i + 2);

      shape->indices_.push_back(i);
      shape->indices_.push_back(i + 1);
      shape->indices_.push_back(i + 3);
    }

    for (int i = 0; i < num; ++i) {
      float fr = float(i) / float(num);
      {
        DrawVert p1;
        p1.pos.x = i * sc + off_x;
        p1.pos.y = off_y;
        p1.pos.z = 1.0 + j * sc * 2;
        p1.uv.x = fr;
        p1.uv.y = 0;
        // These colors multiply with the texture color
        p1.col = glm::vec4(1.0, 1.0, 1.0, 1.0);
        shape->vertices_.push_back(p1);
      }
      {
        DrawVert p2;
        p2.pos.x = i * sc + off_x;
        p2.pos.y = off_y + sc * 4;
        p2.pos.z = 1.0 + j * sc * 2;
        p2.uv.x = fr;
        p2.uv.y = 1.0;
        p2.col = glm::vec4(1.0 - fr, 1.0 - fr, 1.0, 1.0);
        shape->vertices_.push_back(p2);
      }
    }
  }
}

void Shape::init(GLuint& shader_handle)
{
  glGenVertexArrays(1, &vao_handle_);
  glBindVertexArray(vao_handle_);

  glGenBuffers(1, &vbo_handle_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_handle_);
  // copy data to gpu
  glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)vertices_.Size * sizeof(DrawVert),
      (const GLvoid*)vertices_.Data, GL_STREAM_DRAW);

  attrib_location_tex_ = glGetUniformLocation(shader_handle, "Texture");
  attrib_location_proj_mtx_ = glGetUniformLocation(shader_handle, "ProjMtx");
  attrib_location_position_ = glGetAttribLocation(shader_handle, "Position");
  attrib_location_uv_ = glGetAttribLocation(shader_handle, "UV");
  attrib_location_color_ = glGetAttribLocation(shader_handle, "Color");

  checkGLError(__FILE__, __LINE__);
  glEnableVertexAttribArray(attrib_location_position_);
  glEnableVertexAttribArray(attrib_location_uv_);
  glEnableVertexAttribArray(attrib_location_color_);
  checkGLError(__FILE__, __LINE__);
  glVertexAttribPointer(attrib_location_position_, 3, GL_FLOAT, GL_FALSE,
      sizeof(DrawVert), (GLvoid*)offsetof(DrawVert, pos));
  glVertexAttribPointer(attrib_location_uv_, 2, GL_FLOAT, GL_FALSE,
      sizeof(DrawVert), (GLvoid*)offsetof(DrawVert, uv));
  glVertexAttribPointer(attrib_location_color_, 4, GL_FLOAT, GL_FALSE,
      sizeof(DrawVert), (GLvoid*)offsetof(DrawVert, col));
  checkGLError(__FILE__, __LINE__);

  glGenBuffers(1, &elements_handle_);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elements_handle_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)indices_.Size * sizeof(ImDrawIdx),
      (const GLvoid*)indices_.Data, GL_STREAM_DRAW);
  checkGLError(__FILE__, __LINE__);
}

// render the entire background
// this probably will be split out into a widget also.
Viz3D::Viz3D(const std::string name,
    const std::string topic,
    std::shared_ptr<ImGuiImplOpenGL3> renderer,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<rclcpp::Node> node) :
    name_(name),
    renderer_(renderer),
    tf_buffer_(tf_buffer),
    node_(node)
{
  ros_image_.reset(new RosImage("texture", "/image_out", node));

  // TODO(lucasw) maintaining many versions of these seems like a big hassle,
  // which is why bgfx exists...
  // const GLchar* vertex_shader_glsl_130 =
  const GLchar* vertex_shader =
      "uniform mat4 ProjMtx;\n"
      "in vec3 Position;\n"
      "in vec2 UV;\n"
      "in vec4 Color;\n"
      "out vec2 FraUV;\n"
      "out vec4 FraColor;\n"
      "void main()\n"
      "{\n"
      "    FraUV = UV;\n"
      "    FraColor = Color;\n"
      "    gl_Position = ProjMtx * vec4(Position.xyz, 1.0);\n"
      "}\n";
  std::cout << "viz3d vertex shader:\n" << vertex_shader << "\n";

  // const GLchar* fragment_shader_glsl_130 =
  const GLchar* fragment_shader =
      "uniform sampler2D Texture;\n"
      "in vec2 FraUV;\n"
      "in vec4 FraColor;\n"
      "out vec4 Out_Color;\n"
      "void main()\n"
      "{\n"
      "    Out_Color = FraColor * texture(Texture, FraUV.st);\n"
      "}\n";

  // Create shaders
  const GLchar* vertex_shader_with_version[2] = { renderer->GlslVersionString, vertex_shader };
  vert_handle_ = glCreateShader(GL_VERTEX_SHADER);
  if (!vert_handle_)
  {
    std::cerr << "vertex shader failed to create " << glGetError() << "\n";
    return;
  }
  else
  {
    glShaderSource(vert_handle_, 2, vertex_shader_with_version, NULL);
    glCompileShader(vert_handle_);
    CheckShader(vert_handle_, "vertex shader");
  }

  const GLchar* fragment_shader_with_version[2] = { renderer->GlslVersionString, fragment_shader };
  frag_handle_ = glCreateShader(GL_FRAGMENT_SHADER);
  if (!frag_handle_)
  {
    std::cerr << "fragment shader failed to create " << glGetError() << "\n";
    return;
  }
  else
  {
    glShaderSource(frag_handle_, 2, fragment_shader_with_version, NULL);
    glCompileShader(frag_handle_);
    CheckShader(frag_handle_, "fragment shader");
  }

  shader_handle_ = glCreateProgram();
  glAttachShader(shader_handle_, vert_handle_);
  glAttachShader(shader_handle_, frag_handle_);
  glLinkProgram(shader_handle_);
  CheckProgram(shader_handle_, "shader program");

  textured_shape_sub_ = node->create_subscription<imgui_ros::msg::TexturedShape>(topic,
        std::bind(&Viz3D::texturedShapeCallback, this, _1));

  add_shape_ = node->create_service<imgui_ros::srv::AddShape>("add_shape",
      std::bind(&Viz3D::addShape, this, _1, _2));
  #if 0
  test_shape_ = std::make_shared<Shape>();
  makeTestShape(test_shape_);
  shapes_[test_shape_->name_] = test_shape_;
  #endif
}

Viz3D::~Viz3D()
{
#if 0
  glDeleteTextures(1, &texture_id_);
#endif
}

void Viz3D::addShape(const std::shared_ptr<imgui_ros::srv::AddShape::Request> req,
                std::shared_ptr<imgui_ros::srv::AddShape::Response> res)
{
  for (auto textured_shape : req->shapes) {
    auto shape = std::make_shared<imgui_ros::msg::TexturedShape>(textured_shape);
    texturedShapeCallback(shape);
    res->message += " " + shape->name;
  }

  res->success = true;
}

// TODO(lucasw) Shape -> Mesh?
void Viz3D::texturedShapeCallback(const imgui_ros::msg::TexturedShape::SharedPtr msg)
{
  if (msg->name == "") {
    std::cerr << "mesh needs name\n";
    return;
  }

  if (msg->add == false) {

    if (shapes_.count(msg->name) > 0) {
      shapes_.erase(msg->name);
      return;
    }
  }

  bool use_uv = msg->uv.size() > 0;
  if ((use_uv) && (msg->uv.size() != msg->mesh.vertices.size())) {
    std::cerr << "mismatching uv sizes " << msg->uv.size() << " "
        << msg->mesh.vertices.size() << "\n";
    use_uv = false;
  }
  bool use_color = msg->colors.size() > 0;
  if ((use_color) && (msg->colors.size() != msg->mesh.vertices.size())) {
    std::cerr << "mismatching color sizes " << msg->colors.size() << " "
        << msg->mesh.vertices.size() << "\n";
    use_uv = false;
  }
  glm::vec4 default_color = glm::vec4(1.0, 1.0, 1.0, 1.0);
  if ((!use_color) && (msg->colors.size() == 1)) {
    default_color.x = msg->colors[0].r;
    default_color.y = msg->colors[0].g;
    default_color.z = msg->colors[0].b;
    default_color.w = msg->colors[0].a;
  }

  auto shape = std::make_shared<Shape>();
  shape->name_ = msg->name;
  shape->frame_id_ = msg->header.frame_id;
  shape->tf_buffer_ = tf_buffer_;
  shape->texture_ = msg->texture;

  // TODO(lucasw) if is_topic then create RosImage subscriber
  // if msg->image isn't empty create a RosImage and initialize the image
  // with it (RosImage doesn't support that yet).

  for (size_t i = 0; i < msg->mesh.vertices.size(); ++i) {
    DrawVert p1;
    p1.pos.x = msg->mesh.vertices[i].x;
    p1.pos.y = msg->mesh.vertices[i].y;
    p1.pos.z = msg->mesh.vertices[i].z;
    if (use_uv) {
      p1.uv.x = msg->uv[i].x;
      p1.uv.y = msg->uv[i].y;
    }
    // These colors multiply with the texture color
    if (use_color) {
      p1.col.x = msg->colors[i].r;
      p1.col.y = msg->colors[i].g;
      p1.col.z = msg->colors[i].b;
      p1.col.w = msg->colors[i].a;
    } else {
      p1.col = default_color;
    }
    shape->vertices_.push_back(p1);
  }

  for (size_t i = 0; i < msg->mesh.triangles.size(); ++i) {
    for (size_t j = 0; j < msg->mesh.triangles[i].vertex_indices.size(); ++j) {
      const int ind = msg->mesh.triangles[i].vertex_indices[j];
      if ((ind < 0) || (ind >= shape->vertices_.Size)) {
        std::cerr << "bad triangle index " << ind << " >= "
            << shape->vertices_.Size << "\n";
        // TODO(lucasw) or set to zero, or Size - 1?
        return;
      }
      shape->indices_.push_back(ind);
    }
  }

  shape->init(shader_handle_);
  shape->print();
  shapes_[shape->name_] = shape;
}

void Viz3D::draw()
//  const int pos_x, const int pos_y,
//  const int size_x, const int size_y)
{
  ImGui::Begin("camera");
  // ImGuiIO& io = ImGui::GetIO();

  const float sc = 0.03;
  if (ImGui::IsKeyPressed(SDL_SCANCODE_A)) {
    translation_.x += sc;
  }
  if (ImGui::IsKeyPressed(SDL_SCANCODE_D)) {
    translation_.x -= sc;
  }
  if (ImGui::IsKeyPressed(SDL_SCANCODE_W)) {
    translation_.z += sc;
  }
  if (ImGui::IsKeyPressed(SDL_SCANCODE_S)) {
    translation_.z -= sc;
  }
  if (ImGui::IsKeyPressed(SDL_SCANCODE_Q)) {
    translation_.y += sc;
  }
  if (ImGui::IsKeyPressed(SDL_SCANCODE_Z)) {
    translation_.y -= sc;
  }

  // mouse input
  ImVec2 mouse_pos_in_canvas = ImGui::GetIO().MousePos;

  if (dragging_view_) {
    // This allows continued dragging outside the canvas
    ImVec2 offset = ImVec2(
        mouse_pos_in_canvas.x - drag_point_.x,
        mouse_pos_in_canvas.y - drag_point_.y);
    angle_ -= offset.x / 300.0;
    drag_point_ = mouse_pos_in_canvas;
    if (!ImGui::IsMouseDown(0)) {
      dragging_view_ = false;
    }
  }

  if (!ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow)) {
    if (!dragging_view_ && ImGui::IsMouseClicked(0)) {
      drag_point_ = mouse_pos_in_canvas;
      dragging_view_ = true;
    }
    #if 0
    if (!dragging_scale_ && ImGui::IsMouseClicked(1)) {
      drag_point_ = mouse_pos_in_canvas;
      dragging_scale_ = true;
    }
    #endif
  }

  // Maybe aovy should be a ros topic,
  // and the slider a regular Pub widget.
  double min = 1.0;
  double max = 170.0;
  ImGui::SliderScalar("aov y", ImGuiDataType_Double,
      &aov_y_, &min, &max, "%lf");

  min = 0.1;
  max = 10.0;
  ImGui::SliderScalar("aspect scale", ImGuiDataType_Double,
      &aspect_scale_, &min, &max, "%lf");

  std::stringstream ss;
  ss << translation_.x  << " " << translation_.y << " " << translation_.z << ", "
      << angle_;
  ImGui::Text("%s", ss.str().c_str());
  ImGui::End();
}

// Currently calling setupcamera for every object- that seems efficient vs.
// transforming all the data of every object.
void Viz3D::setupCamera(const std::string child_frame_id,
    const int fb_width, const int fb_height, glm::mat4& mvp)
{
  const float aspect = static_cast<float>(fb_width) / static_cast<float>(fb_height) * aspect_scale_;
  glm::mat4 projection_matrix = glm::perspective(static_cast<float>(glm::radians(aov_y_)),
      aspect, near_, far_);

  glm::mat4 model_matrix = glm::mat4(1.0f);
  try {
    geometry_msgs::msg::TransformStamped tf;
    tf = tf_buffer_->lookupTransform(frame_id_, child_frame_id, tf2::TimePointZero);
    tf2::Stamped<tf2::Transform> stamped_transform;
    tf2::fromMsg(tf, stamped_transform);
    #if 0
    tf2::TimePoint time_out;
    // this is private, so doesn't work
    tf_buffer_->lookupTransformImpl(frame_id_, child_frame_id,
        tf2::TimePointZero, transform, time_out);
    #endif

    glm::dmat4 model_matrix_double;
    stamped_transform.getOpenGLMatrix(&model_matrix_double[0][0]);
    // TODO(lucasw) is there a glm double to float conversion function?
    for (size_t i = 0; i < 4; ++i)
      for (size_t j = 0; j < 4; ++j)
        model_matrix[i][j] = model_matrix_double[i][j];
  } catch (tf2::TransformException& ex) {
    // TODO(lucasw) display exception on gui, but this isn't currently the correct
    // time.
    // ImGui::Text("%s", ex.what());
  }

  glm::mat4 view_matrix = glm::lookAt(
      translation_,
      translation_ + glm::vec3(sin(angle_), 0, cos(angle_)),
      glm::vec3(0,1,0)  // Head is up (set to 0,-1,0 to look upside-down)
  );
  mvp = projection_matrix * view_matrix * model_matrix;

  {
    static bool has_printed = false;
    if (!has_printed) {
      has_printed = true;
      std::cout << "projection\n";
      for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 4; ++j) {
          std::cout << mvp[i][j] << " ";
        }
        std::cout << "\n";
      }
    }
  }
}

void Viz3D::render(const int fb_width, const int fb_height,
  const int display_pos_x, const int display_pos_y,
  const int display_size_x, const int display_size_y)
{
  (void)display_pos_x;
  (void)display_pos_y;
  (void)display_size_x;
  (void)display_size_y;

  if (shapes_.size() == 0) {
    return;
  }

  if (fb_width <= 0 || fb_height <= 0) {
    std::cerr << "bad width height " << fb_width << " " << fb_height << "\n";
    return;
  }

  std::shared_ptr<ImGuiImplOpenGL3> renderer = renderer_.lock();
  if (!renderer) {
    std::cerr << "no renderer\n";
    return;
  }

    checkGLError(__FILE__, __LINE__);

    ros_image_->updateTexture();

    GLState gl_state;
    gl_state.backup();

    // Setup render state: alpha-blending enabled, no face culling, no depth testing, scissor enabled, polygon fill
    glEnable(GL_BLEND);
    glBlendEquation(GL_FUNC_ADD);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    // TODO(lucasw) later enable
    glDisable(GL_CULL_FACE);
    // TODO(lucasw) later enable
    glEnable(GL_DEPTH_TEST);
    // Want to draw to whole window
    glDisable(GL_SCISSOR_TEST);
#ifdef GL_POLYGON_MODE
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
#endif
    checkGLError(__FILE__, __LINE__);

    // Our visible imgui space lies from draw_data->DisplayPos (top left) to draw_data->DisplayPos+data_data->DisplaySize (bottom right). DisplayMin is typically (0,0) for single viewport apps.
    glViewport(0, 0, (GLsizei)fb_width, (GLsizei)fb_height);

    // (This is to easily allow multiple GL contexts. VAO are not shared among GL contexts, and we don't track creation/deletion of windows so we don't have an obvious key to use to cache them.)
    checkGLError(__FILE__, __LINE__);

  for (auto shape_pair : shapes_) {
    auto shape = shape_pair.second;

    glm::mat4 mvp;
    setupCamera(shape->frame_id_, fb_width, fb_height, mvp);
    glUseProgram(shader_handle_);
    // glUniformMatrix4dv(shape->attrib_location_proj_mtx_, 1, GL_FALSE, &mvp[0][0]);
    glUniformMatrix4fv(shape->attrib_location_proj_mtx_, 1, GL_FALSE, &mvp[0][0]);
    glUniform1i(shape->attrib_location_tex_, 0);
#ifdef GL_SAMPLER_BINDING
    glBindSampler(0, 0);
    // We use combined texture/sampler state. Applications using GL 3.3 may set that otherwise.
#endif

    checkGLError(__FILE__, __LINE__);

    glBindVertexArray(shape->vao_handle_);

    ImVec4 clip_rect = ImVec4(0, 0, fb_width, fb_height);
    glScissor((int)clip_rect.x, (int)(fb_height - clip_rect.w),
        (int)(clip_rect.z - clip_rect.x), (int)(clip_rect.w - clip_rect.y));

#if 0
    {
      static bool has_printed = false;
      if (!has_printed) {
        has_printed = true;
        shape->print();
      }
    }

    // Bind texture- if it is null then the color is black
    // if (texture_id_ != nullptr)
    GLuint tex_id = 0;
    if ((shape->texture_ != "") && (ros_images_.count(shape->texture_) > 0)) {
      tex_id = (GLuint)(intptr_t)ros_images_[shape->texture_]->texture_id_;
    }
#endif
    const ImDrawIdx* idx_buffer_offset = 0;
      // Bind texture- if it is null then the color is black
      // if (texture_id_ != nullptr)
      glBindBuffer(GL_ARRAY_BUFFER, shape->vbo_handle_);  // needed before bind texture?
      glBindTexture(GL_TEXTURE_2D, (GLuint)(intptr_t)ros_image_->texture_id_);
      glDrawElements(GL_TRIANGLES, (GLsizei)shape->indices_.Size,
          sizeof(ImDrawIdx) == 2 ? GL_UNSIGNED_SHORT : GL_UNSIGNED_INT, idx_buffer_offset);
      // std::cout << cmd_i << " " << tex_id << " " << idx_buffer_offset << "\n";
      checkGLError(__FILE__, __LINE__);
      // idx_buffer_offset += pcmd->ElemCount;
    }  // test draw

  checkGLError(__FILE__, __LINE__);
  gl_state.backup();
  checkGLError(__FILE__, __LINE__);
}

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

// TODO(lucasw) is there a glm double to float conversion function?
void dmat4Todmat(const glm::dmat4& dmat, glm::mat4& mat)
{
  for (size_t i = 0; i < 4; ++i)
    for (size_t j = 0; j < 4; ++j)
      mat[i][j] = dmat[i][j];
}

void printMat(glm::mat4& mat, const std::string& desc = "")
{
  std::cout << desc << "\n";
  for (size_t i = 0; i < 4; ++i) {
    for (size_t j = 0; j < 4; ++j) {
      std::cout << mat[i][j] << " ";
    }
    std::cout << "\n";
  }
}
void printMat(glm::dmat4& mat, const std::string& desc = "")
{
  glm::mat4 mat2;
  dmat4Todmat(mat, mat2);
  printMat(mat2, desc);
}

void printTransform(tf2::Transform& tf, const std::string& desc = "")
{
  std::cout << desc << "\n";
  std::cout
    << tf.getOrigin().x() << " "
    << tf.getOrigin().y() << " "
    << tf.getOrigin().z() << "\n";
}

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

ShaderSet::~ShaderSet()
{
  glDeleteShader(vert_handle_);
  glDeleteShader(frag_handle_);
  glDeleteProgram(shader_handle_);
}

bool ShaderSet::init(const std::string& glsl_version, std::string& message)
{
  // Create shaders
  const GLchar* vertex_shader_with_version[2] = {glsl_version.c_str(),
      vertex_code_.c_str()};
  vert_handle_ = glCreateShader(GL_VERTEX_SHADER);
  if (!vert_handle_)
  {
    message = "vertex shader failed to create " + glGetError();
    return false;
  }
  else
  {
    glShaderSource(vert_handle_, 2, vertex_shader_with_version, NULL);
    glCompileShader(vert_handle_);
    if (!CheckShader(vert_handle_, "vertex shader", message)) {
      std::cout << vertex_code_ << std::endl;
      return false;
    }
  }

  const GLchar* fragment_shader_with_version[2] = {glsl_version.c_str(), fragment_code_.c_str() };
  frag_handle_ = glCreateShader(GL_FRAGMENT_SHADER);
  if (!frag_handle_)
  {
    message = "fragment shader failed to create " + glGetError();
    return false;
  }
  else
  {
    glShaderSource(frag_handle_, 2, fragment_shader_with_version, NULL);
    glCompileShader(frag_handle_);
    if (!CheckShader(frag_handle_, "fragment shader", message)) {
      std::cout << fragment_code_ << std::endl;
      return false;
    }
  }

  shader_handle_ = glCreateProgram();
  glAttachShader(shader_handle_, vert_handle_);
  glAttachShader(shader_handle_, frag_handle_);
  glLinkProgram(shader_handle_);
  if (!CheckProgram(shader_handle_, "shader program", message)) {
    return false;
  }

  // TODO(lucasw) make these generic - provide a list of uniforms
  // in request
  attrib_location_tex_ = glGetUniformLocation(shader_handle_, "Texture");
  attrib_location_proj_mtx_ = glGetUniformLocation(shader_handle_, "ProjMtx");
  attrib_location_position_ = glGetAttribLocation(shader_handle_, "Position");
  attrib_location_uv_ = glGetAttribLocation(shader_handle_, "UV");
  attrib_location_color_ = glGetAttribLocation(shader_handle_, "Color");

  attrib_location_projected_texture_scale_ = glGetUniformLocation(shader_handle_, "projected_texture_scale");
  attrib_location_proj_tex_ = glGetUniformLocation(shader_handle_, "ProjectedTexture");
  attrib_location_proj_tex_mtx_ = glGetUniformLocation(shader_handle_, "ProjTexMtx");

  glEnableVertexAttribArray(attrib_location_position_);
  glEnableVertexAttribArray(attrib_location_uv_);
  glEnableVertexAttribArray(attrib_location_color_);
  glVertexAttribPointer(attrib_location_position_, 3, GL_FLOAT, GL_FALSE,
      sizeof(DrawVert), (GLvoid*)offsetof(DrawVert, pos));
  glVertexAttribPointer(attrib_location_uv_, 2, GL_FLOAT, GL_FALSE,
      sizeof(DrawVert), (GLvoid*)offsetof(DrawVert, uv));
  glVertexAttribPointer(attrib_location_color_, 4, GL_FLOAT, GL_FALSE,
      sizeof(DrawVert), (GLvoid*)offsetof(DrawVert, col));

  std::string msg;
  if (checkGLError2(msg)) {
    return false;
  }

  return true;
}

void Shape::init()
{
  glGenVertexArrays(1, &vao_handle_);
  glBindVertexArray(vao_handle_);

  glGenBuffers(1, &vbo_handle_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_handle_);
  // copy data to gpu
  glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)vertices_.Size * sizeof(DrawVert),
      (const GLvoid*)vertices_.Data, GL_STREAM_DRAW);

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

  if (true) {
    glGenFramebuffers(1, &frame_buffer_);
    glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer_);

    cv::Mat tmp(cv::Size(render_width_, render_height_), CV_8UC4, cv::Scalar(100, 50, 20, 255));

    glGenTextures(1, &rendered_texture_);
    glBindTexture(GL_TEXTURE_2D, rendered_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, render_width_, render_height_, 0, GL_RGBA,
        GL_UNSIGNED_BYTE, &tmp.data[0]);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    glGenRenderbuffers(1, &depth_buffer_);
    glBindRenderbuffer(GL_RENDERBUFFER, depth_buffer_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT,
        render_width_, render_height_);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
        GL_RENDERBUFFER, depth_buffer_);

    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, rendered_texture_, 0);

    GLenum DrawBuffers[1] = {GL_COLOR_ATTACHMENT0};
    glDrawBuffers(1, DrawBuffers);
    // OpenGL 4?
    // glNamedFramebufferDrawBuffers(frame_buffer_, 1, DrawBuffers);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
      RCLCPP_ERROR(node->get_logger(), "Framebuffer is not complete %s", glGetError());
    } else {
      RCLCPP_INFO(node->get_logger(), "Framebuffer setup complete %d %d %d",
          frame_buffer_, depth_buffer_, rendered_texture_);
    }

    // restore default frame buffer
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    std::string msg;
    if (checkGLError2(msg)) {
      throw std::runtime_error(msg);
    }
  }

  transform_.setIdentity();

  textured_shape_sub_ = node->create_subscription<imgui_ros::msg::TexturedShape>(topic,
        std::bind(&Viz3D::texturedShapeCallback, this, _1));

  add_shaders_ = node->create_service<imgui_ros::srv::AddShaders>("add_shaders",
      std::bind(&Viz3D::addShaders, this, _1, _2));
  add_texture_ = node->create_service<imgui_ros::srv::AddTexture>("add_texture",
      std::bind(&Viz3D::addTexture, this, _1, _2));
  add_shape_ = node->create_service<imgui_ros::srv::AddShape>("add_shape",
      std::bind(&Viz3D::addShape, this, _1, _2));
  #if 0
  test_shape_ = std::make_shared<Shape>();
  makeTestShape(test_shape_);
  shapes_[test_shape_->name_] = test_shape_;
  #endif

  initialized_ = true;
}

Viz3D::~Viz3D()
{
#if 0
  glDeleteTextures(1, &texture_id_);
#endif
}

void Viz3D::addShaders(const std::shared_ptr<imgui_ros::srv::AddShaders::Request> req,
                       std::shared_ptr<imgui_ros::srv::AddShaders::Response> res)
{
  res->success = true;
  if (req->remove) {
    if (shader_sets_.count(req->name) > 0) {
      shader_sets_.erase(req->name);
      return;
    }
    return;
  }

  auto shaders = std::make_shared<ShaderSet>(req->name, req->vertex,
      req->geometry, req->fragment);

  std::shared_ptr<ImGuiImplOpenGL3> renderer = renderer_.lock();
  if (!renderer) {
    // TODO(lucasw) maybe should store them and try to initialize them
    // later?
    res->message = "no renderer, not ready for shaders";
    res->success = false;
    return;
  }

  if (!shaders->init(renderer->GlslVersionString, res->message)) {
    res->success = false;
    return;
  }
  shader_sets_[req->name] = shaders;
}

void Viz3D::addTexture(const std::shared_ptr<imgui_ros::srv::AddTexture::Request> req,
                       std::shared_ptr<imgui_ros::srv::AddTexture::Response> res)
{
  res->success = true;
  if (req->remove) {
    if (textures_.count(req->name) > 0) {
      textures_.erase(req->name);
      return;
    }
    return;
  }

  auto texture = std::make_shared<RosImage>(req->name);
  texture->imageCallback(std::make_shared<sensor_msgs::msg::Image>(req->image));
  texture->updateTexture();
  textures_[req->name] = texture;
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
    return;
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

  shape->init();
  shape->print();
  shapes_[shape->name_] = shape;
}

void Viz3D::draw()
//  const int pos_x, const int pos_y,
//  const int size_x, const int size_y)
{
  ImGui::Begin("camera");
  // ImGuiIO& io = ImGui::GetIO();

  if (true) {
    // TODO(lucasw) later re-use code in RosImage
    ImGui::Checkbox("render to texture", &enable_rtt_);
    if (enable_rtt_) {
      ImVec2 win_size = ImGui::GetWindowSize();
      ImGui::Image((void*)(intptr_t)rendered_texture_, win_size);
      // GLuint tex_id = (GLuint)(intptr_t)ros_image_->texture_id_;
      // ImGui::Image((void*)(intptr_t)tex_id, win_size);
    }
  }

  {
    // TODO(lucasw) make this a ros topic, use a regular PUB + SUB gui widget
    // const bool changed =
    ImGui::Checkbox("projected texture", &enable_projected_texture_);
    // TODO(lucasw) switch to ortho projection if angle is zero
    double min = 1.0;
    double max = 170.0;
    ImGui::SliderScalar("projected_texture_aov y", ImGuiDataType_Double,
        &projected_texture_aov_y_, &min, &max, "%lf");
  }
  {
    double min = 0.0001;
    double max = 0.1;
    ImGui::SliderScalar("move scale", ImGuiDataType_Double,
          &move_scale_, &min, &max, "%lf");
  }

  double x_move = 0.0;
  double y_move = 0.0;
  double z_move = 0.0;
  const double sc = move_scale_;
  if (ImGui::IsKeyPressed(SDL_SCANCODE_A)) {
    y_move += sc;
  }
  if (ImGui::IsKeyPressed(SDL_SCANCODE_D)) {
    y_move -= sc;
  }
  if (ImGui::IsKeyPressed(SDL_SCANCODE_W)) {
    x_move += sc;
  }
  if (ImGui::IsKeyPressed(SDL_SCANCODE_S)) {
    x_move -= sc;
  }
  if (ImGui::IsKeyPressed(SDL_SCANCODE_Q)) {
    z_move += sc;
  }
  if (ImGui::IsKeyPressed(SDL_SCANCODE_Z)) {
    z_move -= sc;
  }

  velocity_= velocity_ + tf2::Vector3(-y_move, z_move, -x_move);
  auto rot_mat = transform_.getBasis();
  tf2::Vector3 vel_in_world = rot_mat * velocity_;

  std::stringstream ss;
  ss << "velocity in view: " << velocity_.x()  << " " << velocity_.y() << " " << velocity_.z();
  ImGui::Text("%s", ss.str().c_str());
  ss.str("");
  ss << "velocity in world: " << vel_in_world.x()  << " " << vel_in_world.y() << " " << vel_in_world.z();
  ImGui::Text("%s", ss.str().c_str());

  tf2::Vector3 translation = transform_.getOrigin();
  translation = translation + vel_in_world;
  transform_.setOrigin(translation);

  velocity_ *= 0.8;

  // mouse input
  ImVec2 mouse_pos_in_canvas = ImGui::GetIO().MousePos;

  {
    double min = 5.0;
    double max = 500.0;
    ImGui::SliderScalar("rotate scale", ImGuiDataType_Double,
          &rotate_scale_, &min, &max, "%lf");
  }

  if (dragging_view_) {
    // This allows continued dragging outside the canvas
    ImVec2 offset = ImVec2(
        mouse_pos_in_canvas.x - drag_point_.x,
        mouse_pos_in_canvas.y - drag_point_.y);

    yaw_ -= offset.x / rotate_scale_;
    pitch_ -= offset.y / rotate_scale_;
    if (pitch_ > M_PI/2.0)
      pitch_ = M_PI/2.0;
    if (pitch_ < -M_PI/2.0)
      pitch_ = -M_PI/2.0;

    drag_point_ = mouse_pos_in_canvas;
    if (!ImGui::IsMouseDown(0)) {
      dragging_view_ = false;
    }
  }
  tf2::Quaternion rot;
  rot.setRPY(pitch_, yaw_, 0.0);
  transform_.setRotation(rot);

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

  {
    std::stringstream ss;
    ss << translation.x()  << " " << translation.y() << " " << translation.z() << ", "
        << pitch_;
    ImGui::Text("%s", ss.str().c_str());
  }
  ImGui::End();
}

// Currently calling setupcamera for every object- that seems efficient vs.
// transforming all the data of every object.
bool Viz3D::setupCamera(const tf2::Transform& view_transform,
    const std::string child_frame_id, const double aov_y,
    const int fb_width, const int fb_height, glm::mat4& mvp)
{
  const float aspect = static_cast<float>(fb_width) / static_cast<float>(fb_height) * aspect_scale_;
  glm::mat4 projection_matrix = glm::perspective(static_cast<float>(glm::radians(aov_y)),
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
    dmat4Todmat(model_matrix_double, model_matrix);
  } catch (tf2::TransformException& ex) {
    // TODO(lucasw) display exception on gui, but this isn't currently the correct
    // time.
    // ImGui::Text("%s", ex.what());
    return false;
  }

  glm::dmat4 view_matrix_double;
  view_transform.inverse().getOpenGLMatrix(&view_matrix_double[0][0]);
  // printMat(view_matrix_double, "view_matrix double");
  glm::mat4 view_matrix;
  dmat4Todmat(view_matrix_double, view_matrix);

  mvp = projection_matrix * view_matrix * model_matrix;

  {
    static bool has_printed = false;
    if (!has_printed) {
      has_printed = true;
      printMat(projection_matrix, "projection " + child_frame_id);
      printMat(view_matrix, "view_matrix " + child_frame_id);
      printMat(model_matrix, "model_matrix " + child_frame_id);
      printMat(mvp, "m * v * p");
    }
  }

  return true;
}

bool Viz3D::setupProjectedTexture(const std::string& shape_frame_id)
{
  const std::string name = projected_texture_name_;
  if (textures_.count(name) < 1) {
    return false;
  }
  if (!textures_[name]->image_) {
    return false;
  }
  const int width = textures_[name]->image_->width;
  const int height = textures_[name]->image_->height;

  // this is the view of the projector
  tf2::Stamped<tf2::Transform> stamped_transform;
  try {
    geometry_msgs::msg::TransformStamped tf;
    tf = tf_buffer_->lookupTransform(frame_id_,
        projected_texture_frame_id_, tf2::TimePointZero);
    tf2::fromMsg(tf, stamped_transform);
  } catch (tf2::TransformException& ex) {
    // TODO(lucasw) display exception on gui, but this isn't currently the correct
    // time.
    // ImGui::Text("%s", ex.what());
    return false;
  }

  {
    static bool has_printed = false;
    if (!has_printed) {
      has_printed = true;
      printTransform(stamped_transform, "projection transform");
    }
  }

  // texture projection
  glm::mat4 mtp;
  if (!setupCamera(stamped_transform, shape_frame_id,
      projected_texture_aov_y_,
      width, height,
      mtp))
    return false;

  for (auto shaders : shader_sets_) {
    glUniformMatrix4fv(shaders.second->attrib_location_proj_tex_mtx_, 1, GL_FALSE, &mtp[0][0]);
    // assign this texture to the TEXTURE1 slot
    glUniform1i(shaders.second->attrib_location_proj_tex_, 1);
    checkGLError(__FILE__, __LINE__);
  }
  return true;
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

#if 0
  std::shared_ptr<ImGuiImplOpenGL3> renderer = renderer_.lock();
  if (!renderer) {
    std::cerr << "no renderer\n";
    return;
  }
#endif

    checkGLError(__FILE__, __LINE__);

    ros_image_->updateTexture();

    GLState gl_state;
    gl_state.backup();

    // TODO(lucasw) later render to texture with a variable width and height
    // and then draw the texture to the screen with a textured quad fragment
    // shader, this allows rendering at a different resolution that the screen
    // which might be desirable if performance is suffering.
    render2(fb_width, fb_height);

    gl_state.backup();
    checkGLError(__FILE__, __LINE__);
}

// don't interleave this with regular 3d rendering imgui rendering
void Viz3D::renderToTexture()
{
  // TODO(lwalter) make a checkbox to enable this 
  if (enable_rtt_) {
    GLState gl_state;
    gl_state.backup();

    glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer_);
    glClearColor(0.5, 0.5, 0.1, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // TODO(lucasw) if render width/height change need to update rendered_texture
    render2(render_width_, render_height_);
    checkGLError(__FILE__, __LINE__);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    gl_state.backup();
  }

}

void Viz3D::render2(const int fb_width, const int fb_height)
{
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
    // TODO(lucasw) instead of outputing to stdout put the error in a gui text widget
    if (checkGLError(__FILE__, __LINE__))
      return;

    // Our visible imgui space lies from draw_data->DisplayPos (top left) to draw_data->DisplayPos+data_data->DisplaySize (bottom right). DisplayMin is typically (0,0) for single viewport apps.
    glViewport(0, 0, (GLsizei)fb_width, (GLsizei)fb_height);

    // (This is to easily allow multiple GL contexts. VAO are not shared among GL contexts, and we don't track creation/deletion of windows so we don't have an obvious key to use to cache them.)
    if (checkGLError(__FILE__, __LINE__))
      return;

  std::shared_ptr<ShaderSet> shaders;
  if (shader_sets_.size() == 0) {
    return;
  }

  // TODO(lucasw) for now just use the last shader set
  for (auto shaders_pair : shader_sets_) {
    shaders = shaders_pair.second;
  }

  for (auto shape_pair : shapes_) {
    auto shape = shape_pair.second;

    // TODO(lucasw) later a shape can use certain shaders or just default
    glUseProgram(shaders->shader_handle_);

    {
      glm::mat4 mvp;
      if (!setupCamera(transform_, shape->frame_id_,
          aov_y_,
          fb_width, fb_height,
          mvp))
        continue;
      // TODO(lucasw) use double in the future?
      // glUniformMatrix4dv(shape->attrib_location_proj_mtx_, 1, GL_FALSE, &mvp[0][0]);
      glUniformMatrix4fv(shaders->attrib_location_proj_mtx_, 1, GL_FALSE, &mvp[0][0]);
      glUniform1i(shaders->attrib_location_tex_, 0);
      if (checkGLError(__FILE__, __LINE__))
        return;
    }

    // TODO(lucasw) add imgui use texture projection checkbox
    const bool use_texture_projection = enable_projected_texture_ &&
        setupProjectedTexture(shape->frame_id_);

#ifdef GL_SAMPLER_BINDING
    glBindSampler(0, 0);
    // We use combined texture/sampler state. Applications using GL 3.3 may set that otherwise.
#endif

    glBindVertexArray(shape->vao_handle_);

    ImVec4 clip_rect = ImVec4(0, 0, fb_width, fb_height);
    glScissor((int)clip_rect.x, (int)(fb_height - clip_rect.w),
        (int)(clip_rect.z - clip_rect.x), (int)(clip_rect.w - clip_rect.y));
    if (checkGLError(__FILE__, __LINE__))
      return;

#if 0
    {
      static bool has_printed = false;
      if (!has_printed) {
        has_printed = true;
        shape->print();
      }
    }
#endif

    // Bind texture- if it is null then the color is black
    // if (texture_id_ != nullptr)
    {
      GLuint tex_id = 0;
      if ((shape->texture_ != "") && (textures_.count(shape->texture_) > 0)) {
        tex_id = (GLuint)(intptr_t)textures_[shape->texture_]->texture_id_;
      } else {
        tex_id = (GLuint)(intptr_t)ros_image_->texture_id_;
      }
      glActiveTexture(GL_TEXTURE0);
      // Bind texture- if it is null then the color is black
      glBindTexture(GL_TEXTURE_2D, tex_id);
      if (checkGLError(__FILE__, __LINE__))
        return;
    }

    if (use_texture_projection) {
      // TODO(lucasw) later the scale could be a brightness setting
      glUniform1f(shaders->attrib_location_projected_texture_scale_, 1.0);
      GLuint tex_id = 0;
      // TODO(lucasw) currently hardcoded, later make more flexible
      const std::string name = projected_texture_name_;
      if (textures_.count(name) > 0) {
        tex_id = (GLuint)(intptr_t)textures_[name]->texture_id_;
      } else {
        tex_id = (GLuint)(intptr_t)ros_image_->texture_id_;
      }
      glActiveTexture(GL_TEXTURE1);
      glBindTexture(GL_TEXTURE_2D, tex_id);
      if (checkGLError(__FILE__, __LINE__))
        return;
    } else {
      // turn off projection in fragment shader, otherwise last updated
      // uniform values will be used
      glUniform1f(shaders->attrib_location_projected_texture_scale_, 0.0);
    }

    {
      glBindBuffer(GL_ARRAY_BUFFER, shape->vbo_handle_);  // needed before bind texture?
      const ImDrawIdx* idx_buffer_offset = 0;
      glDrawElements(GL_TRIANGLES, (GLsizei)shape->indices_.Size,
          sizeof(ImDrawIdx) == 2 ? GL_UNSIGNED_SHORT : GL_UNSIGNED_INT, idx_buffer_offset);
      // std::cout << cmd_i << " " << tex_id << " " << idx_buffer_offset << "\n";
      if (checkGLError(__FILE__, __LINE__))
        return;
      // idx_buffer_offset += pcmd->ElemCount;
    }
  }

  return;
}

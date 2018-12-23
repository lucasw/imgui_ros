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
#include <iomanip>
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

std::string printMat(glm::mat4& mat)
{
  std::stringstream ss;
  for (size_t i = 0; i < 4; ++i) {
    for (size_t j = 0; j < 4; ++j) {
      ss << std::setw(5) << std::fixed << mat[i][j] << " ";
    }
    ss << "\n";
  }
  return ss.str();
}
std::string printMat(glm::dmat4& mat)
{
  glm::mat4 mat2;
  dmat4Todmat(mat, mat2);
  return printMat(mat2);
}

std::string printTransform(tf2::Transform& tf)
{
  std::stringstream ss;
  ss << std::setw(5) << tf.getOrigin().x() << " "
    << std::setw(5) << tf.getOrigin().y() << " "
    << std::setw(5) << tf.getOrigin().z() << "\n";
  return ss.str();
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

void Projector::draw()
{
  ImGui::Text("projector");
  // TODO(lucasw) make this a ros topic, use a regular PUB + SUB gui widget
  // const bool changed =
  std::string text;
  text = "enable##projected" + name_;
  ImGui::Checkbox(text.c_str(), &enable_);
  // TODO(lucasw) switch to ortho projection if angle is zero
  double min = 0.1;
  double max = 170.0;
  text = "aov y##projected" + name_;
  ImGui::SliderScalar(text.c_str(), ImGuiDataType_Double,
      &aov_y_, &min, &max, "%0.2f", 2);

  min = 0.01;
  max = 100.0;
  text = "aspect scale##projected" + name_;
  ImGui::SliderScalar(text.c_str(), ImGuiDataType_Double,
      &aspect_scale_, &min, &max, "%.3f", 2);

  ImGui::Text("%s", render_message_.str().c_str());
  render_message_.str("");
}

// TODO(lucasw) make this work outside of Viz3D
bool Viz3D::setupWithShape(std::shared_ptr<Projector> projector,
    const std::string& main_frame_id,
    const std::string& shape_frame_id)
{
  std::shared_ptr<RosImage> texture;
  if (textures_.count(projector->texture_name_) < 1) {
    render_message_ << "no texture " << projector->texture_name_ << ", ";
    return false;
  }
  texture = textures_[projector->texture_name_];
  if (!texture) {
    render_message_ << "no texture, ";
    return false;
  }
  if (!texture->image_) {
    render_message_ << "no texture image, ";
    return false;
  }

  // this is the view of the projector
  tf2::Stamped<tf2::Transform> stamped_transform;
  try {
    geometry_msgs::msg::TransformStamped tf;
    tf = tf_buffer_->lookupTransform(main_frame_id,
        projector->frame_id_, tf2::TimePointZero);
    tf2::fromMsg(tf, stamped_transform);
    render_message_ << "\nprojected texture transform "
        << main_frame_id << " " << projector->frame_id_ << " "
        << printTransform(stamped_transform);
  } catch (tf2::TransformException& ex) {
    // TODO(lucasw) display exception on gui, but this isn't currently the correct
    // time.
    render_message_ << "\n" << ex.what();
    return false;
  }

  const int width = texture->image_->width;
  const int height = texture->image_->height;
  glm::mat4 mtp;
  if (!setupCamera(stamped_transform, shape_frame_id,
      projector->aov_y_,
      width, height,
      mtp,
      projector->aspect_scale_  // TODO(lucasw) relate to aspect_scale_ also?
      ))
    return false;

  for (auto shaders : shader_sets_) {
    glUniformMatrix4fv(shaders.second->attrib_location_proj_tex_mtx_, 1, GL_FALSE, &mtp[0][0]);
    // assign this texture to the TEXTURE1 slot
    glUniform1i(shaders.second->attrib_location_proj_tex_, 1);
    checkGLError(__FILE__, __LINE__);
  }
  return true;
}
////////////////////////////////////////////////////////////
ShaderSet::~ShaderSet()
{
  remove();
}

void ShaderSet::remove()
{
  std::cout << "Deleting shader set " << name_
      << vert_handle_ << " " << frag_handle_ << " " << shader_handle_ << "\n";
  glDeleteShader(vert_handle_);
  glDeleteShader(frag_handle_);
  glDeleteProgram(shader_handle_);
}

bool ShaderSet::init(const std::string& glsl_version, std::string& message)
{
  if ((vert_handle_ != 0) || (frag_handle_ != 0) || (shader_handle_ != 0)) {
    std::stringstream ss;
    ss << "already initialized, deleting old versions "
        << vert_handle_ << " " << frag_handle_ << " " << shader_handle_ << "\n";
    message += ss.str();
    remove();
  }
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

  std::string msg;
  if (checkGLError2(msg)) {
    return false;
  }

  return true;
}

void ShaderSet::draw()
{
  std::stringstream ss;
  ss << "shader: " << name_ << " " << shader_handle_ << ", vert " << vert_handle_
      << ", geometry " << geometry_handle_ << ", frag " << frag_handle_;
  ss << "\n";
  ss << "tex " << attrib_location_tex_ << ", ";
  ss << "proj " << attrib_location_proj_mtx_ << ", ";
  ss << "pos " << attrib_location_position_ << ", ";
  ss << "uv " << attrib_location_uv_ << ", ";
  ss << "col " << attrib_location_color_ << "\n";

  ss << "proj: ";
  ss << "scale " << attrib_location_projected_texture_scale_ << ", ";
  ss << "tex " << attrib_location_proj_tex_ << ", ";
  ss << "tex mtx " << attrib_location_proj_tex_mtx_ << " ";

  if (false) {
    ss << "\n--------------------------";
    ss << vertex_code_ << "\n";
    ss << "--------------------------";
    ss << geometry_code_ << "\n";
    ss << "--------------------------";
    ss << fragment_code_ << "\n";
    ss << "--------------------------";
  }

  ImGui::Text("%s", ss.str().c_str());
}

///////////////////////////////////////////////////////////////////////////////
Shape::~Shape()
{
  glDeleteBuffers(1, &elements_handle_);
  glDeleteBuffers(1, &vbo_handle_);
  glDeleteVertexArrays(1, &vao_handle_);
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

  checkGLError(__FILE__, __LINE__);
  std::cout << name_ << " init vao " << vao_handle_ << ", "
      << "vbo " << vbo_handle_ << ", elements " << elements_handle_ << ", "
      << "vertices size " << vertices_.Size << ", "
      << "indices size " << indices_.Size << "\n";

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elements_handle_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)indices_.Size * sizeof(ImDrawIdx),
      (const GLvoid*)indices_.Data, GL_STREAM_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

#if 0
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
#endif
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
  render_message_.precision(2);
  render_message_.fill('0');

  glsl_version_string_ = renderer->GlslVersionString;

  const bool sub_not_pub = true;
  textures_["default"] = std::make_shared<RosImage>("default", "/image_out", sub_not_pub, node);

  projector_ = std::make_shared<Projector>("projector1", "projected_texture", "projected_texture");

  transform_.setIdentity();

  textured_shape_sub_ = node->create_subscription<imgui_ros::msg::TexturedShape>(topic,
        std::bind(&Viz3D::texturedShapeCallback, this, _1));

  add_camera_ = node->create_service<imgui_ros::srv::AddCamera>("add_camera",
      std::bind(&Viz3D::addCamera, this, _1, _2));
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
}

void Viz3D::addCamera(const std::shared_ptr<imgui_ros::srv::AddCamera::Request> req,
                      std::shared_ptr<imgui_ros::srv::AddCamera::Response> res)
{
  auto node = node_.lock();
  if (!node) {
    res->message = "couldn't get node for camera '" + req->camera.name + "' '" +
        req->camera.texture_name + "'";
    res->success = false;
    return;
  }
  try {
    auto render_texture = std::make_shared<Camera>(req->camera.name,
        req->camera.texture_name,
        req->camera.header.frame_id, req->camera.topic,
        req->camera.width, req->camera.height,
        req->camera.aov_y,
        node);
    textures_[req->camera.texture_name] = render_texture->image_;
    cameras_[req->camera.name] = render_texture;
  } catch (std::runtime_error& ex) {
    res->message = ex.what();
    res->success = false;
    return;
  }
  res->message = "added camera '" + req->camera.name + "' '" + req->camera.texture_name + "'";
  res->success = true;
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

  if (req->name != "default") {
    res->message = "only 1 set of shaders currently supported in 'default' slot";
    res->success = false;
    return;
  }

  auto shaders = std::make_shared<ShaderSet>(req->name, req->vertex,
      req->geometry, req->fragment);

  std::lock_guard<std::mutex> lock(mutex_);
  if (!shaders->init(glsl_version_string_, res->message)) {
    res->success = false;
    return;
  }

  for (auto shape_pair : shapes_) {
    auto shape = shape_pair.second;
    if (!updateShaderShapes(shaders, shape)) {
      res->message = "couldn't update shape " + shape->name_ + " vaos with new shaders ";
      res->success = false;
      return;
    }
  }

  res->message = "succesfully added shaders `" + shaders->name_ + "'";

  shader_sets_[req->name] = shaders;
}

// need to update the connnection between the shader and the shape
// whenever either is replaced
bool Viz3D::updateShaderShapes(std::shared_ptr<ShaderSet> shaders, std::shared_ptr<Shape> shape)
{
  // std::shared_ptr<rclcpp::Node> node = node_.lock();
  // std::stringstream ss;
  std::cout << "updating shape shader connections '"
      << shape->name_ << "' to '" << shaders->name_ << "'\n";
  std::cout << "vao handle: " << shape->vao_handle_ << ", ";
  std::cout << "vbo handle: " << shape->vbo_handle_ << ", ";

  glBindVertexArray(shape->vao_handle_);
  glEnableVertexAttribArray(shaders->attrib_location_position_);
  glEnableVertexAttribArray(shaders->attrib_location_uv_);
  glEnableVertexAttribArray(shaders->attrib_location_color_);
  // TODO(lucasw) check GL


  std::cout << "attribs: " << shaders->attrib_location_position_ << " "
      << shaders->attrib_location_uv_ << " "
      << shaders->attrib_location_color_ << "\n";

  glBindBuffer(GL_ARRAY_BUFFER, shape->vbo_handle_);
  glVertexAttribPointer(shaders->attrib_location_position_, 3, GL_FLOAT, GL_FALSE,
      sizeof(DrawVert), (GLvoid*)offsetof(DrawVert, pos));
  glVertexAttribPointer(shaders->attrib_location_uv_, 2, GL_FLOAT, GL_FALSE,
      sizeof(DrawVert), (GLvoid*)offsetof(DrawVert, uv));
  glVertexAttribPointer(shaders->attrib_location_color_, 4, GL_FLOAT, GL_FALSE,
      sizeof(DrawVert), (GLvoid*)offsetof(DrawVert, col));

#if 0
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
#endif
  checkGLError(__FILE__, __LINE__);
  return true;
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
  res->success = true;
  for (auto textured_shape : req->shapes) {
    auto shape = std::make_shared<imgui_ros::msg::TexturedShape>(textured_shape);
    if (!addShape2(shape, res->message)) {
      res->success = false;
    }
    res->message += " " + shape->name;
  }
  return;
}

// TODO(lucasw) Shape -> Mesh?
void Viz3D::texturedShapeCallback(const imgui_ros::msg::TexturedShape::SharedPtr msg)
{
  std::string message;
  addShape2(msg, message);
  // TODO(lucasw) make a macro or function for this
  // std::shared_ptr<rclcpp::Node> node = node_.lock();
  // RCLCPP_INFO(node->get_logger(), message);
}

bool Viz3D::addShape2(const imgui_ros::msg::TexturedShape::SharedPtr msg, std::string& message)
{
  if (msg->name == "") {
    message += "mesh needs name";
    return false;
  }

  if (msg->add == false) {
    if (shapes_.count(msg->name) > 0) {
      shapes_.erase(msg->name);
      message += msg->name + " erased";
      return true;
    }
    message += msg->name + " doesn't exist, can't delete it";
    return true;
  }

  const glm::vec4 default_color = glm::vec4(1.0, 1.0, 1.0, 1.0);

  auto shape = std::make_shared<Shape>();
  shape->name_ = msg->name;
  shape->frame_id_ = msg->header.frame_id;
  shape->tf_buffer_ = tf_buffer_;
  shape->texture_ = msg->texture;

  // TODO(lucasw) if is_topic then create RosImage subscriber
  // if msg->image isn't empty create a RosImage and initialize the image
  // with it (RosImage doesn't support that yet).

  for (size_t i = 0; i < msg->vertices.size(); ++i) {
    DrawVert p1;
    p1.pos.x = msg->vertices[i].vertex.x;
    p1.pos.y = msg->vertices[i].vertex.y;
    p1.pos.z = msg->vertices[i].vertex.z;
    p1.uv.x = msg->vertices[i].uv.x;
    p1.uv.y = msg->vertices[i].uv.y;
    // These colors multiply with the texture color
    if (true) {
      p1.col.x = msg->vertices[i].color.r;
      p1.col.y = msg->vertices[i].color.g;
      p1.col.z = msg->vertices[i].color.b;
      p1.col.w = msg->vertices[i].color.a;
    } else {
      p1.col = default_color;
    }
    shape->vertices_.push_back(p1);
  }

  for (size_t i = 0; i < msg->triangles.size(); ++i) {
    for (size_t j = 0; j < msg->triangles[i].vertex_indices.size(); ++j) {
      const int ind = msg->triangles[i].vertex_indices[j];
      if ((ind < 0) || (ind >= shape->vertices_.Size)) {
        std::cerr << "bad triangle index " << ind << " >= "
            << shape->vertices_.Size << "\n";
        // TODO(lucasw) or set to zero, or Size - 1?
        return false;
      }
      shape->indices_.push_back(ind);
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  shape->init();
  message += ", " + shape->print();

  if (shader_sets_.count("default") < 1) {
    // this isn't a failure, just a race condition probably
    std::cout << "no shaders yet, will retry when one is set\n";
  } else {
    // TODO(lucasw) for now just use the last shader set
    auto shaders = shader_sets_["default"];
    if (!updateShaderShapes(shaders, shape)) {
      message += "couldn't update shapes vao with new shaders";
      return false;
    }
  }

  shapes_[shape->name_] = shape;
  return true;
}

void Viz3D::draw()
{
  for (auto camera : cameras_) {
    camera.second->draw();
  }

  ImGui::Begin("viz3d");
  // ImGuiIO& io = ImGui::GetIO();

  ImGui::Separator();
  ImGui::Text("shapes");
  for (auto shape_pair : shapes_) {
    ImGui::Separator();
    auto shape = shape_pair.second;
    shape->draw();
  }
  ImGui::Separator();
  ImGui::Text("textures");
  for (auto texture_pair : textures_) {
    ImGui::Separator();
    // text status, only optionally display the actual image with checkbox
    texture_pair.second->draw();
  }
  ImGui::Separator();
  std::stringstream ss;
  ss << "shaders " << shader_sets_.size();
  ImGui::Text("%s", ss.str().c_str());
  for (auto shaders_pair : shader_sets_) {
    ImGui::Separator();
    shaders_pair.second->draw();
  }
  ImGui::Separator();
  projector_->draw();
  ImGui::Separator();
  ImGui::Text("main camera");
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

  ss.str("");
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

  if (!ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow |
      ImGuiHoveredFlags_AllowWhenBlockedByActiveItem)) {
  // if (!ImGui::IsRootWindowOrAnyChildHovered()) {
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
      &aov_y_, &min, &max, "%lf", 2);

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

  {
    ImGui::Separator();
    ImGui::Checkbox("debug output", &enable_render_message_);
    if (enable_render_message_) {
      ImGui::Text("%s", render_message_.str().c_str());
    }
  }
  ImGui::End();
}

// Currently calling setupcamera for every object- that seems efficient vs.
// transforming all the data of every object.
bool Viz3D::setupCamera(const tf2::Transform& view_transform,
    const std::string child_frame_id, const double aov_y,
    const int fb_width, const int fb_height, glm::mat4& mvp,
    float aspect_scale, float sc_vert)
{
  const float aspect = static_cast<float>(fb_width) / static_cast<float>(fb_height) * aspect_scale;
  glm::mat4 projection_matrix = glm::perspective(static_cast<float>(sc_vert * glm::radians(aov_y)),
      sc_vert * aspect, near_, far_);

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
    render_message_ << ", frames: " << frame_id_ << " -> " << child_frame_id << "\n";
    glm::dmat4 model_matrix_double;
    stamped_transform.getOpenGLMatrix(&model_matrix_double[0][0]);
    dmat4Todmat(model_matrix_double, model_matrix);
  } catch (tf2::TransformException& ex) {
    render_message_ << "\n" << ex.what();
    return false;
  }

  glm::dmat4 view_matrix_double;
  view_transform.inverse().getOpenGLMatrix(&view_matrix_double[0][0]);
  // printMat(view_matrix_double, "view_matrix double");
  glm::mat4 view_matrix;
  dmat4Todmat(view_matrix_double, view_matrix);

  mvp = projection_matrix * view_matrix * model_matrix;

  if (false) {
    render_message_ << "\nmatrices:\n";
    render_message_ << "model " << child_frame_id << ":\n" << printMat(model_matrix);
    render_message_ << "view:\n" << printMat(view_matrix);
    render_message_ << "projection:\n" << printMat(projection_matrix);
    render_message_ << "mvp:\n" << printMat(mvp);
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

  render_message_.str("");

  if (shapes_.size() == 0) {
    render_message_ << "no shapes to render";
    return;
  }

  if (fb_width <= 0 || fb_height <= 0) {
    render_message_ << "bad width height " << fb_width << " " << fb_height << "\n";
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

  render_message_ << "textures " << textures_.size();

  for (auto texture_pair : textures_) {
    texture_pair.second->updateTexture();
  }

    GLState gl_state;
    gl_state.backup();

    // TODO(lucasw) later render to texture with a variable width and height
    // and then draw the texture to the screen with a textured quad fragment
    // shader, this allows rendering at a different resolution that the screen
    // which might be desirable if performance is suffering.
    render2(transform_, fb_width, fb_height, aov_y_);

    gl_state.backup();
    checkGLError(__FILE__, __LINE__);
}

// don't interleave this with regular 3d rendering imgui rendering
void Viz3D::renderToTexture()
{
  if (cameras_.size() == 0)
    return;

  GLState gl_state;
  gl_state.backup();
  for (auto camera_pair : cameras_) {
    auto camera = camera_pair.second;
    if (!camera->enable_) {
      return;
    }
    render_message_ << "\nrender to texture " << camera->name_ << "\n";

    glBindFramebuffer(GL_FRAMEBUFFER, camera->frame_buffer_);
    glClearColor(0.1, 0.1, 5.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // TODO(lucasw) if render width/height change need to update rendered_texture

    try {
      geometry_msgs::msg::TransformStamped tf;
      tf = tf_buffer_->lookupTransform(frame_id_,
          camera->frame_id_, tf2::TimePointZero);
      tf2::fromMsg(tf, camera->stamped_transform_);
      #if 0
      tf2::TimePoint time_out;
      // this is private, so doesn't work
      tf_buffer_->lookupTransformImpl(frame_id_, child_frame_id,
          tf2::TimePointZero, transform, time_out);
      #endif
      render_message_ << ", frames: " << frame_id_ << " -> "
          << camera->frame_id_ << "\n";
    } catch (tf2::TransformException& ex) {
      render_message_ << "\n" << ex.what();
      continue;
    }

    render2(camera->stamped_transform_,
        camera->image_->width_,
        camera->image_->height_,
        camera->aov_y_,
        -1.0);

    // TODO(lucasw) copy the date from the texture out to a cv::Mat?
    checkGLError(__FILE__, __LINE__);
  }
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  gl_state.backup();
}

void Viz3D::render2(const tf2::Transform& transform,
    const int fb_width, const int fb_height,
    const float aov_y,
    const float sc_vert)
{
  // TODO(lucasw) should give up if can't lock, just don't render now
  std::lock_guard<std::mutex> lock(mutex_);

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
  if (shader_sets_.count("default") < 1) {
    return;
  }
  // TODO(lucasw) for now just use the last shader set
  shaders = shader_sets_["default"];
  render_message_ << ", shader " << shaders->name_ << " " << shaders->shader_handle_;
  // for (auto shaders_pair : shader_sets_) {
  //  shaders = shaders_pair.second;
  //}
  render_message_ << "\n";

  // TEMP
  std::shared_ptr<Shape> first_shape;

  for (auto shape_pair : shapes_) {
    auto shape = shape_pair.second;
    // TEMP
    if (first_shape == nullptr)
      first_shape = shape;

    render_message_ << "shape: " << shape->name_;

    // TODO(lucasw) later a shape can use certain shaders or just default
    glUseProgram(shaders->shader_handle_);
    {
      glm::mat4 mvp;
      if (!setupCamera(transform, shape->frame_id_,
          aov_y,
          fb_width, fb_height,
          mvp, aspect_scale_, sc_vert))
        continue;
      // TODO(lucasw) use double in the future?
      // glUniformMatrix4dv(shape->attrib_location_proj_mtx_, 1, GL_FALSE, &mvp[0][0]);
      glUniformMatrix4fv(shaders->attrib_location_proj_mtx_, 1, GL_FALSE, &mvp[0][0]);
      glUniform1i(shaders->attrib_location_tex_, 0);
      if (checkGLError(__FILE__, __LINE__))
        return;
    }

    // TODO(lucasw) add imgui use texture projection checkbox
    const bool use_texture_projection = projector_->enable_ &&
        setupWithShape(projector_, frame_id_, shape->frame_id_);

#ifdef GL_SAMPLER_BINDING
    glBindSampler(0, 0);
    // We use combined texture/sampler state. Applications using GL 3.3 may set that otherwise.
#endif

    glBindVertexArray(shape->vao_handle_);
    render_message_ << "vao handle " << shape->vao_handle_;

    ImVec4 clip_rect = ImVec4(0, 0, fb_width, fb_height);
    glScissor((int)clip_rect.x, (int)(fb_height - clip_rect.w),
        (int)(clip_rect.z - clip_rect.x), (int)(clip_rect.w - clip_rect.y));
    if (checkGLError(__FILE__, __LINE__))
      return;

    render_message_ << "\n";

    // Bind texture- if it is null then the color is black
    // if (texture_id_ != nullptr)
    {
      GLuint tex_id = 0;
      if ((shape->texture_ != "") && (textures_.count(shape->texture_) > 0)) {
        tex_id = (GLuint)(intptr_t)textures_[shape->texture_]->texture_id_;
        render_message_ << "texture " << shape->texture_ << " " << tex_id;
      } else if (textures_.count("default") > 0) {
        tex_id = (GLuint)(intptr_t)textures_["default"]->texture_id_;
        render_message_ << ", default texture " << tex_id;
      } else {
        // TODO(lucasw) else stop rendering?
        render_message_ << ", no texture to use";
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
      const std::string name = projector_->texture_name_;
      if (textures_.count(name) > 0) {
        tex_id = (GLuint)(intptr_t)textures_[name]->texture_id_;
        render_message_ << "\nprojected texture " << name << " " << tex_id;
      } else if (textures_.count("default") > 0) {
        tex_id = (GLuint)(intptr_t)textures_["default"]->texture_id_;
        render_message_ << "\ndefault projected texture " << tex_id;
      } else {
        render_message_ << ", no texture to use";
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
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, shape->elements_handle_);
      const ImDrawIdx* idx_buffer_offset = 0;
      glDrawElements(GL_TRIANGLES, (GLsizei)shape->indices_.Size,
          sizeof(ImDrawIdx) == 2 ? GL_UNSIGNED_SHORT : GL_UNSIGNED_INT, idx_buffer_offset);
      // std::cout << cmd_i << " " << tex_id << " " << idx_buffer_offset << "\n";
      if (checkGLError(__FILE__, __LINE__))
        return;
      render_message_ << ", shape " << shape->name_ << " indices " << shape->indices_.Size;
      // idx_buffer_offset += pcmd->ElemCount;
      // glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
    render_message_ << "\n\n";
    // glBindVertexArray(0);
  }

  return;
}

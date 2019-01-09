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

// TODO(lucasw) move to utility file
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

std::string printVec(glm::vec3& vec)
{
  std::stringstream ss;
  for (size_t i = 0; i < 3; ++i) {
    ss << std::setw(3) << std::fixed << vec[i] << " ";
  }
  ss << "\n";
  return ss.str();
}

std::string printVec(glm::vec4& vec)
{
  std::stringstream ss;
  for (size_t i = 0; i < 4; ++i) {
    ss << std::setw(3) << std::fixed << vec[i] << " ";
  }
  ss << "\n";
  return ss.str();
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

/////////////////////////////////////////////////////////////////
// TODO(lucasw) make this work outside of Viz3D
bool Viz3D::setupProjectorsWithShape(
    const std::string& main_frame_id,
    const std::string& shape_frame_id,
    std::vector<std::shared_ptr<Projector> >& projectors)
{
  glm::mat4 view[MAX_PROJECTORS], view_inverse[MAX_PROJECTORS], projection[MAX_PROJECTORS];
  float projected_texture_scale[MAX_PROJECTORS];
  for (size_t i = 0; i < MAX_PROJECTORS; ++i) {
    projected_texture_scale[i] = 0.0;
  }

  size_t ind = 0;
  for (auto projector_pair : projectors_) {
    if (ind >= MAX_PROJECTORS)
      break;
    auto projector = projector_pair.second;
    if (!projector->enable_) {
      continue;
    }
    // TODO(lucasw) select from all projectors the ones
    // closest to the current shape and exclude the rest.

    std::shared_ptr<RosImage> texture;

    if (textures_.count(projector->texture_name_) < 1) {
      // render_message_ << projector->name_ << " no texture in textures "
      //    << projector->texture_name_ << ", ";
      continue;
    }
    texture = textures_[projector->texture_name_];
    if (!texture) {
      // render_message_ << " '" << projector->name_ << "' has no texture '"
      //    << projector->texture_name_ << "', ";
      continue;
    }
    if (!texture->image_) {
      // render_message_ << projector->name_ << " no texture image "
      //    << projector->texture_name_ << ", ";
      continue;
    }

    // this is the view of the projector
    tf2::Stamped<tf2::Transform> stamped_transform;
    try {
      geometry_msgs::msg::TransformStamped tf;
      tf = tf_buffer_->lookupTransform(main_frame_id,
          projector->frame_id_, tf2::TimePointZero);
      tf2::fromMsg(tf, stamped_transform);
      // render_message_ << "projected texture transform "
      //     << main_frame_id << " " << projector->frame_id_ << " "
      //     << printTransform(stamped_transform);
    } catch (tf2::TransformException& ex) {
      // TODO(lucasw) display exception on gui, but this isn't currently the correct
      // time.
      // render_message_ << projector->name_ << "\n" << ex.what();
      continue;
    }

    const int width = texture->image_->width;
    const int height = texture->image_->height;
    // TODO(lucasw) it is redundant to setup model repeatedly,
    // refactor to do that just once
    glm::mat4 model;
    if (!setupCamera(stamped_transform,
        frame_id_, shape_frame_id,
        projector->aov_y_,
        projector->aov_x_,
        projector->near_,
        projector->far_,
        width, height,
        model,
        view[ind],
        view_inverse[ind],
        projection[ind],
        false
        )) {
      continue;
    }

    // enable the projector
    projected_texture_scale[ind] = 1.0;
    projectors.push_back(projector);
    ind += 1;
  }

  // render_message_ << "\nview inverse\n" << printMat(view_inverse[0]) << "\n";
  // glm::vec4 projector_pos = view_inverse[0] * glm::vec4(0.0, 0.0, 0.0, 1.0);
  // render_message_ << "projector pos " << printVec(projector_pos) << "\n";

  // corresponds to glActiveTexture(GL_TEXTURE1) if is 1

  for (auto shaders_pair : shader_sets_) {
    auto shaders = shaders_pair.second;
    auto transpose = GL_FALSE;
    // glUniformMatrix4fv(shaders.second->uniform_locations_["projector_model_matrix"],
    //     1, GL_FALSE, &model[0][0]);
    glUniformMatrix4fv(shaders->uniform_locations_["projector_view_matrix"],
        MAX_PROJECTORS, transpose, &view[0][0][0]);
    glUniformMatrix4fv(shaders->uniform_locations_["projector_view_matrix_inverse"],
        MAX_PROJECTORS, transpose, &view_inverse[0][0][0]);
    glUniformMatrix4fv(shaders->uniform_locations_["projector_projection_matrix"],
        MAX_PROJECTORS, transpose, &projection[0][0][0]);

    // assign these textures to the TEXTURE2..6 slots
    glUniform1iv(shaders->uniform_locations_["ProjectedTexture"],
        MAX_PROJECTORS, &projector_texture_unit_[0]);
    glUniform1iv(shaders->uniform_locations_["projector_shadow_map"],
        MAX_PROJECTORS, &shadow_texture_unit_[0]);

    glUniform1fv(shaders->uniform_locations_["projected_texture_scale"],
        MAX_PROJECTORS, &projected_texture_scale[0]);
    checkGLError(__FILE__, __LINE__);
  }

  return true;
}

///////////////////////////////////////////////////////////////////
// render the entire background
// this probably will be split out into a widget also.
Viz3D::Viz3D(const std::string name,
    const std::string topic,
    std::shared_ptr<ImGuiImplOpenGL3> renderer,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<rclcpp::Node> node) :
    Window(name),
    tf_buffer_(tf_buffer),
    node_(node)
{
  // render_message_.precision(2);
  // render_message_.fill('0');

  glsl_version_string_ = renderer->GlslVersionString;

  // TODO(lucasw) set via service all
  ambient_ = glm::vec3(0.3, 0.3, 0.3);

  const bool sub_not_pub = true;
  textures_["default"] = std::make_shared<RosImage>("default", "default_texture", sub_not_pub, node);

  transform_.setIdentity();

  // tf_broadcaster_ = TfBroadcaster("viewer_camera",
  //     "map", "viewer_camera",
  //     0.0, 0.0, tf_buffer_, node);

  textured_shape_sub_ = node->create_subscription<imgui_ros::msg::TexturedShape>(topic,
        std::bind(&Viz3D::texturedShapeCallback, this, _1));

  int base_tex_ind = 2;
  for (size_t i = 0; i < MAX_PROJECTORS; ++i) {
    projector_texture_unit_[i] = base_tex_ind + i;
    shadow_texture_unit_[i] = base_tex_ind + MAX_PROJECTORS + i;
  }
  texture_unit_["Texture"] = 0;
  texture_unit_["shininess_texture"] = 1;

  add_camera_ = node->create_service<imgui_ros::srv::AddCamera>("add_camera",
      std::bind(&Viz3D::addCamera, this, _1, _2));
  add_cube_camera_ = node->create_service<imgui_ros::srv::AddCubeCamera>("add_cube_camera",
      std::bind(&Viz3D::addCubeCamera, this, _1, _2));
  add_projector_ = node->create_service<imgui_ros::srv::AddProjector>("add_projector",
      std::bind(&Viz3D::addProjector, this, _1, _2));
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
        req->camera.header.frame_id,
        req->camera.aov_y,
        req->camera.aov_x,
        node);
    render_texture->init(
        req->camera.width, req->camera.height,
        req->camera.texture_name,
        req->camera.topic,
        node);
    render_texture->near_ = req->camera.near;
    render_texture->far_ = req->camera.far;
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

void Viz3D::addCubeCamera(const std::shared_ptr<imgui_ros::srv::AddCubeCamera::Request> req,
                      std::shared_ptr<imgui_ros::srv::AddCubeCamera::Response> res)
{
  auto node = node_.lock();
  if (!node) {
    res->message = "couldn't get node for camera '" + req->camera.name + "' '" +
        req->camera.texture_name + "'";
    res->success = false;
    return;
  }
  try {

  #if 0
    const std::string texture_name = "test_cube_camera";
    auto cube_camera = std::make_shared<CubeCamera>(
        "test_cube_camera",
        "cube_camera",
        90.0, 90.0,
        node);
    cube_camera->init(800, 800, 512, texture_name, "", node);
    cube_cameras_["test"] = cube_camera;
    // TODO(lucasw) see what happens if imgui tries to draw the cubemap
    // cameras_[cube_camera->name_] = cube_camera;
    textures_[texture_name] = cube_camera->image_;
  #endif

    auto cube_camera = std::make_shared<CubeCamera>(
        req->camera.name,
        req->camera.header.frame_id,
        req->camera.aov_y,
        req->camera.aov_x,
        node);
    cube_camera->init(
        req->camera.width, req->camera.height,
        req->face_width,
        req->camera.texture_name,
        req->camera.topic,
        node);
    cube_camera->near_ = req->camera.near;
    cube_camera->far_ = req->camera.far;
    textures_[req->camera.texture_name] = cube_camera->image_;
    cube_cameras_[req->camera.name] = cube_camera;
  } catch (std::runtime_error& ex) {
    res->message = ex.what();
    res->success = false;
    return;
  }
  res->message = "added camera '" + req->camera.name + "' '" + req->camera.texture_name + "'";
  res->success = true;
}


void Viz3D::addProjector(const std::shared_ptr<imgui_ros::srv::AddProjector::Request> req,
                         std::shared_ptr<imgui_ros::srv::AddProjector::Response> res)
{
  const std::string name = req->projector.camera.name;
  // const std::string texture_name = texture_name; -> std::bad_alloc - why compile at all?
  const std::string texture_name = req->projector.camera.texture_name;
  auto node = node_.lock();
  if (!node) {
    res->message = "couldn't get node for projector '" + name + "' '" +
        texture_name + "'";
    res->success = false;
    return;
  }

  if ((projectors_.count(name) < 1) &&
      (projectors_.size() >= MAX_PROJECTORS)) {
    res->message = "only supporting " + std::to_string(MAX_PROJECTORS)
        + " currently, need to remove any existing: ";
    for (auto projector_pair : projectors_) {
      res->message += projector_pair.second->name_ + ", ";
    }
    res->success = false;
    return;
  }

  try {
    auto projector = std::make_shared<Projector>(
        name,
        texture_name,
        req->projector.camera.header.frame_id,
        req->projector.camera.aov_y,
        req->projector.camera.aov_x,
        req->projector.max_range,
        req->projector.constant_attenuation,
        req->projector.linear_attenuation,
        req->projector.quadratic_attenuation,
        node);
    projector->near_ = req->projector.camera.near;
    projector->far_ = req->projector.camera.far;
    projectors_[name] = projector;
  } catch (std::runtime_error& ex) {
    res->message = ex.what();
    res->success = false;
    return;
  }
  res->message = "added projector '" + name + "', texture '" + texture_name + "'";
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

  auto shaders = std::make_shared<ShaderSet>(req->name, req->vertex,
      req->geometry, req->fragment);

  std::lock_guard<std::mutex> lock(mutex_);
  if (!shaders->init(glsl_version_string_, res->message)) {
    res->success = false;
    return;
  }

  for (auto shape_pair : shapes_) {
    auto shape = shape_pair.second;
    if (!shape) {
      res->message = "bad shape " + shape_pair.first;
      res->success = false;
      continue;
    }
    if (!updateShaderShapes(shaders, shape)) {
      res->message = "couldn't update shape " + shape->name_ + " vaos with new shaders ";
      res->success = false;
      return;
    }
  }

  res->message = "successfully added shaders `" + shaders->name_ + "'";

  shader_sets_[req->name] = shaders;
}

// need to update the connnection between the shader and the shape
// whenever either is replaced
bool Viz3D::updateShaderShapes(std::shared_ptr<ShaderSet> shaders, std::shared_ptr<Shape> shape)
{
  // std::shared_ptr<rclcpp::Node> node = node_.lock();
  // std::stringstream ss;
  std::cout << "updating shape shader connections '"
      << shape->name_ << "' to '" << shaders->name_ << "'" << std::endl;  //"\n";
  std::cout << "vao handle: " << shape->vao_handle_ << ", ";
  std::cout << "vbo handle: " << shape->vbo_handle_ << ", ";

  glBindVertexArray(shape->vao_handle_);
  glBindBuffer(GL_ARRAY_BUFFER, shape->vbo_handle_);

  if (shaders->attrib_locations_.count("Position") > 0) {
    glEnableVertexAttribArray(shaders->attrib_locations_["Position"]);
    glVertexAttribPointer(shaders->attrib_locations_["Position"], 3, GL_FLOAT, GL_FALSE,
        sizeof(DrawVert), (GLvoid*)offsetof(DrawVert, pos));
  }

  if (shaders->attrib_locations_.count("Normal") > 0) {
    glEnableVertexAttribArray(shaders->attrib_locations_["Normal"]);
    glVertexAttribPointer(shaders->attrib_locations_["Normal"], 3, GL_FLOAT, GL_FALSE,
        sizeof(DrawVert), (GLvoid*)offsetof(DrawVert, nrm));
  }

  if (shaders->attrib_locations_.count("UV") > 0) {
    glEnableVertexAttribArray(shaders->attrib_locations_["UV"]);
    glVertexAttribPointer(shaders->attrib_locations_["UV"], 2, GL_FLOAT, GL_FALSE,
        sizeof(DrawVert), (GLvoid*)offsetof(DrawVert, uv));
  }

  if (shaders->attrib_locations_.count("Color") > 0) {
    glEnableVertexAttribArray(shaders->attrib_locations_["Color"]);
    glVertexAttribPointer(shaders->attrib_locations_["Color"], 4, GL_FLOAT, GL_FALSE,
        sizeof(DrawVert), (GLvoid*)offsetof(DrawVert, col));
  }

  // TODO(lucasw) check GL
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
  texture->draw_texture_controls_ = true;
  texture->imageCallback(std::make_shared<sensor_msgs::msg::Image>(req->image));
  texture->wrap_s_ind_ = req->wrap_s;
  texture->wrap_t_ind_ = req->wrap_t;
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

  auto shape = std::make_shared<Shape>(msg->name,
      msg->header.frame_id, msg->texture, msg->shininess_texture, tf_buffer_);
  shape->enable_ = msg->enable;

  // TODO(lucasw) if is_topic then create RosImage subscriber
  // if msg->image isn't empty create a RosImage and initialize the image
  // with it (RosImage doesn't support that yet).

  for (size_t i = 0; i < msg->vertices.size(); ++i) {
    DrawVert p1;
    p1.pos.x = msg->vertices[i].vertex.x;
    p1.pos.y = msg->vertices[i].vertex.y;
    p1.pos.z = msg->vertices[i].vertex.z;
    p1.nrm.x = msg->vertices[i].normal.x;
    p1.nrm.y = msg->vertices[i].normal.y;
    p1.nrm.z = msg->vertices[i].normal.z;
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

  // TODO(lucasw) doesn't 'depth' need this update also?
  if (shader_sets_.size() < 1) {
    // this isn't a failure, just a race condition probably
    std::cout << "no shaders yet, will retry when one is set\n";
  } else {
    // TODO(lucasw) for now just use the last shader set
    for (auto shader_pair : shader_sets_) {
      auto shaders = shader_pair.second;
      if (!updateShaderShapes(shaders, shape)) {
        message += "couldn't update shapes vao with new shadpers "
            + shape->name_ + " " + shaders->name_;
        continue;
      }
    }
  }

  shapes_[shape->name_] = shape;
  return true;
}

void Viz3D::update()
{
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

  tf2::Vector3 translation = transform_.getOrigin();
  translation = translation + vel_in_world;
  transform_.setOrigin(translation);

  velocity_ *= 0.8;

  // mouse input
  ImVec2 mouse_pos_in_canvas = ImGui::GetIO().MousePos;

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

  // update
  for (auto camera : cameras_) {
    // TODO(lucasw) are all these textures also in the texture list below?
    camera.second->image_->updateTexture();
    camera.second->image_->publish();
  }
  for (auto texture_pair : textures_) {
    texture_pair.second->updateTexture();
  }
}

void Viz3D::draw()
{
  ImGui::Begin("viz3d");

  // TODO(lucasw) show render time, update time, draw time
  // show current resolution

  // ImGuiIO& io = ImGui::GetIO();
  if (ImGui::CollapsingHeader("Viewer")) {

    const int display_size_x = ImGui::GetIO().DisplaySize.x;
    const int display_size_y = ImGui::GetIO().DisplaySize.y;
    const int fb_width = display_size_x * ImGui::GetIO().DisplayFramebufferScale.x;
    const int fb_height = display_size_y * ImGui::GetIO().DisplayFramebufferScale.y;
    ImGui::Text("size %d x %d, %d x %d",
        display_size_x, display_size_y,
        fb_width, fb_height);

    ImGui::ColorEdit4("clear color", (float*)&clear_color_);
    ImGui::ColorEdit3("ambient color", (float*)&ambient_[0]);

    ImGui::Text("position %0.2lf %0.2lf %0.2lf",
        transform_.getOrigin().x(),
        transform_.getOrigin().y(),
        transform_.getOrigin().z());
    ImGui::Text("velocity in view %0.2lf %0.2lf %0.2lf",
        velocity_.x(), velocity_.y(), velocity_.z());
    // ImGui::Text("velocity in world %0.2lf %0.2lf %0.2lf",
    //     vel_in_world.x(), vel_in_world.y(), vel_in_world.z());

    {
      double min = 0.0001;
      double max = far_;
      ImGui::SliderScalar("near clip", ImGuiDataType_Double,
            &near_, &min, &max, "%lf", 3);
      min = near_;
      max = 100.0;
      ImGui::SliderScalar("far clip", ImGuiDataType_Double,
            &far_, &min, &max, "%lf", 3);
    }

    {
      double min = 0.0001;
      double max = 0.1;
      ImGui::SliderScalar("move scale", ImGuiDataType_Double,
            &move_scale_, &min, &max, "%lf");
    }

    {
      double min = 5.0;
      double max = 500.0;
      ImGui::SliderScalar("rotate scale", ImGuiDataType_Double,
            &rotate_scale_, &min, &max, "%lf");
    }

    // Maybe aovy should be a ros topic,
    // and the slider a regular Pub widget.
    double min = 1.0;
    double max = 170.0;
    ImGui::SliderScalar("aov y##viz3d", ImGuiDataType_Double,
        &aov_y_, &min, &max, "%lf", 2);

    min = 0.0;
    ImGui::SliderScalar("aov x##viz3d", ImGuiDataType_Double,
        &aov_x_, &min, &max, "%lf", 2);

    {
      std::stringstream ss;
      auto translation = transform_.getOrigin();
      ss << translation.x()  << " " << translation.y() << " " << translation.z() << ", "
          << pitch_;
      ImGui::Text("%s", ss.str().c_str());
    }
  }

  const bool draw_cameras = ImGui::CollapsingHeader("Cameras");
  if (draw_cameras) {
    for (auto camera : cameras_) {
      ImGui::Separator();
      camera.second->draw();
    }
  }

  const bool draw_cube_cameras = ImGui::CollapsingHeader("Cube Cameras");
  if (draw_cube_cameras) {
    for (auto cube_camera_pair : cube_cameras_) {
      ImGui::Separator();
      cube_camera_pair.second->draw();
    }
  }

  std::vector<std::string> texture_names;
  std::string texture_items = "";
  const bool draw_textures = ImGui::CollapsingHeader("Textures");
  for (auto texture_pair : textures_) {
    // text status, only optionally display the actual image with checkbox
    if (draw_textures) {
      ImGui::Separator();
      texture_pair.second->draw();
    }
    texture_names.push_back(texture_pair.first);
    texture_items += texture_pair.first + '\0';
  }
  if (ImGui::CollapsingHeader("Projectors")) {
    ImGui::Text("projectors %lu", projectors_.size());
    for (auto projector_pair : projectors_) {
      ImGui::Separator();
      projector_pair.second->draw(texture_names, texture_items);
    }
  }
  if (ImGui::CollapsingHeader("Surfaces")) {
    for (auto shape_pair : shapes_) {
      ImGui::Separator();
      const auto shape = shape_pair.second;
      try {
        shape->draw(texture_names, texture_items);
      } catch (std::logic_error& ex) {
        // render_message_ << ex.what() << "\n";
      }
    }
  }
  if (ImGui::CollapsingHeader("Shaders")) {
    std::stringstream ss;
    ss << "shaders " << shader_sets_.size();
    ImGui::Text("%s", ss.str().c_str());
    for (auto shaders_pair : shader_sets_) {
      ImGui::Separator();
      shaders_pair.second->draw();
    }
  }
  if (ImGui::CollapsingHeader("Debug")) {
    ImGui::Text("%s", render_message_.str().c_str());
  }
  ImGui::End();
}

void Viz3D::addTF(tf2_msgs::msg::TFMessage& tfm, const rclcpp::Time& now)
{
  // convert tf2::Transform to geometry_msgs TransformStamped
  geometry_msgs::msg::TransformStamped ts;
  ts.transform = tf2::toMsg(transform_);
  ts.header.stamp = now;
  ts.header.frame_id = frame_id_;
  // TODO(lucasw) make this name configurable
  ts.child_frame_id = main_window_frame_id_;
  tfm.transforms.push_back(ts);
}

// Currently calling setupcamera for every object- that seems efficient vs.
// transforming all the data of every object.
// frame id is the base frame of everything-
// it doesn't matter where it is or if it is moving,
// unless it is so far away from the camera and the viewed object
// it is causing precision loss issues.
// The view_transform has to be transformed relative to frame_id also-
bool Viz3D::setupCamera(const tf2::Transform& view_transform,
    const std::string& frame_id,
    const std::string& child_frame_id,
    const double aov_y,
    const double aov_x,
    const double near,
    const double far,
    const int fb_width, const int fb_height,
    glm::mat4& model_matrix,
    glm::mat4& view_matrix,
    glm::mat4& view_matrix_inverse,
    glm::mat4& projection_matrix,
    const bool vert_flip)
{
  const float pixel_aspect = static_cast<float>(fb_width) / static_cast<float>(fb_height);
  float aspect = pixel_aspect;
  if (aov_x != 0.0) {
    const float aov_aspect = aov_x / aov_y;
    aspect = aov_aspect;
  }

  const float sc_vert = vert_flip ? -1.0 : 1.0;

  projection_matrix = glm::perspective(
      static_cast<float>(sc_vert * glm::radians(aov_y)),
      sc_vert * aspect,
      static_cast<float>(near),
      static_cast<float>(far));

  model_matrix = glm::mat4(1.0f);
  try {
    geometry_msgs::msg::TransformStamped model_tf;
    model_tf = tf_buffer_->lookupTransform(frame_id, child_frame_id, tf2::TimePointZero);
    tf2::Stamped<tf2::Transform> model_stamped_transform;
    tf2::fromMsg(model_tf, model_stamped_transform);
    #if 0
    tf2::TimePoint time_out;
    // this is private, so doesn't work
    tf_buffer_->lookupTransformImpl(frame_id, child_frame_id,
        tf2::TimePointZero, transform, time_out);
    #endif
    // render_message_ << ", frames: " << frame_id_ << " -> " << child_frame_id << "\n";
    glm::dmat4 model_matrix_double;
    model_stamped_transform.getOpenGLMatrix(&model_matrix_double[0][0]);
    dmat4Todmat(model_matrix_double, model_matrix);
  } catch (tf2::TransformException& ex) {
    // render_message_ << "\n" << ex.what();
    return false;
  }

  glm::dmat4 view_matrix_double;
  view_transform.inverse().getOpenGLMatrix(&view_matrix_double[0][0]);
  dmat4Todmat(view_matrix_double, view_matrix);

  view_transform.getOpenGLMatrix(&view_matrix_double[0][0]);
  dmat4Todmat(view_matrix_double, view_matrix_inverse);
  if (false) {  // child_frame_id == "projector1") {
    // TEMP Debug
    // render_message_ << "\n======" << printMat(view_matrix_double) << "=====\n";
  }

  // mv = view_matrix * model_matrix;
  // mvp = projection_matrix * view_matrix * model_matrix;

  if (false) {
    // render_message_ << "\nmatrices:\n";
    // render_message_ << "model " << child_frame_id << ":\n" << printMat(model_matrix);
    // render_message_ << "view:\n" << printMat(view_matrix);
    // render_message_ << "projection:\n" << printMat(projection_matrix);
    // render_message_ << "mvp:\n" << printMat(mvp);
  }

  return true;
}

bool Viz3D::bindTexture(const std::string& name, const int active_ind)
{
  GLuint tex_id = 0;
  if ((name != "") && (textures_.count(name) > 0)) {
    tex_id = (GLuint)(intptr_t)textures_[name]->texture_id_;
    // render_message_ << "texture '" << name << "' " << tex_id;
  } else if (textures_.count("default") > 0) {
    tex_id = (GLuint)(intptr_t)textures_["default"]->texture_id_;
    // render_message_ << ", 'default' texture " << tex_id;
  } else {
    // TODO(lucasw) else stop rendering?
    // render_message_ << ", no texture to use";
  }
  // render_message_ << ", active texture ind " << active_ind << " -> "
  //    << GL_TEXTURE0 + active_ind << "\n";
  glActiveTexture(GL_TEXTURE0 + active_ind);
  // Bind texture- if it is null then the color is black
  glBindTexture(GL_TEXTURE_2D, tex_id);
  if (checkGLError(__FILE__, __LINE__)) {
    return false;
  }

  return true;
}

void Viz3D::render(const int fb_width, const int fb_height,
  const int display_pos_x, const int display_pos_y,
  const int display_size_x, const int display_size_y)
{
  // render_message_ << "\n############# rendering main\n";
  (void)display_pos_x;
  (void)display_pos_y;
  (void)display_size_x;
  (void)display_size_y;

  if (shapes_.size() == 0) {
    // render_message_ << "no shapes to render";
    return;
  }

  if (fb_width <= 0 || fb_height <= 0) {
    // render_message_ << "bad width height " << fb_width << " " << fb_height << "\n";
    return;
  }

  checkGLError(__FILE__, __LINE__);

  // render_message_ << "textures " << textures_.size();

  for (auto texture_pair : textures_) {
    texture_pair.second->updateTexture();
  }

    GLState gl_state;
    gl_state.backup();

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // TODO(lucasw) later enable, but it is nice to see geometry from back
    // add a combo for this
    glDisable(GL_CULL_FACE);

    // TODO(lucasw) later render to texture with a variable width and height
    // and then draw the texture to the screen with a textured quad fragment
    // shader, this allows rendering at a different resolution that the screen
    // which might be desirable if performance is suffering.
    const bool vert_flip = false;
    render2("default", transform_,
        fb_width, fb_height,
        aov_y_, aov_x_,
        near_, far_,
        vert_flip);

    gl_state.restore();
    checkGLError(__FILE__, __LINE__);
}

void Viz3D::renderShadows()
{
  // render_message_ << "\n############# rendering " << projectors_.size() << " shadows\n";
  if (projectors_.size() == 0) {
    // no need for shadows without directional light
    return;
  }

  GLState gl_state;
  gl_state.backup();
  for (auto projector_pair : projectors_) {
    auto projector = projector_pair.second;
    if (!projector->enable_) {
      continue;
    }
    // render_message_ << "\nrender depth shadows '" << projector->name_ << "'\n";

    glBindFramebuffer(GL_FRAMEBUFFER, projector->shadow_framebuffer_);
    glClear(GL_DEPTH_BUFFER_BIT);
    // only render backs of objects to shadows
    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);
    // TODO(lucasw) if render width/height change need to update rendered_texture

    try {
      geometry_msgs::msg::TransformStamped tf;
      tf = tf_buffer_->lookupTransform(frame_id_,
          projector->frame_id_, tf2::TimePointZero);
      tf2::fromMsg(tf, projector->stamped_transform_);
      #if 0
      tf2::TimePoint time_out;
      // this is private, so doesn't work
      tf_buffer_->lookupTransformImpl(frame_id_, child_frame_id,
          tf2::TimePointZero, transform, time_out);
      #endif
      // render_message_ << ", frames: " << frame_id_ << " -> "
      //     << projector->frame_id_ << "\n";
    } catch (tf2::TransformException& ex) {
      // render_message_ << "\n" << ex.what();
      continue;
    }

    const bool vert_flip = false;
    const bool use_projectors = false;
    render2("depth", projector->stamped_transform_,
        projector->shadow_width_,
        projector->shadow_height_,
        // TODO(lucasw) there is a factor of 2 error somewhere in here,
        // this 0.5 fudges it back to working
        projector->aov_y_,  // * 0.5,
        projector->aov_x_,  // * 0.5,
        projector->near_,
        projector->far_,
        vert_flip,
        use_projectors);

    // TODO(lucasw) copy the date from the texture out to a cv::Mat?
    checkGLError(__FILE__, __LINE__);
  }
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  gl_state.restore();
  // render_message_ << "\n########\n";
}

void Viz3D::renderCubeCameras()
{
  // render_message_ << "\n\n------ render cube cameras -----\n";
  // TODO(lucasw) make GLState back up in the constructor and restore in the destructor
  GLState gl_state;
  gl_state.backup();
  for (auto cube_camera_pair : cube_cameras_) {
    renderCubeCameraInner(cube_camera_pair.second);
  }
  gl_state.restore();
  // render_message_ << "\n---------- end render cube camera --------------\n";
}

bool Viz3D::renderCubeCameraInner(std::shared_ptr<CubeCamera> cube_camera)
{
  if (!cube_camera->enable_) {
    return false;
  }

  // 1st pass - render the scene to the cube map
  {
    try {
      geometry_msgs::msg::TransformStamped tf;
      tf = tf_buffer_->lookupTransform(frame_id_,
          cube_camera->frame_id_, tf2::TimePointZero);
      tf2::fromMsg(tf, cube_camera->stamped_transform_);
      #if 0
      tf2::TimePoint time_out;
      // this is private, so doesn't work
      tf_buffer_->lookupTransformImpl(frame_id_, child_frame_id,
          tf2::TimePointZero, transform, time_out);
      #endif
      // render_message_ << ", frames: " << frame_id_ << " -> "
      //     << cube_camera->frame_id_ << "\n";
    } catch (tf2::TransformException& ex) {
      // render_message_ << "\n" << ex.what();
      return false;
    }

    // TODO(lucasw) need a transform to rotate with for every face, face 5 ought
    // to be identity

    for (auto face : cube_camera->faces_) {
    // 5 NEGATIVE_Z is the forward face
    // Could safely skip rendering 5 most of the time, unless lens is spherical
    // auto face = cube_camera->faces_[4];
    // {
      // render_message_ << "cube camera face " << face->dir_ << "\n";
      glBindFramebuffer(GL_FRAMEBUFFER, face->frame_buffer_);
      glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
          face->dir_, cube_camera->cube_texture_id_, 0);
      glDrawBuffer(GL_COLOR_ATTACHMENT0);
      glClearColor(
          cube_camera->clear_color_.x,
          cube_camera->clear_color_.y,
          cube_camera->clear_color_.z,
          cube_camera->clear_color_.w);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      glDisable(GL_CULL_FACE);

      // TODO(lucasw) need to generate 5 transforms beyond
      // the one returned by lookup, to point in all six directions.
      const bool vert_flip = true;
      const int width = face->image_->width_;
      const int height = face->image_->height_;
      auto face_transform = cube_camera->stamped_transform_ * face->transform_;
      render2("default",
          face_transform,
          width,
          height,
          90.0,
          90.0,
          cube_camera->near_,
          cube_camera->far_,
          vert_flip);

      #if 0
      // texture copy
      // glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer_);
      // glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
      //                       face->dir_, cube_texture_id_, 0);
      glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT1,
                             GL_TEXTURE_2D, face->image_->texture_id_, 0);
      glDrawBuffer(GL_COLOR_ATTACHMENT1);
      glBlitFramebuffer(0, 0, width, height, 0, 0, width, height,
                        GL_COLOR_BUFFER_BIT, GL_NEAREST);
      #endif
    }
    // TODO(lucasw) copy the date from the texture out to a cv::Mat?
    checkGLError(__FILE__, __LINE__);
  }

  // 2nd pass - render the cube map onto the camera 'lens' geometry
  // use an orthographic projection

  // The active texture and uniform need to be bound together in order for this to work,
  // maybe the binds below also ought to be grouped above.
  if (shader_sets_.count("cube_map") < 1) {
    return false;
  }

  // TODO(lucasw) could a regular camera render be used here
  // instead of mostly redundant code?

  glBindFramebuffer(GL_FRAMEBUFFER, cube_camera->frame_buffer_);
  glClearColor(
        cube_camera->clear_color_.x,
        cube_camera->clear_color_.y,
        cube_camera->clear_color_.z,
        cube_camera->clear_color_.w);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  auto lens_shader = shader_sets_["cube_map"];
  int texture_unit = 0;
  // render_message_ << "cube_map "
  //     << lens_shader->uniform_locations_["cube_map"] << "\n";
  glActiveTexture(GL_TEXTURE0 + texture_unit);
  glBindTexture(GL_TEXTURE_CUBE_MAP, cube_camera->cube_texture_id_);
  glUniform1i(lens_shader->uniform_locations_["cube_map"], texture_unit);

  // TODO(lucasw) should give up if can't lock, just don't render now
  // std::lock_guard<std::mutex> lock(mutex_);

  // enabling blending really fouls up the graphics- maybe alpha
  // is getting mishandled somewhere.
  glDisable(GL_BLEND);
  // TODO(lucasw) later enable
  glDisable(GL_CULL_FACE);
  // TODO(lucasw) try disabling this
  glEnable(GL_DEPTH_TEST);
  // Want to draw to whole window
  glDisable(GL_SCISSOR_TEST);
#ifdef GL_POLYGON_MODE
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
#endif
  // TODO(lucasw) instead of outputing to stdout put the error in a gui text widget
  if (checkGLError(__FILE__, __LINE__))
    return false;

  // Our visible imgui space lies from draw_data->DisplayPos (top left) to draw_data->DisplayPos+data_data->DisplaySize (bottom right). DisplayMin is typically (0,0) for single viewport apps.
  // TODO(lucasw) need a cube_camera width and height for the full image
  const int fb_width = cube_camera->image_->width_;
  const int fb_height = cube_camera->image_->height_;
  glViewport(0, 0,
      (GLsizei)fb_width,
      (GLsizei)fb_height);

  if (checkGLError(__FILE__, __LINE__))
    return false;

  // render_message_ << ", shader '" << lens_shader->name_ << "', handle: " << lens_shader->shader_handle_;
  // render_message_ << "\n";

  // TODO(lucasw) store this lens name in the cube_camera
  if (shapes_.count(cube_camera->lens_name_) < 1) {
    // render_message_ << " no lens shape for cube camera "
    //     << cube_camera->lens_name_ << "\n";
    return false;
  }
  auto lens_shape = shapes_[cube_camera->lens_name_];
  if (!lens_shape) {
    // render_message_ << " null shape\n";
    return false;
  }

  // render_message_ << "lens shape: " << lens_shape->name_;

  glUseProgram(lens_shader->shader_handle_);
  {
    tf2::Transform transform;
    transform.setIdentity();
    glm::mat4 lens_model, view, view_inverse, projection;
    // TODO(lucasw) use ortho projection later
    bool vert_flip = false;
    if (!setupCamera(
        transform,
        // cube_camera->stamped_transform_,
        cube_camera->frame_id_,
        // frame_id_,
        lens_shape->frame_id_,
        cube_camera->aov_y_,
        cube_camera->aov_x_,
        cube_camera->near_,
        cube_camera->far_,
        fb_width, fb_height,
        lens_model, view, view_inverse, projection,
        vert_flip)) {
      return false;
    }
    // transfer data to shaders
    const auto transpose = GL_FALSE;
    glUniformMatrix4fv(lens_shader->uniform_locations_["model_matrix"],
        1, transpose, &lens_model[0][0]);
    glUniformMatrix4fv(lens_shader->uniform_locations_["view_matrix"],
        1, transpose, &view[0][0]);
    glUniformMatrix4fv(lens_shader->uniform_locations_["projection_matrix"],
        1, transpose, &projection[0][0]);

    // render_message_ << "lens:\n" << printMat(lens_model);
    printMat(lens_model);
    // render_message_ << "view:\n" << printMat(view);

#ifdef GL_SAMPLER_BINDING
    glBindSampler(0, 0);
    // We use combined texture/sampler state. Applications using GL 3.3 may set that otherwise.
#endif

    glBindVertexArray(lens_shape->vao_handle_);
    // render_message_ << "vao handle " << lens_shape->vao_handle_;

    ImVec4 clip_rect = ImVec4(0, 0, fb_width, fb_height);
    glScissor((int)clip_rect.x, (int)(fb_height - clip_rect.w),
        (int)(clip_rect.z - clip_rect.x), (int)(clip_rect.w - clip_rect.y));
    if (checkGLError(__FILE__, __LINE__))
      return false;

    // render_message_ << "\n";

    {
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, lens_shape->elements_handle_);
      const ImDrawIdx* idx_buffer_offset = 0;
      glDrawElements(GL_TRIANGLES, (GLsizei)lens_shape->indices_.Size,
          sizeof(ImDrawIdx) == 2 ? GL_UNSIGNED_SHORT : GL_UNSIGNED_INT, idx_buffer_offset);
      if (checkGLError(__FILE__, __LINE__))
        return false;
      // render_message_ << ", lens shape " << lens_shape->name_
      //    << " indices " << lens_shape->indices_.Size;
    }
  }
  return true;
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
      continue;
    }
    // render_message_ << "\nrender to texture " << camera->name_ << "\n";

    glBindFramebuffer(GL_FRAMEBUFFER, camera->frame_buffer_);
    glClearColor(
        camera->clear_color_.x,
        camera->clear_color_.y,
        camera->clear_color_.z,
        camera->clear_color_.w);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_CULL_FACE);
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
      // render_message_ << ", frames: " << frame_id_ << " -> "
      //    << camera->frame_id_ << "\n";
    } catch (tf2::TransformException& ex) {
      // render_message_ << "\n" << ex.what();
      continue;
    }

    const bool vert_flip = true;
    render2("default",
        camera->stamped_transform_,
        camera->image_->width_,
        camera->image_->height_,
        camera->aov_y_,
        camera->aov_x_,
        camera->near_,
        camera->far_,
        vert_flip);

    // TODO(lucasw) copy the date from the texture out to a cv::Mat?
    checkGLError(__FILE__, __LINE__);
  }
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  gl_state.restore();
}

void Viz3D::render2(
    const std::string& shaders_name,
    const tf2::Transform& transform,
    const int fb_width, const int fb_height,
    const float aov_y,
    const float aov_x,
    const float near,
    const float far,
    const bool vert_flip,
    const bool use_projectors)
{

  if (shader_sets_.size() == 0) {
    // render_message_ << "no shaders\n";
    return;
  }
  if (shader_sets_.count(shaders_name) < 1) {
    // render_message_ << "no shader '" << shaders_name << "'\n";
    return;
  }
  // TODO(lucasw) for now just use the last shader set
  auto shaders = shader_sets_[shaders_name];

  // TODO(lucasw) should give up if can't lock, just don't render now
  std::lock_guard<std::mutex> lock(mutex_);

    // Setup render state: alpha-blending enabled, no face culling, no depth testing, scissor enabled, polygon fill
    glEnable(GL_BLEND);
    glBlendEquation(GL_FUNC_ADD);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
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

  // render_message_ << ", shader '" << shaders->name_ << "', handle: " << shaders->shader_handle_;
  // for (auto shaders_pair : shader_sets_) {
  //  shaders = shaders_pair.second;
  //}
  // render_message_ << "\n";

  for (auto shape_pair : shapes_) {
    auto shape = shape_pair.second;
    if (!shape) {
      // render_message_ << " null shape\n";
      continue;
    }

    // render_message_ << "shape: " << shape->name_;
    if (!shape->enable_) {
      // render_message_ << " disabled\n";
      continue;
    }

    // TODO(lucasw) later a shape can use certain shaders or just default
    glUseProgram(shaders->shader_handle_);
    {
      glm::mat4 model, view, view_inverse, projection;
      if (!setupCamera(transform,
          frame_id_,
          shape->frame_id_,
          aov_y,
          aov_x,
          near,
          far,
          fb_width, fb_height,
          model, view, view_inverse, projection,
          vert_flip))
        continue;
      // TODO(lucasw) use double in the future?
      // glUniformMatrix4dv(shape->attrib_location.proj_mtx_, 1, GL_FALSE, &mvp[0][0]);

      {
        tf2::Vector3 translation = transform.getOrigin();
        glm::vec3 eye_pos = glm::vec3(translation.x(), translation.y(), translation.z());
        // render_message_ << "eye pos: " << printVec(eye_pos) << "\n";
        glUniform3fv(shaders->uniform_locations_["eye_pos"],
            1, &eye_pos[0]);
      }
      // transfer data to shaders
      const auto transpose = GL_FALSE;
      glUniformMatrix4fv(shaders->uniform_locations_["model_matrix"],
          1, transpose, &model[0][0]);
      glUniformMatrix4fv(shaders->uniform_locations_["view_matrix"],
          1, transpose, &view[0][0]);
      glUniformMatrix4fv(shaders->uniform_locations_["projection_matrix"],
          1, transpose, &projection[0][0]);
    }

    // if doing shadows then don't need to bind any textures,
    // for now use use_projectors = false as meaning doing depth only render
    if (use_projectors) {
      glUniform1i(shaders->uniform_locations_["Texture"],
          texture_unit_["Texture"]);
      glUniform1i(shaders->uniform_locations_["shininess_texture"],
          texture_unit_["shininess_texture"]);

      if (checkGLError(__FILE__, __LINE__))
        return;
    }

    // TODO(lucasw) add imgui use texture projection checkbox
    // make a vector of projectors to use here, no more than MAX_PROJECTORS
    std::vector<std::shared_ptr<Projector> > projectors;
    if (use_projectors) {
      setupProjectorsWithShape(frame_id_, shape->frame_id_, projectors);
    }

#ifdef GL_SAMPLER_BINDING
    glBindSampler(0, 0);
    // We use combined texture/sampler state. Applications using GL 3.3 may set that otherwise.
#endif

    glBindVertexArray(shape->vao_handle_);
    // render_message_ << "vao handle " << shape->vao_handle_;

    ImVec4 clip_rect = ImVec4(0, 0, fb_width, fb_height);
    glScissor((int)clip_rect.x, (int)(fb_height - clip_rect.w),
        (int)(clip_rect.z - clip_rect.x), (int)(clip_rect.w - clip_rect.y));
    if (checkGLError(__FILE__, __LINE__))
      return;

    // render_message_ << "\n";

    // TODO(lucasw) why not group these with uniform setting of textures above?
    // if doing shadows then don't need to bind any textures,
    // for now use use_projectors = false as meaning doing depth only render
    if (use_projectors) {
      // Bind texture- if it is null then the color is black
      bindTexture(shape->texture_, texture_unit_["Texture"]);
      // render_message_ << "shininess ";
      bindTexture(shape->shininess_texture_, texture_unit_["shininess_texture"]);
    }

    if (use_projectors) {
      const int num_projectors = projectors.size();

      float max_range[MAX_PROJECTORS];
      float constant_attenuation[MAX_PROJECTORS];
      float linear_attenuation[MAX_PROJECTORS];
      float quadratic_attenuation[MAX_PROJECTORS];
      for (size_t i = 0; i < projectors.size(); ++i) {
        max_range[i] = projectors[i]->max_range_;
        constant_attenuation[i] = projectors[i]->constant_attenuation_;
        linear_attenuation[i] = projectors[i]->linear_attenuation_;
        quadratic_attenuation[i] = projectors[i]->quadratic_attenuation_;

        // TODO(lucasw) later the scale could be a brightness setting
        // TODO(lucasw) index currently hardcoded, later make more flexible
        // render_message_ << "bind projector texture ";
        // glUniform with ProjectedTexture needs to have been set already
        bindTexture(projectors[i]->texture_name_, projector_texture_unit_[i]);

        // shadows
        {
          // glActiveTexture(GL_TEXTURE1 + num_projectors + i);
          int active_texture_ind = GL_TEXTURE0 + shadow_texture_unit_[i];
          // render_message_ << "bind projector shadow "
          //     << projectors[i]->shadow_depth_texture_ << ", active texture ind "
          //     << active_texture_ind << "\n";
          glActiveTexture(active_texture_ind);
          glBindTexture(GL_TEXTURE_2D, projectors[i]->shadow_depth_texture_);
        }

        if (checkGLError(__FILE__, __LINE__))
          return;
      }

      glUniform1iv(shaders->uniform_locations_["num_projectors"], 1, &num_projectors);

      // TODO(lucasw) later only change uniforms if they change
      float near = near_;
      float far = far_;
      glUniform1fv(shaders->uniform_locations_["near_clip"],
         1, &near);
      glUniform1fv(shaders->uniform_locations_["far_clip"],
         1, &far);
      glUniform3fv(shaders->uniform_locations_["ambient"],
         1, &ambient_[0]);
      glUniform1fv(shaders->uniform_locations_["projector_max_range"],
         MAX_PROJECTORS, &max_range[0]);
      glUniform1fv(shaders->uniform_locations_["projector_constant_attenuation"],
         MAX_PROJECTORS, &constant_attenuation[0]);
      glUniform1fv(shaders->uniform_locations_["projector_linear_attenuation"],
         MAX_PROJECTORS, &linear_attenuation[0]);
      glUniform1fv(shaders->uniform_locations_["projector_quadratic_attenuation"],
         MAX_PROJECTORS, &quadratic_attenuation[0]);
    }  // use_projectors

    if ((shape->draw_mode_ >= 0) && (shape->draw_mode_ < Shape::draw_modes_.size())) {
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, shape->elements_handle_);
      const ImDrawIdx* idx_buffer_offset = 0;
      glDrawElements(Shape::draw_modes_[shape->draw_mode_],
          (GLsizei)shape->indices_.Size,
          sizeof(ImDrawIdx) == 2 ? GL_UNSIGNED_SHORT : GL_UNSIGNED_INT, idx_buffer_offset);
      // std::cout << cmd_i << " " << tex_id << " " << idx_buffer_offset << "\n";
      if (checkGLError(__FILE__, __LINE__))
        return;
      // render_message_ << ", shape " << shape->name_ << " indices " << shape->indices_.Size;
      // idx_buffer_offset += pcmd->ElemCount;
      // glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
    // render_message_ << "\n------------------------\n";
    // glBindVertexArray(0);
  }  // loop through shapes to draw

  return;
}

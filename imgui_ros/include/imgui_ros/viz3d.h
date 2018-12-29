/*
 * Copyright (c) 2017 Lucas Walter
 * June 2017
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
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef IMGUI_ROS_VIZ3D_H
#define IMGUI_ROS_VIZ3D_H

#include <glm/glm.hpp>
#include <imgui.h>
#include <imgui_ros/camera.h>
#include <imgui_ros/imgui_impl_opengl3.h>
#include <imgui_ros/image.h>
#include <imgui_ros/msg/textured_shape.hpp>
#include <imgui_ros/projector.h>
#include <imgui_ros/shaders.h>
#include <imgui_ros/surface.h>
#include <imgui_ros/srv/add_camera.hpp>
#include <imgui_ros/srv/add_projector.hpp>
#include <imgui_ros/srv/add_shaders.hpp>
#include <imgui_ros/srv/add_shape.hpp>
#include <imgui_ros/srv/add_texture.hpp>
// #include <imgui_ros/window.h>
#include <mutex>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
// About OpenGL function loaders: modern OpenGL doesn't have a standard header
// file and requires individual function pointers to be loaded manually. Helper
// libraries are often used for this purpose! Here we are supporting a few
// common ones: gl3w, glew, glad. You may use another loader/header of your
// choice (glext, glLoadGen, etc.), or chose to manually implement your own.
#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
#include <GL/gl3w.h> // Initialize with gl3wInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
#include <GL/glew.h> // Initialize with glewInit()
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
#include <glad/glad.h> // Initialize with gladLoadGL()
#else
#include IMGUI_IMPL_OPENGL_LOADER_CUSTOM
#endif
#pragma GCC diagnostic pop

// TODO(lucasw) need to support covering the entire background,
// and being within a widget, possibly with subclassing.
// For now just do entire background.
struct Viz3D {
  Viz3D(const std::string name,
    const std::string topic,
    std::shared_ptr<ImGuiImplOpenGL3> renderer,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<rclcpp::Node> node
    );
  ~Viz3D();
  void render(const int fb_width, const int fb_height,
      const int display_pos_x, const int display_pos_y,
      const int display_size_x, const int display_size_y);
  void renderShadows();
  void renderToTexture();
  void render2(
      const std::string& shaders_name,
      const tf2::Transform& transform,
      const int fb_width, const int fb_height,
      const float aov_y, const float aov_x,
      const bool vert_flip = false,
      const bool use_projectors = true);

  std::stringstream render_message_;

  void draw();
  //    const int pos_x, const int pos_y,
  //    const int size_x, const int size_y);
protected:
  // TODO(lucasw) later this will be a matrix
  // glm::vec3 translation_ = glm::vec3(0, 0, 0);
  tf2::Transform transform_;
  tf2::Vector3 velocity_ = tf2::Vector3(0.0, 0.0, 0.0);

  // TODO(lucasw) should this go somewhere else?
  glm::vec3 ambient_;

  double move_scale_ = 0.005;
  double rotate_scale_ = 300.0;
  double pitch_ = 0.0;
  double yaw_ = 0.0;

  bool dragging_view_ = false;
  ImVec2 drag_point_;

  double aov_y_ = 45.0f;
  double aov_x_ = 0.0f;
  float near_ = 0.01f;
  float far_ = 100.0f;
  // TODO(lucasw) should be setupProjection or setupModelViewProjection
  // though later will do the matrix multiplication inside shader?
  bool setupCamera(const tf2::Transform& view_transform, const std::string child_frame_id,
      const double aov_y,
      const double aov_x,
      const int fb_width, const int fb_height,
      glm::mat4& model_matrix,
      glm::mat4& view_matrix,
      glm::mat4& view_matrix_inverse,
      glm::mat4& projection_matrix,
      const bool vert_flip = false);

  // this is done for every shape - probably can refactor to
  // get only the relevant parts that change per shape
  bool setupProjectorsWithShape(
      const std::string& main_frame_id, const std::string& shape_frame_id,
      std::vector<std::shared_ptr<Projector> >& projectors);

  std::string glsl_version_string_ = "";
#if 0
  // temp texture test
  GLuint texture_id_ = 0;
  cv::Mat test_;
#endif
#if 0
  // TODO(lucasw) or NULL or -1?
  GLuint texture_id_ = 0;
  size_t width_ = 0;
  size_t height_ = 0;

  static const GLfloat* getVertexBuffer() {
    static const GLfloat g_vertex_buffer_data_[] = {
      -1.0f, -1.0f, 0.0f,
      1.0f, -1.0f, 0.0f,
      0.0f,  1.0f, 0.0f,
    };
    return &g_vertex_buffer_data_[0];
  }
  GLuint vertex_buffer_;
#endif

  rclcpp::Service<imgui_ros::srv::AddCamera>::SharedPtr add_camera_;
  void addCamera(const std::shared_ptr<imgui_ros::srv::AddCamera::Request> req,
                  std::shared_ptr<imgui_ros::srv::AddCamera::Response> res);
  std::map<std::string, std::shared_ptr<Camera> > cameras_;

  // Can exceed this number of projectors but this is the number
  // than can simultaneously be projectored on any surface.
  // This constant has to be matched in shaders
  static const int MAX_PROJECTORS = 4;
  rclcpp::Service<imgui_ros::srv::AddProjector>::SharedPtr add_projector_;
  void addProjector(const std::shared_ptr<imgui_ros::srv::AddProjector::Request> req,
                  std::shared_ptr<imgui_ros::srv::AddProjector::Response> res);
  std::map<std::string, std::shared_ptr<Projector> > projectors_;

  rclcpp::Service<imgui_ros::srv::AddShaders>::SharedPtr add_shaders_;
  void addShaders(const std::shared_ptr<imgui_ros::srv::AddShaders::Request> req,
                  std::shared_ptr<imgui_ros::srv::AddShaders::Response> res);
  std::map<std::string, std::shared_ptr<ShaderSet> > shader_sets_;

  bool updateShaderShapes(std::shared_ptr<ShaderSet> shaders, std::shared_ptr<Shape> shape);

  rclcpp::Service<imgui_ros::srv::AddTexture>::SharedPtr add_texture_;
  void addTexture(const std::shared_ptr<imgui_ros::srv::AddTexture::Request> req,
                  std::shared_ptr<imgui_ros::srv::AddTexture::Response> res);

  rclcpp::Service<imgui_ros::srv::AddShape>::SharedPtr add_shape_;
  void addShape(const std::shared_ptr<imgui_ros::srv::AddShape::Request> req,
                std::shared_ptr<imgui_ros::srv::AddShape::Response> res);
  void texturedShapeCallback(const imgui_ros::msg::TexturedShape::SharedPtr msg);
  rclcpp::Subscription<imgui_ros::msg::TexturedShape>::SharedPtr textured_shape_sub_;
  bool addShape2(const imgui_ros::msg::TexturedShape::SharedPtr msg, std::string& message);
  // TODO(lucasw) it would be nice if the received TexturedShape
  // could be passed into opengl directly, which it probably could be made
  // to do, but for now interpret it on reception into local class.
  std::map<std::string, std::shared_ptr<Shape> > shapes_;

  std::string name_;
  std::string frame_id_ = "map";
  // TODO(lucasw) don't need this now
  std::weak_ptr<ImGuiImplOpenGL3> renderer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::weak_ptr<rclcpp::Node> node_;

  std::map<std::string, std::shared_ptr<RosImage> > textures_;

  bool initialized_ = false;

  std::mutex mutex_;
};

#endif  // IMGUI_ROS_VIZ3D_H

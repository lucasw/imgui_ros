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
#include <imgui_ros/cube_camera.h>
#include <imgui_ros/imgui_impl_opengl3.h>
#include <imgui_ros/image.h>
#include <imgui_ros/projector.h>
#include <imgui_ros/shaders.h>
#include <imgui_ros/surface.h>
#include <imgui_ros/window.h>
#include <imgui_ros_msgs/AddCamera.h>
#include <imgui_ros_msgs/AddCubeCamera.h>
#include <imgui_ros_msgs/AddProjector.h>
#include <imgui_ros_msgs/AddShaders.h>
#include <imgui_ros_msgs/AddShape.h>
#include <imgui_ros_msgs/AddTexture.h>
#include <imgui_ros_msgs/TexturedShape.h>
// #include <imgui_ros/window.h>
#include <map>
#include <mutex>
#include <opencv2/core.hpp>
#include <ros/ros.h>
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

namespace imgui_ros
{

// TODO(lucasw) need to support covering the entire background,
// and being within a widget, possibly with subclassing.
// For now just do entire background.
// TODO(lucasw) making this subclass Window add some things it doesn't need
// maybe should make a root class of Window to avoid that.
struct Viz3D : public Window {
  Viz3D(const std::string name,
    const std::string topic,
    std::shared_ptr<ImGuiImplOpenGL3> renderer,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    ros::NodeHandle* nh,
    std::shared_ptr<ImageTransfer> image_transfer
    );
  ~Viz3D();

  void renderMain(const int fb_width, const int fb_height,
      const int display_pos_x, const int display_pos_y,
      const int display_size_x, const int display_size_y);
  void renderShadows();
  void renderCubeCameras();
  bool renderCubeCamera(std::shared_ptr<CubeCamera> cube_camera);
  // TODO(lucasw) rename to renderCameras
  void renderCamerasToTexture();
  void render2(
      const std::string& shaders_name,
      const tf2::Transform& transform,
      const int fb_width, const int fb_height,
      const float aov_y, const float aov_x,
      const float near, const float far,
      const bool vert_flip = false,
      // no texture gets rendered with default shader if this is false
      const bool use_projectors = true);

  std::string debug_win_ = "main_render";
  std::map<std::string, std::vector<std::string>> render_messages_;
  ImVec4 clear_color_ = ImVec4(0.4, 0.3, 0.5, 1.0);
  // TODO(lucasw) should this go somewhere else?
  glm::vec3 ambient_ = glm::vec3(0.3, 0.3, 0.3);

  virtual void update(const ros::Time& stamp, const std::string dropped_file);
  virtual void draw(const int outer_window_width, const int outer_window_height);
  //    const int pos_x, const int pos_y,
  //    const int size_x, const int size_y);

  virtual void addTF(tf2_msgs::TFMessage& tfm, const ros::Time& now);

  // Can exceed this number of projectors but this is the number
  // than can simultaneously be projectored on any surface.
  // This constant has to be matched in shaders
  static const int MAX_PROJECTORS = 6;

  void addOrReplaceShape(const std::string& name, const std::shared_ptr<Shape> shape);
  // void removeShape(const std::shared_ptr<Shape> shape);
  void removeShape(const std::string& name, std::string& message);
  void removeShape(const std::string& name) {
    std::string message;
    removeShape(name, message);
  }

protected:
  virtual void drawMain();

  // TODO(lucasw) maybe this should be a std::map of std::vectors
  std::map<std::string, int > texture_unit_;
  int projector_texture_unit_[MAX_PROJECTORS];
  int shadow_texture_unit_[MAX_PROJECTORS];
  bool bindTexture(const std::string& name, const int tex_ind);

  ///////// main window camera movement //////////////////////////
  tf2::Vector3 velocity_ = tf2::Vector3(0.0, 0.0, 0.0);
  double move_scale_ = 0.05;
  double rotate_scale_ = 300.0;
  double pitch_ = 0.0;
  double yaw_ = 0.0;

  bool dragging_view_ = false;
  ImVec2 drag_point_;

  tf2::Transform transform_;
  // the parent and child frames of the above transform
  std::string frame_id_ = "map";
  std::string main_window_frame_id_ = "viz3d_main_window_camera";
  ////////////////////////////////////////////////////////////////

  double aov_y_ = 45.0f;
  double aov_x_ = 0.0f;
  // too low and get glitches with shadows, but that can be solved with better shadow
  // techniques- like only rendering back faces
  double near_ = 0.1f;
  double far_ = 100.0f;
  // TODO(lucasw) should be setupProjection or setupModelViewProjection
  // though later will do the matrix multiplication inside shader?
  bool setupCamera(const tf2::Transform& view_transform,
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
      const bool vert_flip = false);

  // this is done for every shape - probably can refactor to
  // get only the relevant parts that change per shape
  bool setupProjectorsWithShape(
      const std::string& main_frame_id, const std::string& shape_frame_id,
      std::vector<std::shared_ptr<Projector> >& projectors);

  std::string glsl_version_string_ = "";

  ros::ServiceServer add_camera_;
  bool addCamera(imgui_ros_msgs::AddCamera::Request& req,
                 imgui_ros_msgs::AddCamera::Response& res);
  std::map<std::string, std::shared_ptr<Camera> > cameras_;

  ros::ServiceServer add_cube_camera_;
  bool addCubeCamera(imgui_ros_msgs::AddCubeCamera::Request& req,
                     imgui_ros_msgs::AddCubeCamera::Response& res);
  std::map<std::string, std::shared_ptr<CubeCamera> > cube_cameras_;

  ros::ServiceServer add_projector_;
  bool addProjector(imgui_ros_msgs::AddProjector::Request& req,
                    imgui_ros_msgs::AddProjector::Response& res);
  std::map<std::string, std::shared_ptr<Projector> > projectors_;

  ros::ServiceServer add_shaders_;
  bool addShaders(imgui_ros_msgs::AddShaders::Request& req,
                  imgui_ros_msgs::AddShaders::Response& res);
  std::map<std::string, std::shared_ptr<ShaderSet> > shader_sets_;

  bool updateShaderShapes(std::shared_ptr<ShaderSet> shaders, std::shared_ptr<Shape> shape);

  ros::ServiceServer add_texture_;
  bool addTexture(imgui_ros_msgs::AddTexture::Request& req,
                  imgui_ros_msgs::AddTexture::Response& res);

  ros::ServiceServer add_shape_;
  bool addShape(imgui_ros_msgs::AddShape::Request& req,
                imgui_ros_msgs::AddShape::Response& res);
  void texturedShapeCallback(const imgui_ros_msgs::TexturedShape::ConstPtr& msg);
  ros::Subscriber textured_shape_sub_;
  bool addShape2(const imgui_ros_msgs::TexturedShape::ConstPtr& msg, std::string& message);
  // TODO(lucasw) it would be nice if the received TexturedShape
  // could be passed into opengl directly, which it probably could be made
  // to do, but for now interpret it on reception into local class.
  std::map<std::string, std::shared_ptr<Shape> > shapes_;

  std::string name_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  // std::weak_ptr<ros::NodeHandle> nh_;
  ros::NodeHandle* nh_ = nullptr;

  std::shared_ptr<ImageTransfer> image_transfer_;
  std::map<std::string, std::shared_ptr<RosImage> > textures_;

  // std::shared_ptr<imgui_ros::TfBroadcaster> tf_broadcaster_;

  bool multisample_ = false;
  int num_samples_ = 2;
  bool initialized_ = false;

  double line_width_ = 3.0;
  double point_size_ = 3.0;

  std::mutex mutex_;
};

}  // namespace imgui_ros
#endif  // IMGUI_ROS_VIZ3D_H

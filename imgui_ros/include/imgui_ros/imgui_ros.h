// dear imgui: standalone example application for SDL2 + OpenGL
// If you are new to dear imgui, see examples/README.txt and documentation at
// the top of imgui.cpp. (SDL is a cross-platform general purpose library for
// handling windows, inputs, OpenGL/Vulkan graphics context creation, etc.)
// (GL3W is a helper library to access OpenGL functions since there is no
// standard header to access modern OpenGL functions easily. Alternatives are
// GLEW, Glad, etc.)

#include "imgui.h"
// #include "imgui_impl_opengl3.h"
// #include "imgui_impl_sdl.h"
#include <imgui_ros/Image.h>
#include <map>
#include <mutex>
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <SDL.h>

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

struct GlImage {
  GlImage(const std::string name);
  ~GlImage();
  virtual bool updateTexture() = 0;
  virtual void draw() = 0;
protected:
  // TODO(lucasw) or NULL or -1?
  GLuint texture_id_ = 0;
  bool dirty_ = true;
  std::string name_ = "";
  std::mutex mutex_;
};

struct RosImage : public GlImage {
  RosImage(const std::string name, const std::string topic,
           ros::NodeHandle& nh);

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  // TODO(lucasw) factor this into a generic opengl function to put in parent class
  // if the image changes need to call this
  virtual bool updateTexture();

  // TODO(lucasw) factor out common code
  virtual void draw();

private:
  ros::Subscriber sub_;
  sensor_msgs::ImageConstPtr image_;
};  // RosImage

struct CvImage : public GlImage {
  CvImage(const std::string name);
  // TODO(lucasw) instead of cv::Mat use a sensor_msgs Image pointer,
  // an convert straight from that format rather than converting to cv.
  // Or just have two implementations of Image here, the cv::Mat
  // would be good to keep as an example.
  cv::Mat image_;

  // if the image changes need to call this
  virtual bool updateTexture();
  virtual void draw();
};

///////////////////////////////////////////////////////////////////////////////
namespace imgui_ros {
class ImguiRos {
public:
  ImguiRos();
  ~ImguiRos();

private:
  bool init();
  bool addImage(imgui_ros::Image::Request& req,
                imgui_ros::Image::Response& res);
  void update(const ros::TimerEvent& e);

  ros::NodeHandle nh_;
  SDL_Window *window;
  ImGuiIO io;
  SDL_GLContext gl_context;
  bool show_demo_window = true;
  bool show_another_window = false;
  ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

  std::map<std::string, std::shared_ptr<GlImage> > images_;

  // TODO(lucasw) still need to update even if ros time is paused
  ros::Timer update_timer_;

  ros::ServiceServer add_image_;
};

}  // namespace imgui_ros

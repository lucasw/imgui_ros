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
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <chrono>
// #include <geometry_msgs/pose.hpp>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <imgui.h>
#include <imgui_impl_sdl.h>
#pragma GCC diagnostic pop
#if 0
#include <imgui_ros/image.h>
#endif
#include <imgui_ros/imgui_impl_opengl3.h>
#include <imgui_ros/imgui_ros.h>
#if 0
#include <imgui_ros/graph.h>
#include <imgui_ros/param.h>
#include <imgui_ros/point_cloud.h>
#include <imgui_ros/pub.h>
#include <imgui_ros/sub.h>
#include <imgui_ros/tf.h>
#include <imgui_ros/viz2d.h>
#endif
// #include <opencv2/highgui.hpp>
#if 0
#include <std_msgs/Bool.h>
#include <std_msgs/float32.h>
#include <std_msgs/float64.h>
#include <std_msgs/int16.h>
#include <std_msgs/int32.h>
#include <std_msgs/int64.hpp>
#include <std_msgs/int8.hpp>
#include <std_msgs/string.hpp>
#include <std_msgs/u_int16.hpp>
#include <std_msgs/u_int32.hpp>
#include <std_msgs/u_int64.hpp>
#include <std_msgs/u_int8.hpp>
#endif
#include <SDL2/SDL_keycode.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// TODO(lucasw) this may not shut down properly
// #define RUN_IMAGE_TRANSFER_SEPARATE_THREAD

namespace imgui_ros
{

ImguiRos::ImguiRos()  // :
  // clock_(std::make_shared<ros::Clock>(RCL_SYSTEM_TIME)),
  // buffer_(clock_)
{
  postInit();
}

void ImguiRos::postInit()
{
  ROS_INFO_STREAM("imgui ros init");
#if 0 
  internal_pub_sub::Node::postInit(core);
  image_transfer_ = std::make_shared<ImageTransfer>();
  image_transfer_->init("image_transfer", get_namespace());
  image_transfer_->postInit(core);
  #ifdef RUN_IMAGE_TRANSFER_SEPARATE_THREAD
  ros_io_thread_ = std::thread(
      std::bind(&ImguiRos::runNodeSingleThreaded, this, image_transfer_));
  #endif

  tf_pub_ = create_publisher<tf2_msgs::TFMessage>("/tf");
  clock_ = std::make_shared<ros::Clock>(RCL_SYSTEM_TIME);
  #if 1
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
  #else
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
  #endif
  #if 0
  // this puts tf listening in this same thread, will that hurt performance?
  const bool spin_thread = false;
  // shared_from_this won't unload properly
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_,
      shared_from_this(), spin_thread);
  #else

  #if 1
  // TODO(lucasw) can't seem to kill tfl_ once it starts,
  // maybe should hold onto tf_node?  Or don't depend on spinning inside TransformListener,
  // make own executor out here?
  // auto tf_node = ros::Node::make_shared("transform_listener_impl", get_namespace());
  tf_node_ = ros::Node::make_shared("transform_listener_impl", get_namespace());
  // auto tf2_buffer = tf2_ros::Buffer(clock_);
  tf_buffer_->setUsingDedicatedThread(true);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_,  // buffer_,
      tf_node_, false);
  #else
  // this doesn't shut down properly either
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  #endif
  #endif

  // TODO(lucasw) check if width and height are > minimum value
  try {
    get_parameter_or("name", name_, name_);
    get_parameter_or("width", width_, width_);
    get_parameter_or("height", height_, height_);
  } catch (ros::ParameterTypeException& ex) {
    ROS_ERROR_STREAM(ex.what());
  }

  // parameters_client_ = std::make_shared<ros::AsyncParametersClient>(this);

  // building this causes the node to crash only in release mode
  add_tf_ = create_service<AddTf>("add_tf",
      std::bind(&ImguiRos::addTf, this, std::placeholders::_1, std::placeholders::_2));

#endif
  add_window_ = nh_.advertiseService("add_window",
      &ImguiRos::addWindow, this);
  update_timer_ = nh_.createTimer(ros::Duration(0.033), &ImguiRos::update, this);
}

ImguiRos::~ImguiRos()
{
  ROS_INFO_STREAM("shutting down imgui_ros");
#if 0
  #ifdef RUN_IMAGE_TRANSFER_SEPARATE_THREAD
  ros_io_thread_.join();
  #endif
  image_transfer_ = nullptr;
  viz3d = nullptr;
  windows_.clear();
  // tf_buffer_ = nullptr;
  // this locks up
  // tfl_ = nullptr;
#endif
  imgui_impl_opengl3_->Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();

  SDL_GL_DeleteContext(gl_context);
  SDL_DestroyWindow(sdl_window_);
  SDL_Quit();
  ROS_INFO_STREAM("finished shutting down imgui_ros");
}

void ImguiRos::glInit()
{
  thread_id_ = std::this_thread::get_id();
  ROS_INFO_STREAM("imgui thread init 0x" << std::hex << thread_id_ << std::dec);
  ROS_INFO("opengl init %d", init_);

  // Setup SDL
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
    // RCLCPP_ERROR(this->get_logger(), "Error: %s", SDL_GetError());
    // TODO(lucasw) throw
    throw std::runtime_error("SDL Error: " + std::string(SDL_GetError()));
  }

  // Decide GL+GLSL versions
#if __APPLE__
  // GL 3.2 Core + GLSL 150
  const char *glsl_version = "#version 150";
  SDL_GL_SetAttribute(
      SDL_GL_CONTEXT_FLAGS,
      SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG); // Always required on Mac
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
#else
  // GL 3.0 + GLSL 130
  const char *glsl_version = "#version 130";
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
#endif

  // Create window with graphics context
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
  SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);

#if 1
  // TODO(lucasw) optional with combobox
  SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
  SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);
  // SDL_GL_SetAttribute(SDL_GL_ACCELERATED_VISUAL, 1);
  // glEnable(GL_MULTISAMPLE);
  // SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
#endif

  SDL_DisplayMode current;
  SDL_GetCurrentDisplayMode(0, &current);
  // TODO(lucasw) window.reset()
  sdl_window_ = SDL_CreateWindow(
      name_.c_str(), SDL_WINDOWPOS_CENTERED,
      SDL_WINDOWPOS_CENTERED, width_, height_,
      SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE); // | SDL_WINDOW_BORDERLESS);
  gl_context = SDL_GL_CreateContext(sdl_window_);
  SDL_GL_SetSwapInterval(1); // Enable vsync

  // Initialize OpenGL loader
#if defined(IMGUI_IMPL_OPENGL_LOADER_GL3W)
  bool err = gl3wInit() != 0;
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLEW)
  bool err = glewInit() != GLEW_OK;
#elif defined(IMGUI_IMPL_OPENGL_LOADER_GLAD)
  bool err = gladLoadGL() == 0;
#endif
  if (err) {
    throw std::runtime_error("Failed to initialize OpenGL loader!");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  // Setup Dear ImGui binding
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard
  // Controls

  ImGui_ImplSDL2_InitForOpenGL(sdl_window_, gl_context);
  imgui_impl_opengl3_ = std::make_shared<ImGuiImplOpenGL3>();
  imgui_impl_opengl3_->Init(glsl_version);

  // Setup style
  ImGui::StyleColorsDark();
  // ImGui::StyleColorsClassic();

  // Load Fonts
  // - If no fonts are loaded, dear imgui will use the default font. You can
  // also load multiple fonts and use ImGui::PushFont()/PopFont() to select
  // them.
  // - AddFontFromFileTTF() will return the ImFont* so you can store it if you
  // need to select the font among multiple.
  // - If the file cannot be loaded, the function will return NULL. Please
  // handle those errors in your application (e.g. use an assertion, or display
  // an error and quit).
  // - The fonts will be rasterized at a given size (w/ oversampling) and stored
  // into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which
  // ImGui_ImplXXXX_NewFrame below will call.
  // - Read 'misc/fonts/README.txt' for more instructions and details.
  // - Remember that in C/C++ if you want to include a backslash \ in a string
  // literal you need to write a double backslash \\ !
  // io.Fonts->AddFontDefault();
  // io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
  // io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
  // io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
  // io.Fonts->AddFontFromFileTTF("../../misc/fonts/ProggyTiny.ttf", 10.0f);
  // ImFont* font =
  // io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f,
  // NULL, io.Fonts->GetGlyphRangesJapanese()); IM_ASSERT(font != NULL);

#if 0
  const std::string viz3d_name = "main render window";
  viz3d = std::make_shared<Viz3D>(viz3d_name, "shapes",
      imgui_impl_opengl3_,
      tf_buffer_,
      shared_from_this(),
      image_transfer_);

  windows_[viz3d_name] = viz3d;

  try {
    get_parameter_or("red", viz3d->clear_color_.x, viz3d->clear_color_.x);
    get_parameter_or("green", viz3d->clear_color_.y, viz3d->clear_color_.y);
    get_parameter_or("blue", viz3d->clear_color_.z, viz3d->clear_color_.z);
    get_parameter_or("alpha", viz3d->clear_color_.w, viz3d->clear_color_.w);
  } catch (ros::ParameterTypeException& ex) {
    ROS_ERROR_STREAM(ex.what());
  }
#endif
  init_ = true;  // viz3d->initialized_;
}

#if 0
void ImguiRos::addTf(const std::shared_ptr<imgui_ros::AddTf::Request> req,
           std::shared_ptr<imgui_ros::AddTf::Response> res)
{
  RCLCPP_DEBUG(get_logger(), "new tf pub %s", req->tf.name);
  std::shared_ptr<Pub> pub = std::make_shared<TfBroadcaster>(
      req->tf,
      tf_buffer_,
      shared_from_this());

  std::shared_ptr<Window> window;
  if (windows_.count(req->tf.window) > 0) {
    window = windows_[req->tf.window];
  } else {
    window = std::make_shared<Window>(req->tf.window);
    windows_[req->tf.window] = window;
  }
  window->add(pub, req->tf.tab_name);
  res->success = true;
  res->message += "added tf pub " + req->tf.name + " to " + req->tf.window;
}

#endif

bool ImguiRos::addWindow(imgui_ros_msgs::AddWindow::Request& req,
    imgui_ros_msgs::AddWindow::Response& res)
{
  // std::cout << "adding window " << req->name << "\n";
  // TODO(lucasw) there could be a mutex only protecting the windows_
  std::lock_guard<std::mutex> lock(mutex_);
  res.success = true;
  if (req.remove) {
    if (windows_.count(req.name) > 0) {
      windows_.erase(req.name);
    }
    return true;
  }

  std::shared_ptr<Window> window;
  if (windows_.count(req.name) > 0) {
    window = windows_[req.name];
  } else {
    window = std::make_shared<Window>(req.name);
  }

  ImGuiWindowFlags window_flags = ImGuiWindowFlags_None;

  // TODO(lucasw) if python can have access to what flags are just pass in an int
  if (req.no_title_bar) { window_flags |= ImGuiWindowFlags_NoTitleBar; }
  if (req.no_resize) { window_flags |= ImGuiWindowFlags_NoResize; }
  if (req.no_move) { window_flags |= ImGuiWindowFlags_NoMove; }
  if (req.no_scrollbar) { window_flags |= ImGuiWindowFlags_NoScrollbar; }
  if (req.no_collapse) { window_flags |= ImGuiWindowFlags_NoCollapse; }
  if (req.no_decoration) { window_flags |= ImGuiWindowFlags_NoDecoration; }

  if (req.init) {
    window->setSettings(
        ImVec2(req.position.x, req.position.y),
        ImVec2(req.size.x, req.size.y),
        req.scroll_y,
        req.collapsed,
        window_flags);
  }

#if 0
  for (size_t i = 0; i < req.widgets.size(); ++i) {
    const auto tab_name = req.widgets[i].tab_name;
    if (req.widgets[i].remove) {
      window->remove(req.widgets[i].name, tab_name);
      continue;
    }
    std::string message;
    std::shared_ptr<Widget> widget;
    // std::cout << "new widget " << req.widgets[i].name << "\n";
    const bool rv = addWidget(req.widgets[i], message, widget);
    res->success = res->success && rv && widget;
    res->message += ", " + message;
    if (rv && widget) {
      widget->enable_info_ = req.widgets[i].enable_info;
      window->add(widget, tab_name);
    }
  }
#endif
  windows_[req.name] = window;
  return true;
}

#if 0
// TODO(lucasw) move into widget.cpp?
bool ImguiRos::addWidget(const imgui_ros::Widget& widget,
    std::string& message, std::shared_ptr<Widget>& imgui_widget)
{
  if (widget.type == imgui_ros::Widget::IMAGE) {
    std::shared_ptr<RosImage> ros_image;
    const bool sub_not_pub = true;
    // std::cout << "new ros image " << widget.name << "\n";

    // TODO(lucasw) if widget.topic already exists somewhere in a RosImage
    // subscriber need to re-use it, can't duplicate subscribers.i
    // viz3d has any number of RosImages also.
    ros_image.reset(new RosImage(widget.name, widget.topic, sub_not_pub,
        false, shared_from_this(),
        image_transfer_));
    ros_image->enable_draw_image_ = true;
    imgui_widget = ros_image;
    return true;
  ///////////////////////////////////////////////////////////////////////////
  // publisher types
  } else if (widget.type == imgui_ros::Widget::PUB) {
    std::shared_ptr<Pub> pub;
    int64_t value_int = widget.value;
    int64_t min = widget.min;
    int64_t max = widget.max;
    if (widget.sub_type == Widget::FLOAT32) {
      pub.reset(new GenericPub<std_msgs::Float32>(
          widget.name, widget.topic,
          widget.value, widget.min, widget.max, shared_from_this()));
    } else if (widget.sub_type == Widget::BOOL) {
      bool value = widget.value;
      pub.reset(new BoolPub(widget.name, widget.topic,
          value, shared_from_this()));
    // Templated types //////////////////////////////////
    } else if (widget.sub_type == Widget::FLOAT32) {
      pub.reset(new GenericPub<std_msgs::Float32>(
          widget.name, widget.topic,
          widget.value, widget.min, widget.max, shared_from_this()));
    } else if (widget.sub_type == Widget::FLOAT64) {
      pub.reset(new GenericPub<std_msgs::Float64>(
          widget.name, widget.topic,
          widget.value, widget.min, widget.max, shared_from_this()));
    } else if (widget.sub_type == Widget::INT8) {
      pub.reset(new GenericPub<std_msgs::Int8>(
          widget.name, widget.topic,
          value_int, min, max, shared_from_this()));
    } else if (widget.sub_type == Widget::INT16) {
      pub.reset(new GenericPub<std_msgs::Int16>(
          widget.name, widget.topic,
          value_int, min, max, shared_from_this()));
    } else if (widget.sub_type == Widget::INT32) {
      pub.reset(new GenericPub<std_msgs::Int32>(
          widget.name, widget.topic,
          value_int, min, max, shared_from_this()));
    } else if (widget.sub_type == Widget::INT64) {
      pub.reset(new GenericPub<std_msgs::Int64>(
          widget.name, widget.topic,
          value_int, min, max, shared_from_this()));
    } else if (widget.sub_type == Widget::UINT8) {
      pub.reset(new GenericPub<std_msgs::UInt8>(
          widget.name, widget.topic,
          value_int, min, max, shared_from_this()));
    } else if (widget.sub_type == Widget::UINT16) {
      pub.reset(new GenericPub<std_msgs::UInt16>(
          widget.name, widget.topic,
          value_int, min, max, shared_from_this()));
    } else if (widget.sub_type == Widget::UINT32) {
      pub.reset(new GenericPub<std_msgs::UInt32>(
          widget.name, widget.topic,
          value_int, min, max, shared_from_this()));
    } else if (widget.sub_type == Widget::UINT64) {
      pub.reset(new GenericPub<std_msgs::UInt64>(
          widget.name, widget.topic,
          value_int, min, max, shared_from_this()));
    ////////////////////////////////////////////////////
    } else if (widget.sub_type == Widget::STRING) {
      pub.reset(new StringPub(
          widget.name, widget.topic,
          widget.items, shared_from_this()));
    } else if (widget.sub_type == Widget::MENU) {
      int value = widget.value;
      pub.reset(new MenuPub(widget.name, widget.topic,
          value, widget.items, shared_from_this()));
    } else if (widget.sub_type == Widget::TF) {
      RCLCPP_DEBUG(get_logger(), "new tf pub");
      if (widget.items.size() < 2) {
        std::stringstream ss;
        ss << widget.name << " need two widget items for tf parent and child "
            << widget.items.size();
        message = ss.str();
        return false;
      }
      pub.reset(new TfBroadcaster(widget.name,
          widget.items[0], widget.items[1],
          widget.min, widget.max,
          tf_buffer_,
          shared_from_this()));
    } else {
      std::stringstream ss;
      ss << "unsupported window type " << std::dec << widget.sub_type;
      message = ss.str();
      return false;
    }
    imgui_widget = pub;
    return true;
  ///////////////////////////////////////////////////////////////////////////
  // subscription types
  } else if (widget.type == imgui_ros::Widget::SUB) {
    RCLCPP_DEBUG(get_logger(), "new sub %s %d", widget.name.c_str(), widget.sub_type);
    std::shared_ptr<Sub> sub;
    if (widget.sub_type == Widget::FLOAT32) {
      sub.reset(new GenericSub<std_msgs::Float32>(
          widget.name, widget.topic,
          shared_from_this()));
    } else if (widget.sub_type == Widget::FLOAT64) {
      sub.reset(new GenericSub<std_msgs::Float64>(
          widget.name, widget.topic,
          shared_from_this()));
    } else if (widget.sub_type == Widget::INT8) {
      sub.reset(new GenericSub<std_msgs::Int8>(
          widget.name, widget.topic,
          shared_from_this()));
    } else if (widget.sub_type == Widget::INT16) {
      sub.reset(new GenericSub<std_msgs::Int16>(
          widget.name, widget.topic,
          shared_from_this()));
    } else if (widget.sub_type == Widget::INT32) {
      sub.reset(new GenericSub<std_msgs::Int32>(
          widget.name, widget.topic,
          shared_from_this()));
    } else if (widget.sub_type == Widget::INT64) {
      sub.reset(new GenericSub<std_msgs::Int64>(
          widget.name, widget.topic,
          shared_from_this()));
    } else if (widget.sub_type == Widget::UINT8) {
      sub.reset(new GenericSub<std_msgs::UInt8>(
          widget.name, widget.topic,
          shared_from_this()));
    } else if (widget.sub_type == Widget::UINT16) {
      sub.reset(new GenericSub<std_msgs::UInt16>(
          widget.name, widget.topic,
          shared_from_this()));
    } else if (widget.sub_type == Widget::UINT32) {
      sub.reset(new GenericSub<std_msgs::UInt32>(
          widget.name, widget.topic,
          shared_from_this()));
    } else if (widget.sub_type == Widget::UINT64) {
      sub.reset(new GenericSub<std_msgs::UInt64>(
          widget.name, widget.topic,
          shared_from_this()));
    } else if (widget.sub_type == Widget::STRING) {
      sub.reset(new GenericSub<std_msgs::String>(
          widget.name, widget.topic,
          shared_from_this()));
    } else if (widget.sub_type == Widget::TF) {
      RCLCPP_DEBUG(get_logger(), "new tf echo");
      if (widget.items.size() < 2) {
        std::stringstream ss;
        ss << widget.name << " need two widget items for tf parent and child "
            << widget.items.size();
        message = ss.str();
        return false;
      }
      sub.reset(new TfEcho(widget.name,
          widget.items[0], widget.items[1],
          tf_buffer_,
          shared_from_this()));
    } else if (widget.sub_type == Widget::VIZ2D) {
      ROS_INFO_STREAM("new viz 2d");
      if (widget.items.size() < 1) {
        std::stringstream ss;
        ss << widget.name << " need two widget items for tf parent and child "
            << widget.items.size();
        message = ss.str();
        return false;
      }
      const std::string frame_id = widget.items[0];
      const double pixels_per_meter = widget.max;
      sub.reset(new Viz2D(widget.name,
          widget.topic,
          frame_id,
          widget.items,  // all the tf frames to display, later make these configurable live
          pixels_per_meter,
          tf_buffer_,
          shared_from_this()));
    } else if (widget.sub_type == Widget::BOOL) {
      bool value = widget.value;
      sub.reset(new BoolSub(widget.name, widget.topic,  // widget.sub_type,
          value, shared_from_this()));
    } else if (widget.sub_type == Widget::POINTCLOUD) {
      // There won't be anything in the shape to draw yet
      if (widget.remove) {
        viz3d->removeShape(widget.name);
      } else {
        auto pc = std::make_shared<PointCloud>(widget.name,
            widget.topic, tf_buffer_, shared_from_this());
        sub = pc;
        viz3d->addOrReplaceShape(pc->shape_->name_, pc->shape_);
      }
    } else {
      std::stringstream ss;
      ss << "unsupported window type " << std::dec << widget.sub_type;
      message = ss.str();
      return false;
    }
    imgui_widget = sub;
    return true;
  } else if (widget.type == imgui_ros::Widget::PLOT) {
    std::shared_ptr<Sub> sub;
    if (widget.sub_type == Widget::BOOL) {
      sub.reset(new PlotSub<std_msgs::Bool>(
          widget.name, widget.topic, widget.value,
          shared_from_this()));
    } else if (widget.sub_type == Widget::FLOAT32) {
      sub.reset(new PlotSub<std_msgs::Float32>(
          widget.name, widget.topic, widget.value,
          shared_from_this()));
    } else if (widget.sub_type == Widget::FLOAT64) {
      sub.reset(new PlotSub<std_msgs::Float64>(
          widget.name, widget.topic, widget.value,
          shared_from_this()));
    } else if (widget.sub_type == Widget::INT8) {
      sub.reset(new PlotSub<std_msgs::Int8>(
          widget.name, widget.topic, widget.value,
          shared_from_this()));
    } else if (widget.sub_type == Widget::INT16) {
      sub.reset(new PlotSub<std_msgs::Int16>(
          widget.name, widget.topic, widget.value,
          shared_from_this()));
    } else if (widget.sub_type == Widget::INT32) {
      sub.reset(new PlotSub<std_msgs::Int32>(
          widget.name, widget.topic, widget.value,
          shared_from_this()));
    } else if (widget.sub_type == Widget::INT64) {
      sub.reset(new PlotSub<std_msgs::Int64>(
          widget.name, widget.topic, widget.value,
          shared_from_this()));
    } else if (widget.sub_type == Widget::UINT8) {
      sub.reset(new PlotSub<std_msgs::UInt8>(
          widget.name, widget.topic, widget.value,
          shared_from_this()));
    } else if (widget.sub_type == Widget::UINT16) {
      sub.reset(new PlotSub<std_msgs::UInt16>(
          widget.name, widget.topic, widget.value,
          shared_from_this()));
    } else if (widget.sub_type == Widget::UINT32) {
      sub.reset(new PlotSub<std_msgs::UInt32>(
          widget.name, widget.topic, widget.value,
          shared_from_this()));
    } else if (widget.sub_type == Widget::UINT64) {
      sub.reset(new PlotSub<std_msgs::UInt64>(
          widget.name, widget.topic, widget.value,
          shared_from_this()));
    } else {
      std::stringstream ss;
      ss << "unsupported window type " << std::dec << widget.sub_type;
      message = ss.str();
      return false;
    }
    imgui_widget = sub;
  //////////////////////////////////////////////////////////
  } else if (widget.type == imgui_ros::Widget::PARAM) {
    std::shared_ptr<Param> param;
    const std::string node_name = widget.topic;
    if (widget.items.size() < 1) {
      std::stringstream ss;
      ss << widget.name << " Need to specify parameter name in widget items[0]";
      message = ss.str();
      return false;
    }
    // TODO(lucasw) have a better variable for this
    const std::string parameter_name = widget.items[0];
    if (widget.sub_type == Widget::BOOL) {
      param.reset(new Param(widget.name,
          node_name, parameter_name,
          rcl_interfaces::ParameterType::PARAMETER_BOOL,
          0, 1,
          shared_from_this()));
    } else if ((widget.sub_type == Widget::FLOAT32) ||
        (widget.sub_type == Widget::FLOAT64)) {
      param.reset(new Param(widget.name,
          node_name, parameter_name,
          rcl_interfaces::ParameterType::PARAMETER_DOUBLE,
          widget.min, widget.max,
          shared_from_this()));
    } else if ((widget.sub_type == Widget::INT8) ||
        (widget.sub_type == Widget::INT16) ||
        (widget.sub_type == Widget::INT32) ||
        (widget.sub_type == Widget::INT64)) {
      param.reset(new Param(widget.name,
          node_name, parameter_name,
          rcl_interfaces::ParameterType::PARAMETER_INTEGER,
          widget.min, widget.max,
          shared_from_this()));
    } else if ((widget.sub_type == Widget::STRING)) {
      param.reset(new Param(widget.name,
          node_name, parameter_name,
          rcl_interfaces::ParameterType::PARAMETER_STRING,
          widget.min, widget.max,
          shared_from_this()));
    } else {
      std::stringstream ss;
      ss << widget.name << " " << node_name << " " <<  parameter_name
          << " " << widget.sub_type
          << " Unexpected param type";
      message = ss.str();
      return false;
    }
    // TODO(lucasw) need to handle deletion
    if (parameters_clients_.count(node_name) < 1) {
      parameters_clients_[node_name] = std::make_shared<ros::AsyncParametersClient>(this, node_name);
    }
    // TODO(lucasw) need to handle deletion
    param_widgets_[node_name][widget.name] = param;
    imgui_widget = param;
  ///////////////////////////////////////////////////////////////////////////
  } else if (widget.type == imgui_ros::Widget::GRAPH) {
    imgui_widget = std::make_shared<Graph>(widget.name, shared_from_this());
  } else {
    std::stringstream ss;
    // TODO(lucasw) typeToString()
    ss << "unsupported type " << widget.type << " " << widget.sub_type;
    message = ss.str();
    return false;
  }
  return true;
}
#endif

// TODO(lucasw) move to draw method
void ImguiRos::drawStats(ros::Time stamp)
{
  if (stats_window_init_) {
    stats_window_init_ = false;
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(400, 500));
    ImGui::SetNextWindowCollapsed(false);
  }
  ImGui::Begin("##stats");
  ImGui::Text("%1.4f", stamp.toSec());
  // Make tab, put all viz3d stuff into tab?
  ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
              1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
  std::stringstream ss;
  ss << std::this_thread::get_id();  // << " " << thread_id_;
  ImGui::Text("Thread %s", ss.str().c_str());
  // TODO(lucasw) put this in different tab?
  // image_transfer_->draw(stamp);
  ImGui::End();
}

void ImguiRos::update(const ros::TimerEvent& ev)
{
  if (!init_) {
    glInit();
  }

  const auto& stamp = ev.current_real;

  if (restore_window_size_) {
    // this is ineffective - is imgui overriding?
    ROS_INFO_STREAM("restoring window "
        << old_width_ << " " << old_height_);
    SDL_SetWindowSize(sdl_window_, old_width_, old_height_);
    restore_window_size_ = false;
  }

  // Poll and handle events (inputs, window resize, etc.)
  // You can read the gui_io.WantCaptureMouse, gui_io.WantCaptureKeyboard flags to
  // tell if dear imgui wants to use your inputs.
  // - When gui_io.WantCaptureMouse is true, do not dispatch mouse input data to
  // your main application.
  // - When gui_io.WantCaptureKeyboard is true, do not dispatch keyboard input
  // data to your main application. Generally you may always pass all inputs
  // to dear imgui, and hide them from your application based on those two
  // flags.
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    if (!ros::ok())
      return;
    ImGui_ImplSDL2_ProcessEvent(&event);
    if (event.type == SDL_KEYUP) {
      if (event.key.keysym.sym == SDLK_F11) {
        ROS_INFO_STREAM("TODO switch to/from fullscreen");
        if (fullscreen_) {
          SDL_SetWindowFullscreen(sdl_window_, 0);
          // this is ineffective - is imgui overriding?
          // SDL_SetWindowSize(sdl_window_, old_width_, old_height_);
          restore_window_size_ = true;
          fullscreen_ = false;
        } else {
          SDL_DisplayMode current;
          // TODO(lucasw) there are potentially multiple displays,
          // which is the current one?
          const int rv = SDL_GetCurrentDisplayMode(0, &current);
          SDL_GetWindowSize(sdl_window_, &old_width_, &old_height_);
          ROS_INFO_STREAM(current.w << " " << current.h << ", "
              << old_width_ << " " << old_height_);
          SDL_SetWindowSize(sdl_window_, current.w, current.h);
          SDL_SetWindowFullscreen(sdl_window_, SDL_WINDOW_FULLSCREEN);
          fullscreen_ = true;
        }
      }
    } else if (event.type == SDL_QUIT) {
      ros::shutdown();
      return;
    } else if (event.type == SDL_WINDOWEVENT &&
        event.window.event == SDL_WINDOWEVENT_CLOSE &&
        event.window.windowID == SDL_GetWindowID(sdl_window_)) {
      ROS_INFO_STREAM("window closed - shutting down");
      ros::shutdown();
      return;
    }
    // TODO(lucasw) support drag and drop loading of textures with special widget
  }

  {
  std::lock_guard<std::mutex> lock(mutex_);
  // Start the Dear ImGui frame
  if (std::this_thread::get_id() != thread_id_) {
    std::cerr << "imgui thread " << std::this_thread::get_id() <<
      " " << thread_id_ << std::endl;
    // TODO(lucasw) throw
    return;
  }
  imgui_impl_opengl3_->NewFrame();
  ImGui_ImplSDL2_NewFrame(sdl_window_);
  ImGui::NewFrame();

  {
#if 0
    // TODO(lucasw) make image_transfer into a widget?
    #ifndef RUN_IMAGE_TRANSFER_SEPARATE_THREAD
    ros::spin_some(image_transfer_);
    image_transfer_->update();
    #endif
#endif

    for (auto& window : windows_) {
      if (window.second) {
        window.second->update(stamp);
      }
    }

    // TODO(lucasw) mutex lock just for windows
    for (auto& window : windows_) {
      if (window.second) {
        window.second->draw();
      }
    }
    drawStats(stamp);
  }

  //////////////////////////////////////////////////////////////
  // Rendering
  ImGui::Render();
  // TODO(lucasw) or wait until after GetDrawData() to unlock?
  }

#if 0
  viz3d->render_message_.str("");
  // Need to render these before using them in the regular viz3d render below
  viz3d->renderShadows();
#endif

  SDL_GL_MakeCurrent(sdl_window_, gl_context);
  checkGLError(__FILE__, __LINE__);
  const int display_size_x = ImGui::GetIO().DisplaySize.x;
  const int display_size_y = ImGui::GetIO().DisplaySize.y;


  glViewport(0, 0, (int)display_size_x, (int)display_size_y);
  checkGLError(__FILE__, __LINE__);
  glClearColor(0.1, 0.2, 0.3, 1.0);
#if 0
  glClearColor(
      viz3d->clear_color_.x,
      viz3d->clear_color_.y,
      viz3d->clear_color_.z,
      viz3d->clear_color_.w);
#endif
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  // TODO(lucasw) render anything else into the background here,
  // and the ui will appear over it?
  // bgfx does the 3D render after imgui render

  checkGLError(__FILE__, __LINE__);
#if 0
  if (true) {
    const int fb_width = display_size_x * ImGui::GetIO().DisplayFramebufferScale.x;
    const int fb_height = display_size_y * ImGui::GetIO().DisplayFramebufferScale.y;
    viz3d->render(fb_width, fb_height,
        ImGui::GetDrawData()->DisplayPos.x, ImGui::GetDrawData()->DisplayPos.y,
        ImGui::GetDrawData()->DisplaySize.x, ImGui::GetDrawData()->DisplaySize.y
        );
  }
#endif
  checkGLError(__FILE__, __LINE__);
  imgui_impl_opengl3_->RenderDrawData(ImGui::GetDrawData());
  checkGLError(__FILE__, __LINE__);

  SDL_GL_SwapWindow(sdl_window_);
  ////////////////////////////////////////////////////////////////////

#if 0
  {
    // need to do this out of the main rendering above
    viz3d->renderCubeCameras();
    viz3d->renderToTexture();
  }

  // update all tfs
  tf2_msgs::TFMessage tfs;
  for (auto& window : windows_) {
    if (window.second) {
      window.second->addTF(tfs, stamp);
    }
  }
  if (tfs.transforms.size() > 0) {
    tf_pub_->publish(tfs);
  }

  // update all parameters that need to be updated
  for (auto& param_widgets_pair : param_widgets_) {
    const std::string node_name = param_widgets_pair.first;
    std::vector<ros::Parameter> parameters;
    for (auto& param_widget_pair : param_widgets_pair.second) {
      auto param_widget = param_widget_pair.second;
      if (param_widget->update_) {
        parameters.push_back(ros::Parameter(
            param_widget->parameter_name_, param_widget->value_));
        param_widget->update_ = false;
      }
    }
    parameters_clients_[node_name]->set_parameters(parameters);
  } // param update

#endif
}

}  // namespace imgui_ros

int main(int argn, char* argv[])
{
  ros::init(argn, argv, "imgui_ros");
  auto imgui_ros = std::make_shared<imgui_ros::ImguiRos>();
  ros::spin();
}

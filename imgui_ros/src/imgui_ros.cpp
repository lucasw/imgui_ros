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
// #include <geometry_msgs/msg/pose.hpp>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <imgui.h>
#include <imgui_impl_sdl.h>
#pragma GCC diagnostic pop
#include <imgui_ros/srv/add_window.hpp>
#include <imgui_ros/image.h>
#include <imgui_ros/imgui_impl_opengl3.h>
#include <imgui_ros/imgui_ros.h>
#include <imgui_ros/param.h>
#include <imgui_ros/pub.h>
#include <imgui_ros/sub.h>
#include <imgui_ros/tf.h>
#include <imgui_ros/viz2d.h>
// #include <opencv2/highgui.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace imgui_ros {
  ImguiRos::ImguiRos() : Node("imgui_ros") {

    tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf");
    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    #if 1
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);
    #else
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    #endif
    tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // TODO(lucasw) check if width and height are > minimum value
    get_parameter_or("name", name_, name_);
    get_parameter_or("width", width_, width_);
    get_parameter_or("height", height_, height_);

    // parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);

    // building this causes the node to crash only in release mode
    add_tf_ = create_service<srv::AddTf>("add_tf",
        std::bind(&ImguiRos::addTf, this, _1, _2));

    add_window_ = create_service<srv::AddWindow>("add_window",
        std::bind(&ImguiRos::addWindow, this, _1, _2));

    update_timer_ = this->create_wall_timer(33ms,
        std::bind(&ImguiRos::update, this));
  }

  ImguiRos::~ImguiRos() {
    // Cleanup
    imgui_impl_opengl3_->Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();
  }

  void ImguiRos::glInit() {
    RCLCPP_INFO(this->get_logger(), "opengl init %d", init_);

    // Setup SDL
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
      RCLCPP_ERROR(this->get_logger(), "Error: %s", SDL_GetError());
      // TODO(lucasw) throw
      return;
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
    window = SDL_CreateWindow(
        name_.c_str(), SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED, width_, height_,
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE); // | SDL_WINDOW_BORDERLESS);
    gl_context = SDL_GL_CreateContext(window);
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
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize OpenGL loader!");
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    // Setup Dear ImGui binding
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard
    // Controls

    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
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

    const std::string viz3d_name = "main render window";
    viz3d = std::make_shared<Viz3D>(viz3d_name, "shapes",
        imgui_impl_opengl3_,
        tf_buffer_,
        shared_from_this());

    windows_[viz3d_name] = viz3d;

    get_parameter_or("red", viz3d->clear_color_.x, viz3d->clear_color_.x);
    get_parameter_or("green", viz3d->clear_color_.y, viz3d->clear_color_.y);
    get_parameter_or("blue", viz3d->clear_color_.z, viz3d->clear_color_.z);
    get_parameter_or("alpha", viz3d->clear_color_.w, viz3d->clear_color_.w);

    init_ = true;  // viz3d->initialized_;
  }

  void ImguiRos::addTf(const std::shared_ptr<imgui_ros::srv::AddTf::Request> req,
             std::shared_ptr<imgui_ros::srv::AddTf::Response> res)
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
    window->add(pub);
    res->success = true;
    res->message += "added tf pub " + req->tf.name + " to " + req->tf.window;
  }

  void ImguiRos::addWindow(const std::shared_ptr<imgui_ros::srv::AddWindow::Request> req,
      std::shared_ptr<imgui_ros::srv::AddWindow::Response> res) {
    // std::cout << "adding window " << req->name << "\n";
    // TODO(lucasw) there could be a mutex only protecting the windows_
    std::lock_guard<std::mutex> lock(mutex_);
    res->success = true;
    if (req->remove) {
      if (windows_.count(req->name) > 0) {
        windows_.erase(req->name);
      }
      return;
    }
    auto window = std::make_shared<Window>(req->name);
    for (size_t i = 0; i < req->widgets.size(); ++i) {
      if (req->widgets[i].remove) {
        window->remove(req->widgets[i].name);
        continue;
      }
      std::string message;
      std::shared_ptr<Widget> widget;
      // std::cout << "new widget " << req->widgets[i].name << "\n";
      const bool rv = addWidget(req->widgets[i], message, widget);
      res->success = res->success && rv && widget;
      res->message += ", " + message;
      if (rv && widget) {
        window->add(widget);
      }
    }
    windows_[req->name] = window;
  }

  // TODO(lucasw) move into widget.cpp?
  bool ImguiRos::addWidget(const imgui_ros::msg::Widget& widget,
      std::string& message, std::shared_ptr<Widget>& imgui_widget) {
    if (widget.type == imgui_ros::msg::Widget::IMAGE) {
      std::shared_ptr<RosImage> ros_image;
      const bool sub_not_pub = true;
      // std::cout << "new ros image " << widget.name << "\n";
      ros_image.reset(new RosImage(widget.name, widget.topic, sub_not_pub, shared_from_this()));
      ros_image->enable_draw_image_ = true;
      imgui_widget = ros_image;
      return true;
    // publisher types
    } else if (widget.type == imgui_ros::msg::Widget::PUB) {
      std::shared_ptr<Pub> pub;
      int64_t value_int = widget.value;
      int64_t min = widget.min;
      int64_t max = widget.max;
      if (widget.sub_type == msg::Widget::FLOAT32) {
        pub.reset(new GenericPub<std_msgs::msg::Float32>(
            widget.name, widget.topic,
            widget.value, widget.min, widget.max, shared_from_this()));
      } else if (widget.sub_type == msg::Widget::BOOL) {
        bool value = widget.value;
        pub.reset(new BoolPub(widget.name, widget.topic,
            value, shared_from_this()));
      // Templated types //////////////////////////////////
      } else if (widget.sub_type == msg::Widget::FLOAT32) {
        pub.reset(new GenericPub<std_msgs::msg::Float32>(
            widget.name, widget.topic,
            widget.value, widget.min, widget.max, shared_from_this()));
      } else if (widget.sub_type == msg::Widget::FLOAT64) {
        pub.reset(new GenericPub<std_msgs::msg::Float64>(
            widget.name, widget.topic,
            widget.value, widget.min, widget.max, shared_from_this()));
      } else if (widget.sub_type == msg::Widget::INT8) {
        pub.reset(new GenericPub<std_msgs::msg::Int8>(
            widget.name, widget.topic,
            value_int, min, max, shared_from_this()));
      } else if (widget.sub_type == msg::Widget::INT16) {
        pub.reset(new GenericPub<std_msgs::msg::Int16>(
            widget.name, widget.topic,
            value_int, min, max, shared_from_this()));
      } else if (widget.sub_type == msg::Widget::INT32) {
        pub.reset(new GenericPub<std_msgs::msg::Int32>(
            widget.name, widget.topic,
            value_int, min, max, shared_from_this()));
      } else if (widget.sub_type == msg::Widget::INT64) {
        pub.reset(new GenericPub<std_msgs::msg::Int64>(
            widget.name, widget.topic,
            value_int, min, max, shared_from_this()));
      } else if (widget.sub_type == msg::Widget::UINT8) {
        pub.reset(new GenericPub<std_msgs::msg::UInt8>(
            widget.name, widget.topic,
            value_int, min, max, shared_from_this()));
      } else if (widget.sub_type == msg::Widget::UINT16) {
        pub.reset(new GenericPub<std_msgs::msg::UInt16>(
            widget.name, widget.topic,
            value_int, min, max, shared_from_this()));
      } else if (widget.sub_type == msg::Widget::UINT32) {
        pub.reset(new GenericPub<std_msgs::msg::UInt32>(
            widget.name, widget.topic,
            value_int, min, max, shared_from_this()));
      } else if (widget.sub_type == msg::Widget::UINT64) {
        pub.reset(new GenericPub<std_msgs::msg::UInt64>(
            widget.name, widget.topic,
            value_int, min, max, shared_from_this()));
      ////////////////////////////////////////////////////
      } else if (widget.sub_type == msg::Widget::STRING) {
        pub.reset(new StringPub(
            widget.name, widget.topic,
            widget.items, shared_from_this()));
      } else if (widget.sub_type == msg::Widget::MENU) {
        int value = widget.value;
        pub.reset(new MenuPub(widget.name, widget.topic,
            value, widget.items, shared_from_this()));
      } else if (widget.sub_type == msg::Widget::TF) {
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
    // subscription types
    } else if (widget.type == imgui_ros::msg::Widget::SUB) {
      RCLCPP_DEBUG(get_logger(), "new sub %s %d", widget.name.c_str(), widget.sub_type);
      std::shared_ptr<Sub> sub;
      if (widget.sub_type == msg::Widget::FLOAT32) {
        sub.reset(new GenericSub<std_msgs::msg::Float32>(
            widget.name, widget.topic,
            shared_from_this()));
      } else if (widget.sub_type == msg::Widget::FLOAT64) {
        sub.reset(new GenericSub<std_msgs::msg::Float64>(
            widget.name, widget.topic,
            shared_from_this()));
      } else if (widget.sub_type == msg::Widget::INT8) {
        sub.reset(new GenericSub<std_msgs::msg::Int8>(
            widget.name, widget.topic,
            shared_from_this()));
      } else if (widget.sub_type == msg::Widget::INT16) {
        sub.reset(new GenericSub<std_msgs::msg::Int16>(
            widget.name, widget.topic,
            shared_from_this()));
      } else if (widget.sub_type == msg::Widget::INT32) {
        sub.reset(new GenericSub<std_msgs::msg::Int32>(
            widget.name, widget.topic,
            shared_from_this()));
      } else if (widget.sub_type == msg::Widget::INT64) {
        sub.reset(new GenericSub<std_msgs::msg::Int64>(
            widget.name, widget.topic,
            shared_from_this()));
      } else if (widget.sub_type == msg::Widget::UINT8) {
        sub.reset(new GenericSub<std_msgs::msg::UInt8>(
            widget.name, widget.topic,
            shared_from_this()));
      } else if (widget.sub_type == msg::Widget::UINT16) {
        sub.reset(new GenericSub<std_msgs::msg::UInt16>(
            widget.name, widget.topic,
            shared_from_this()));
      } else if (widget.sub_type == msg::Widget::UINT32) {
        sub.reset(new GenericSub<std_msgs::msg::UInt32>(
            widget.name, widget.topic,
            shared_from_this()));
      } else if (widget.sub_type == msg::Widget::UINT64) {
        sub.reset(new GenericSub<std_msgs::msg::UInt64>(
            widget.name, widget.topic,
            shared_from_this()));
      } else if (widget.sub_type == msg::Widget::STRING) {
        sub.reset(new GenericSub<std_msgs::msg::String>(
            widget.name, widget.topic,
            shared_from_this()));
      } else if (widget.sub_type == msg::Widget::TF) {
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
      } else if (widget.sub_type == msg::Widget::VIZ2D) {
        RCLCPP_INFO(get_logger(), "new viz 2d");
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
      } else if (widget.sub_type == msg::Widget::BOOL) {
        bool value = widget.value;
        sub.reset(new BoolSub(widget.name, widget.topic,  // widget.sub_type,
            value, shared_from_this()));
      } else {
        std::stringstream ss;
        ss << "unsupported window type " << std::dec << widget.sub_type;
        message = ss.str();
        return false;
      }
      imgui_widget = sub;
      return true;
    } else if (widget.type == imgui_ros::msg::Widget::PLOT) {
      std::shared_ptr<Sub> sub;
      if (widget.sub_type == msg::Widget::BOOL) {
        sub.reset(new PlotSub<std_msgs::msg::Bool>(
            widget.name, widget.topic, widget.value,
            shared_from_this()));
      } else if (widget.sub_type == msg::Widget::FLOAT32) {
        sub.reset(new PlotSub<std_msgs::msg::Float32>(
            widget.name, widget.topic, widget.value,
            shared_from_this()));
      } else if (widget.sub_type == msg::Widget::FLOAT64) {
        sub.reset(new PlotSub<std_msgs::msg::Float64>(
            widget.name, widget.topic, widget.value,
            shared_from_this()));
      } else if (widget.sub_type == msg::Widget::INT8) {
        sub.reset(new PlotSub<std_msgs::msg::Int8>(
            widget.name, widget.topic, widget.value,
            shared_from_this()));
      } else if (widget.sub_type == msg::Widget::INT16) {
        sub.reset(new PlotSub<std_msgs::msg::Int16>(
            widget.name, widget.topic, widget.value,
            shared_from_this()));
      } else if (widget.sub_type == msg::Widget::INT32) {
        sub.reset(new PlotSub<std_msgs::msg::Int32>(
            widget.name, widget.topic, widget.value,
            shared_from_this()));
      } else if (widget.sub_type == msg::Widget::INT64) {
        sub.reset(new PlotSub<std_msgs::msg::Int64>(
            widget.name, widget.topic, widget.value,
            shared_from_this()));
      } else if (widget.sub_type == msg::Widget::UINT8) {
        sub.reset(new PlotSub<std_msgs::msg::UInt8>(
            widget.name, widget.topic, widget.value,
            shared_from_this()));
      } else if (widget.sub_type == msg::Widget::UINT16) {
        sub.reset(new PlotSub<std_msgs::msg::UInt16>(
            widget.name, widget.topic, widget.value,
            shared_from_this()));
      } else if (widget.sub_type == msg::Widget::UINT32) {
        sub.reset(new PlotSub<std_msgs::msg::UInt32>(
            widget.name, widget.topic, widget.value,
            shared_from_this()));
      } else if (widget.sub_type == msg::Widget::UINT64) {
        sub.reset(new PlotSub<std_msgs::msg::UInt64>(
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
    } else if (widget.type == imgui_ros::msg::Widget::PARAM) {
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
      if (widget.sub_type == msg::Widget::BOOL) {
        param.reset(new Param(widget.name,
            node_name, parameter_name,
            rcl_interfaces::msg::ParameterType::PARAMETER_BOOL,
            0, 1,
            shared_from_this()));
      } else if ((widget.sub_type == msg::Widget::FLOAT32) ||
          (widget.sub_type == msg::Widget::FLOAT64)) {
        param.reset(new Param(widget.name,
            node_name, parameter_name,
            rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE,
            widget.min, widget.max,
            shared_from_this()));
      } else if ((widget.sub_type == msg::Widget::INT8) ||
          (widget.sub_type == msg::Widget::INT16) ||
          (widget.sub_type == msg::Widget::INT32) ||
          (widget.sub_type == msg::Widget::INT64)) {
        param.reset(new Param(widget.name,
            node_name, parameter_name,
            rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER,
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
        parameters_clients_[node_name] = std::make_shared<rclcpp::AsyncParametersClient>(this, node_name);
      }
      // TODO(lucasw) need to handle deletion
      param_widgets_[node_name][widget.name] = param;
      imgui_widget = param;
    } else {
      std::stringstream ss;
      // TODO(lucasw) typeToString()
      ss << "unsupported type " << widget.type << " " << widget.sub_type;
      message = ss.str();
      return false;
    }
    return true;
  }

  void ImguiRos::update() {
    if (!init_) {
      glInit();
#if 0
    // can't do this in constructor because node hasn't finished yet?
    // but then putting it here ruins ability of param clients to interact with other nodes parameters
      param_sub_ = parameters_client_->on_parameter_event(
          std::bind(&ImguiRos::onParameterEvent, this, std::placeholders::_1));
#endif
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
      if (!rclcpp::ok())
        return;
      ImGui_ImplSDL2_ProcessEvent(&event);
      if (event.type == SDL_QUIT) {
        rclcpp::shutdown();
        return;
      }
      if (event.type == SDL_WINDOWEVENT &&
          event.window.event == SDL_WINDOWEVENT_CLOSE &&
          event.window.windowID == SDL_GetWindowID(window)) {
        rclcpp::shutdown();
        return;
      }
    }

    {
    std::lock_guard<std::mutex> lock(mutex_);
    // Start the Dear ImGui frame
    imgui_impl_opengl3_->NewFrame();
    ImGui_ImplSDL2_NewFrame(window);
    ImGui::NewFrame();

    {
      viz3d->update();

      ImGui::Begin("stats"); // Create a window called "stats"
                             // and append into it.

      ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                  1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
      ImGui::End();

      // TODO(lucasw) mutex lock just for windows
      for (auto& window : windows_) {
        if (window.second) {
          window.second->draw();
        }
      }
    }

    //////////////////////////////////////////////////////////////
    // Rendering
    ImGui::Render();
    // TODO(lucasw) or wait until after GetDrawData() to unlock?
    }

    viz3d->render_message_.str("");
    // Need to render these before using them in the regular viz3d render below
    viz3d->renderShadows();

    {
    SDL_GL_MakeCurrent(window, gl_context);
    checkGLError(__FILE__, __LINE__);
    const int display_size_x = ImGui::GetIO().DisplaySize.x;
    const int display_size_y = ImGui::GetIO().DisplaySize.y;
    const int fb_width = display_size_x * ImGui::GetIO().DisplayFramebufferScale.x;
    const int fb_height = display_size_y * ImGui::GetIO().DisplayFramebufferScale.y;


    glViewport(0, 0, (int)display_size_x, (int)display_size_y);
    checkGLError(__FILE__, __LINE__);
    // glClearColor(clear_color_.x, clear_color_.y, clear_color_.z, clear_color_.w);
    glClearColor(
        viz3d->clear_color_.x,
        viz3d->clear_color_.y,
        viz3d->clear_color_.z,
        viz3d->clear_color_.w);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // TODO(lucasw) render anything else into the background here,
    // and the ui will appear over it?
    // bgfx does the 3D render after imgui render

    checkGLError(__FILE__, __LINE__);
    if (true) {
      viz3d->render(fb_width, fb_height,
          ImGui::GetDrawData()->DisplayPos.x, ImGui::GetDrawData()->DisplayPos.y,
          ImGui::GetDrawData()->DisplaySize.x, ImGui::GetDrawData()->DisplaySize.y
          );
    }
    checkGLError(__FILE__, __LINE__);
    imgui_impl_opengl3_->RenderDrawData(ImGui::GetDrawData());
    checkGLError(__FILE__, __LINE__);

    SDL_GL_SwapWindow(window);
    }
    ////////////////////////////////////////////////////////////////////

    {
      // need to do this out of the main rendering above
      viz3d->renderCubeCameras();
      viz3d->renderToTexture();
    }

    // update all tfs
    tf2_msgs::msg::TFMessage tfs;
    rclcpp::Time cur = now();
    for (auto& window : windows_) {
      if (window.second) {
        window.second->addTF(tfs, cur);
      }
    }
    if (tfs.transforms.size() > 0) {
      tf_pub_->publish(tfs);
    }

    // update all parameters that need to be updated
    for (auto& param_widgets_pair : param_widgets_) {
      const std::string node_name = param_widgets_pair.first;
      std::vector<rclcpp::Parameter> parameters;
      for (auto& param_widget_pair : param_widgets_pair.second) {
        auto param_widget = param_widget_pair.second;
        if (param_widget->update_) {
          parameters.push_back(rclcpp::Parameter(
              param_widget->parameter_name_, param_widget->value_));
          param_widget->update_ = false;
        }
      }
      parameters_clients_[node_name]->set_parameters(parameters);
    } // param update
  }

  // TODO(lucasw) need to push this up into containing viz3d class,
  // it will have a list of namespaces that it has parameter events for and will
  // receive the event and distribute the values to the proper param
  void ImguiRos::onParameterEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
  {
    // RCLCPP_INFO(get_logger(), "%s", event->node);
    std::cout << event->node << "\n";
    for (auto & parameter : event->new_parameters) {
      std::cout << parameter.name << " ";
    }
    std::cout << "\nchanged";
    for (auto & parameter : event->changed_parameters) {
      std::cout << parameter.name << " ";
    }
    std::cout << "\n";
#if 0
    if (event->node != node_name_) {
      return;
    }
    for (auto & parameter : event->new_parameters) {
      if (parameter.name == parameter_name_) {
        updateValue(parameter.value);
      }
    }
    for (auto & parameter : event->changed_parameters) {
      if (parameter.name == parameter_name_) {
        updateValue(parameter.value);
      }
    }
    // TODO(lucasw) do something when the parameter is deleted?
#endif
  }

}  // namespace imgui_ros

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within a launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::spin(std::make_shared<imgui_ros::ImguiRos>());
  rclcpp::shutdown();
  return 0;
}

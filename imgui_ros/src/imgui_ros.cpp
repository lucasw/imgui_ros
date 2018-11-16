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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <imgui.h>
#include <imgui_impl_opengl3.h>
#include <imgui_impl_sdl.h>
#pragma GCC diagnostic pop
#include <imgui_ros/srv/add_window.hpp>
#include <imgui_ros/image.h>
#include <imgui_ros/imgui_ros.h>
#include <imgui_ros/pub.h>
#include <imgui_ros/sub.h>
// #include <opencv2/highgui.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace imgui_ros {
  ImguiRos::ImguiRos() : Node("imgui_ros") {
    // ros init
    // add_window_ = getPrivateNodeHandle().advertiseService("add_window",
    //    &ImguiRos::addWindow, this);
    add_window_ = create_service<srv::AddWindow>("add_window",
        std::bind(&ImguiRos::addWindow, this, _1, _2));

    update_timer_ = this->create_wall_timer(33ms,
        std::bind(&ImguiRos::update, this));
  }

  ImguiRos::~ImguiRos() {
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
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
    SDL_DisplayMode current;
    SDL_GetCurrentDisplayMode(0, &current);
    std::string title = "imgui_ros";
    // ros::param::get("~title", title);
    // TODO(lucasw) window.reset()
    window = SDL_CreateWindow(
        title.c_str(), SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED, 1280, 720,
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
    io = ImGui::GetIO();
    (void)io;
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard
    // Controls

    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init(glsl_version);

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
    // temp test code
    {
      std::string image_file = "";
      ros::param::get("~image", image_file);
      std::shared_ptr<CvImage> image;
      image.reset(new CvImage("test"));
      image->image_ = cv::imread(image_file, CV_LOAD_IMAGE_COLOR);
      image->updateTexture();
      windows_.push_back(image);

      std::shared_ptr<RosImage> ros_image;
      ros_image.reset(new RosImage("test2", "/image_source/image_raw", getNodeHandle));
      windows_.push_back(ros_image);
    }
#endif

    init_ = true;
  }

  void ImguiRos::addWindow(const std::shared_ptr<imgui_ros::srv::AddWindow::Request> req,
      std::shared_ptr<imgui_ros::srv::AddWindow::Response> res) {
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
      std::string message;
      std::shared_ptr<Widget> widget;
      const bool rv = addWidget(req->widgets[i], message, widget);
      res->success = res->success && rv;
      res->message += ", " + message;
      // TODO(lucasw) remove widget if requested
      window->add(widget);
    }
    windows_[req->name] = window;
  }

  // TODO(lucasw) move into widget.cpp?
  bool ImguiRos::addWidget(const imgui_ros::msg::Widget& widget,
      std::string& message, std::shared_ptr<Widget>& imgui_widget) {
    if (widget.type == imgui_ros::msg::Widget::IMAGE) {
      std::shared_ptr<RosImage> ros_image;
      ros_image.reset(new RosImage(widget.name, widget.topic, shared_from_this()));
      imgui_widget = ros_image;
      return true;
    } else if (widget.type == imgui_ros::msg::Widget::PUB) {
      std::shared_ptr<Pub> pub;
      if (widget.sub_type == msg::Widget::FLOAT32) {
        pub.reset(new FloatPub(widget.name, widget.topic,  // widget.sub_type,
            widget.value, widget.min, widget.max, shared_from_this()));
      } else if (widget.sub_type == msg::Widget::BOOL) {
        bool value = widget.value;
        pub.reset(new BoolPub(widget.name, widget.topic,  // widget.sub_type,
            value, shared_from_this()));
      } else if (widget.sub_type == msg::Widget::INT32) {
        int value = widget.value;
        int min = widget.min;
        int max = widget.max;
        pub.reset(new IntPub(widget.name, widget.topic,  // widget.sub_type,
            value, min, max, shared_from_this()));
      } else {
        std::stringstream ss;
        ss << "unsupported window type " << std::dec << widget.sub_type;
        message = ss.str();
        return false;
      }
      imgui_widget = pub;
      return true;
    } else if (widget.type == imgui_ros::msg::Widget::SUB) {
      std::shared_ptr<Sub> sub;
      if (widget.sub_type == msg::Widget::FLOAT32) {
        sub.reset(new FloatSub(widget.name, widget.topic,  // widget.sub_type,
            widget.value, shared_from_this()));
      } else if (widget.sub_type == msg::Widget::BOOL) {
        bool value = widget.value;
        sub.reset(new BoolSub(widget.name, widget.topic,  // widget.sub_type,
            value, shared_from_this()));
      } else if (widget.sub_type == msg::Widget::INT32) {
        int value = widget.value;
        sub.reset(new IntSub(widget.name, widget.topic,  // widget.sub_type,
            value, shared_from_this()));
      } else {
        std::stringstream ss;
        ss << "unsupported window type " << std::dec << widget.sub_type;
        message = ss.str();
        return false;
      }
      imgui_widget = sub;
      return true;

    } else {
      std::stringstream ss;
      // TODO(lucasw) typeToString()
      ss << "unsupported type " << widget.type;
      message = ss.str();
      return false;
    }
    return true;
  }

  void ImguiRos::update() {
    if (!init_)
      glInit();
    // Poll and handle events (inputs, window resize, etc.)
    // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to
    // tell if dear imgui wants to use your inputs.
    // - When io.WantCaptureMouse is true, do not dispatch mouse input data to
    // your main application.
    // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input
    // data to your main application. Generally you may always pass all inputs
    // to dear imgui, and hide them from your application based on those two
    // flags.
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
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
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame(window);
    ImGui::NewFrame();

    {

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

    // Rendering
    ImGui::Render();
    // TODO(lucasw) or wait until after GetDrawData() to unlock?
    }
    SDL_GL_MakeCurrent(window, gl_context);
    glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
    glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    SDL_GL_SwapWindow(window);
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

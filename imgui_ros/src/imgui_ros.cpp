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

#include "imgui.h"
#include "imgui_impl_opengl3.h"
#include "imgui_impl_sdl.h"
#include <imgui_ros/Image.h>
#include <imgui_ros/imgui_ros.h>
// #include <opencv2/highgui.hpp>

// namespace {
  GlImage::GlImage(const std::string name) : name_(name) {
    glGenTextures(1, &texture_id_);
  }

  GlImage::~GlImage() {
    ROS_INFO_STREAM("freeing texture " << texture_id_ << " " << name_);
    glDeleteTextures(1, &texture_id_);
  }

  RosImage::RosImage(const std::string name, const std::string topic,
             ros::NodeHandle& nh) : GlImage(name) {
    ROS_INFO_STREAM("subscribing to topic " << topic);
    sub_ = nh.subscribe(topic, 4, &RosImage::imageCallback, this);
  }

  void RosImage::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    ROS_DEBUG_STREAM("image callback "
        << msg->header.stamp << " "
        << msg->data.size() << " "
        << msg->width << " " << msg->height << ", "
        << sub_.getTopic());
    std::lock_guard<std::mutex> lock(mutex_);
    image_ = msg;
    dirty_ = true;
  }

  // TODO(lucasw) factor this into a generic opengl function to put in parent class
  // if the image changes need to call this
  bool RosImage::updateTexture() {
    sensor_msgs::ImageConstPtr image = image_;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!dirty_)
        return true;
      dirty_ = false;
    }

    if (!image) {
      // TODO(lucasw) or make the texture 0x0 or 1x1 gray.
      return false;
    }

    #if 0
    if (texture_id_ != 0) {
      // if this has happened then probably a crash is going to happen,
      // the memory used to create the texture has been freed?
      ROS_ERROR_STREAM("can't update with non-zero texture_id " << texture_id_);
      return false;
    }
    #endif

    // TODO(lucasw) this is crashing the second time through
    ROS_DEBUG_STREAM("image update " << texture_id_ << " "
        << image->header.stamp << " "
        << image->data.size() << " "
        << image->width << " " << image->height);
    glBindTexture(GL_TEXTURE_2D, texture_id_);

    // TODO(lucasw) only need to do these once (unless altering)
    // TODO(lucasw) make these configurable live
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    // Set texture clamping method - GL_CLAMP isn't defined
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    // Copy the data to the graphics memory.
    // TODO(lucasw) actually look at the image encoding type and
    // have a big switch statement here
    // TODO(lucasw) if the old texture is the same width and height and number of channels
    // (and color format?) as the old one, use glTexSubImage2D
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                 image->width, image->height,
                 0, GL_BGR, GL_UNSIGNED_BYTE, &image->data[0]);

    // one or both of these are causing a crash
    // use fast 4-byte alignment (default anyway) if possible
    // glPixelStorei(GL_UNPACK_ALIGNMENT, (image->step & 3) ? 1 : 4);
    // set length of one complete row in data (doesn't need to equal image.cols)
    // glPixelStorei(GL_UNPACK_ROW_LENGTH, image->step / 1);  // image.elemSize()); TODO(lucasw)

    // ROS_INFO_STREAM(texture_id_ << " " << image.size());
    return true;
  }

  // TODO(lucasw) factor out common code
  void RosImage::draw() {
    // only updates if dirty
    updateTexture();
    ImGui::Begin(name_.c_str());
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (image_ && (texture_id_ != 0)) {
        std::stringstream ss;
        static int count = 0;
        ss << texture_id_ << " " << sub_.getTopic() << " "
            << image_->width << " " << image_->height << " " << count++;
        // const char* text = ss.str().c_str();
        std::string text = ss.str();
        ImGui::Text("%.*s", static_cast<int>(text.size()), text.data());
        ImGui::Image((void*)(intptr_t)texture_id_, ImVec2(image_->width, image_->height));
      }
    }
    ImGui::End();
  }

  CvImage::CvImage(const std::string name) : GlImage(name) {
  }

  // if the image changes need to call this
  bool CvImage::updateTexture() {
    if (!dirty_)
      return true;
    dirty_ = false;

    if (image_.empty()) {
      // TODO(lucasw) or make the texture 0x0 or 1x1 gray.
      return false;
    }

    glBindTexture(GL_TEXTURE_2D, texture_id_);

    // Do these know which texture to use because of the above bind?
    // TODO(lucasw) make these configurable live
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    // Set texture clamping method - GL_CLAMP isn't defined
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    // does this copy the data to the graphics memory?
    // TODO(lucasw) actually look at the image encoding type and
    // have a big switch statement here
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                 image_.cols, image_.rows,
                 0, GL_BGR, GL_UNSIGNED_BYTE, image_.ptr());
    // use fast 4-byte alignment (default anyway) if possible
    glPixelStorei(GL_UNPACK_ALIGNMENT, (image_.step & 3) ? 1 : 4);

    // set length of one complete row in data (doesn't need to equal image.cols)
    glPixelStorei(GL_UNPACK_ROW_LENGTH, image_.step / image_.elemSize());
    ROS_INFO_STREAM(texture_id_ << " " << image_.size());
    return true;
  }

  void CvImage::draw() {
    // only updates if dirty
    updateTexture();
    // TODO(lucasw) another kind of dirty_ - don't redraw if image hasn't changed,
    // window hasn't changed?  How to detect need to redraw window?
    ImGui::Begin(name_.c_str());
    if (!image_.empty() && (texture_id_ != 0)) {
      std::stringstream ss;
      static int count = 0;
      ss << texture_id_ << " " << image_.size() << " " << count++;
      // const char* text = ss.str().c_str();
      std::string text = ss.str();
      ImGui::Text("%.*s", static_cast<int>(text.size()), text.data());
      ImGui::Image((void*)(intptr_t)texture_id_, ImVec2(image_.cols, image_.rows));
    }
    ImGui::End();
  }
// }

///////////////////////////////////////////////////////////////////////////////
namespace imgui_ros {
  ImguiRos::ImguiRos() {
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

  void ImguiRos::onInit() {
    // Setup SDL
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
      ROS_ERROR_STREAM("Error: " << SDL_GetError());
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
    ros::param::get("~title", title);
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
      ROS_ERROR_STREAM("Failed to initialize OpenGL loader!");
      return;
    }

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
      images_.push_back(image);

      std::shared_ptr<RosImage> ros_image;
      ros_image.reset(new RosImage("test2", "/image_source/image_raw", getNodeHandle));
      images_.push_back(ros_image);
    }
#endif

    // ros init
    add_image_ = getPrivateNodeHandle().advertiseService("add_image", &ImguiRos::addImage, this);
    update_timer_ = getPrivateNodeHandle().createTimer(ros::Duration(1.0 / 30.0),
        &ImguiRos::update, this);
  }

  bool ImguiRos::addImage(imgui_ros::Image::Request& req,
                imgui_ros::Image::Response& res) {
    res.success = true;
    if (req.remove) {
      if (images_.count(req.name) > 0) {
        images_.erase(req.name);
      }
      return true;
    }
    std::shared_ptr<RosImage> ros_image;
    ros_image.reset(new RosImage(req.name, req.topic, getPrivateNodeHandle()));
    images_[req.name] = ros_image;
    return true;
  }

  void ImguiRos::update(const ros::TimerEvent& e) {
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
        ros::shutdown();
        return;
      }
      if (event.type == SDL_WINDOWEVENT &&
          event.window.event == SDL_WINDOWEVENT_CLOSE &&
          event.window.windowID == SDL_GetWindowID(window)) {
        ros::shutdown();
        return;
      }
    }

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

      for (auto& image : images_) {
        image.second->draw();
      }
    }

    // Rendering
    ImGui::Render();
    SDL_GL_MakeCurrent(window, gl_context);
    glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
    glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    SDL_GL_SwapWindow(window);
  }
};  // namespace imgui_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(imgui_ros::ImguiRos, nodelet::Nodelet)

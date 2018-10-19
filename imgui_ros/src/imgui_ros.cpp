// dear imgui: standalone example application for SDL2 + OpenGL
// If you are new to dear imgui, see examples/README.txt and documentation at
// the top of imgui.cpp. (SDL is a cross-platform general purpose library for
// handling windows, inputs, OpenGL/Vulkan graphics context creation, etc.)
// (GL3W is a helper library to access OpenGL functions since there is no
// standard header to access modern OpenGL functions easily. Alternatives are
// GLEW, Glad, etc.)

#include "imgui.h"
#include "imgui_impl_opengl3.h"
#include "imgui_impl_sdl.h"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <SDL.h>
#include <stdio.h>

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
  GlImage(const std::string name) : name_(name) {

  }
  ~GlImage() {
    ROS_INFO_STREAM("freeing texture " << texture_id_ << " " << name_);
    free();
  }

  virtual void update() = 0;
  virtual void draw() = 0;

  void free() {
    if (texture_id_ > 0) {
      // TODO(lucasw) or keep the texture id but call glTexImage2D to
      // make a 0x0 or 1x1 image?
      glDeleteTextures(1, &texture_id_);
    }
  }

  void init() {
    free();
    // next get the image into opengl (this stores it in graphics memory?)
    glGenTextures(1, &texture_id_);
  }

  // TODO(lucasw) or NULL or -1?
  GLuint texture_id_ = 0;
  bool dirty_ = true;
  std::string name_ = "";
};

struct RosImage : public GlImage {
  RosImage(const std::string name, const std::string topic,
           ros::NodeHandle& nh) : GlImage(name) {
    ROS_INFO_STREAM("subscribing to topic " << topic);
    sub_ = nh.subscribe(topic, 4, &RosImage::imageCallback, this);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // TODO(lucasw) lock
    image_ = msg;
    dirty_ = true;
  }

  ros::Subscriber sub_;
  sensor_msgs::ImageConstPtr image_;

  // TODO(lucasw) factor this into a generic opengl function to put in parent class
  // if the image changes need to call this
  virtual void update() {
    if (!dirty_)
      return;
    dirty_ = false;

    if (!image_) {
      // TODO(lucasw) or make the texture 0x0 or 1x1 gray.
      return;
    }

    if (texture_id_ == 0) {
      init();
    } else {
      free();
      init();
    }

    // TODO(lucasw) this is crashing the second time through
    ROS_INFO_STREAM("image update " << texture_id_ << " "
        << image_->data.size() << " "
        << image_->width << " " << image_->height);
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
                 image_->width, image_->height,
                 0, GL_BGR, GL_UNSIGNED_BYTE, &image_->data[0]);
    // use fast 4-byte alignment (default anyway) if possible
    glPixelStorei(GL_UNPACK_ALIGNMENT, (image_->step & 3) ? 1 : 4);

    // set length of one complete row in data (doesn't need to equal image.cols)
    glPixelStorei(GL_UNPACK_ROW_LENGTH, image_->step / 1);  // image_.elemSize()); TODO(lucasw)
    // ROS_INFO_STREAM(texture_id_ << " " << image_.size());
  }

  // TODO(lucasw) factor out common code
  virtual void draw() {
    // only updates if dirty
    update();
    ImGui::Begin(name_.c_str());
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
    ImGui::End();
  }
};  // RosImage

struct CvImage : public GlImage {
  CvImage(const std::string name) : GlImage(name) {
  }
  // TODO(lucasw) instead of cv::Mat use a sensor_msgs Image pointer,
  // an convert straight from that format rather than converting to cv.
  // Or just have two implementations of Image here, the cv::Mat
  // would be good to keep as an example.
  cv::Mat image_;

  // if the image changes need to call this
  virtual void update() {
    if (!dirty_)
      return;
    dirty_ = false;

    if (image_.empty()) {
      // TODO(lucasw) or make the texture 0x0 or 1x1 gray.
      return;
    }

    if (texture_id_ == 0) {
      init();
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
  }

  virtual void draw() {
    // only updates if dirty
    update();
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

};

class ImguiRos {
public:
  ImguiRos() {
    init();

    // temp test code
    {
      std::string image_file = "";
      ros::param::get("~image", image_file);
      std::shared_ptr<CvImage> image;
      image.reset(new CvImage("test"));
      image->image_ = cv::imread(image_file, CV_LOAD_IMAGE_COLOR);
      image->update();
      images_.push_back(image);

      std::shared_ptr<RosImage> ros_image;
      ros_image.reset(new RosImage("test2", "/image_source/image_raw", nh_));
      images_.push_back(ros_image);
    }
  }

  ~ImguiRos() {
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();
  }

private:
  bool init() {
    // Setup SDL
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
      printf("Error: %s\n", SDL_GetError());
      return false;
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
    window = SDL_CreateWindow(
        "Dear ImGui SDL2+OpenGL3 example", SDL_WINDOWPOS_CENTERED,
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
      fprintf(stderr, "Failed to initialize OpenGL loader!\n");
      return false;
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

    update_timer_ = nh_.createTimer(ros::Duration(1.0 / 30.0),
        &ImguiRos::update, this);
  }

  void update(const ros::TimerEvent& e) {
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

    // 1. Show the big demo window (Most of the sample code is in
    // ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear
    // ImGui!).
    if (show_demo_window)
      ImGui::ShowDemoWindow(&show_demo_window);

    // 2. Show a simple window that we create ourselves. We use a Begin/End pair
    // to created a named window.
    {
      static float f = 0.0f;
      static int counter = 0;

      ImGui::Begin("Hello, world!"); // Create a window called "Hello, world!"
                                     // and append into it.

      ImGui::Text("This is some useful text."); // Display some text (you can
                                                // use a format strings too)
      ImGui::Checkbox(
          "Demo Window",
          &show_demo_window); // Edit bools storing our window open/close state
      ImGui::Checkbox("Another Window", &show_another_window);

      ImGui::SliderFloat("float", &f, 0.0f,
                         1.0f); // Edit 1 float using a slider from 0.0f to 1.0f
      ImGui::ColorEdit3(
          "clear color",
          (float *)&clear_color); // Edit 3 floats representing a color

      if (ImGui::Button("Button")) // Buttons return true when clicked (most
                                   // widgets return true when edited/activated)
        counter++;
      ImGui::SameLine();
      ImGui::Text("counter = %d", counter);

      ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                  1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
      ImGui::End();

      for (size_t i = 0; i < images_.size(); ++i) {
        images_[i]->draw();
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

  ros::NodeHandle nh_;
  SDL_Window *window;
  ImGuiIO io;
  SDL_GLContext gl_context;
  bool show_demo_window = true;
  bool show_another_window = false;
  ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

  std::vector<std::shared_ptr<GlImage> > images_;

  // TODO(lucasw) still need to update even if ros time is paused
  ros::Timer update_timer_;
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "imgui_ros");
  ImguiRos imgui_ros;
  ros::spin();
}

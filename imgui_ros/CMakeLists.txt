cmake_minimum_required(VERSION 3.5)
project(imgui_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(OpenCV REQUIRED)
find_package(SDL2 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(glm REQUIRED)
# find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(rosidl_typesupport_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Camera.msg"
  "msg/Vertex.msg"
  "msg/Widget.msg"
  "msg/TexturedShape.msg"
  "srv/AddCamera.srv"
  "srv/AddTexture.srv"
  "srv/AddWindow.srv"
  "srv/AddShape.srv"
  "srv/AddShaders.srv"
  DEPENDENCIES geometry_msgs sensor_msgs shape_msgs std_msgs
)

# TODO(lucasw) currently have to symlink imgui in,
# later could make it a subrepo, or git clone it right here
# from cmake.
include_directories(
  include
  imgui
  imgui/examples
  imgui/examples/libs/gl3w
  ${OpenCV_INCLUDE_DIRECTORIES}
  ${SDL2_INCLUDE_DIRS}
)

add_executable(imgui_ros_node
  src/camera.cpp
  src/imgui_ros.cpp
  src/image.cpp
  # src/param.cpp
  src/pub.cpp
  src/sub.cpp
  src/tf.cpp
  src/viz2d.cpp
  src/imgui_impl_opengl3.cpp
  src/viz3d.cpp
  src/window.cpp
  imgui/imgui.cpp
  imgui/imgui_demo.cpp
  imgui/imgui_draw.cpp
  imgui/imgui_widgets.cpp
  imgui/examples/imgui_impl_sdl.cpp
  # imgui/examples/imgui_impl_opengl3.cpp
  imgui/examples/libs/gl3w/GL/gl3w.c
)
set_source_files_properties(imgui/examples/libs/gl3w/GL/gl3w.c PROPERTIES COMPILE_FLAGS -Wno-pedantic)

ament_target_dependencies(imgui_ros_node
  "geometry_msgs"
  "rcl_interfaces"
  "rclcpp"
  "rcutils"
  "shape_msgs"
  "sensor_msgs"
  "std_msgs"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
  "visualization_msgs"
)
rosidl_target_interfaces(imgui_ros_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(imgui_ros_node
  ${OpenCV_LIBRARIES}
  ${SDL2_LIBRARIES}
  GL
  dl
)

install(TARGETS imgui_ros_node DESTINATION lib/${PROJECT_NAME})

add_executable(param_to_topic src/param_to_topic.cpp)
ament_target_dependencies(param_to_topic
  # "example_interfaces"
  "rclcpp"
  "rcutils"
  "std_msgs")
install(TARGETS param_to_topic DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# TODO(lucasw) glob
file(GLOB SCRIPT_FILES scripts/*py)
install(PROGRAMS ${SCRIPT_FILES}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # remove the line when a copyright and license is present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # remove the line when this package is a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.5)
project(imgui_ros2)

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

find_package(SDL2 REQUIRED)
find_package(ament_cmake REQUIRED)
# find_package(example_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(rosidl_typesupport_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddWindow.srv"
)

# TODO(lucasw) currently have to symlink imgui in,
# later could make it a subrepo, or git clone it right here
# from cmake.
include_directories(
  imgui
  imgui/examples
  imgui/examples/libs/gl3w
  include
  ${SDL2_INCLUDE_DIRS}
)

add_executable(imgui_ros
  src/imgui_ros.cpp
  src/image.cpp
  src/pub.cpp
  imgui/imgui.cpp
  imgui/imgui_demo.cpp
  imgui/imgui_draw.cpp
  imgui/imgui_widgets.cpp
  imgui/examples/imgui_impl_sdl.cpp
  imgui/examples/imgui_impl_opengl3.cpp
  imgui/examples/libs/gl3w/GL/gl3w.c
)
set_source_files_properties(imgui/examples/libs/gl3w/GL/gl3w.c PROPERTIES COMPILE_FLAGS -Wno-pedantic)

ament_target_dependencies(imgui_ros
  "rclcpp"
  "rcutils"
  "sensor_msgs"
  "std_msgs")
rosidl_target_interfaces(imgui_ros ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(imgui_ros
  ${SDL2_LIBRARIES}
  GL
  dl
)

install(TARGETS imgui_ros DESTINATION lib/${PROJECT_NAME})

add_executable(param_to_topic src/param_to_topic.cpp)
ament_target_dependencies(param_to_topic
  # "example_interfaces"
  "rclcpp"
  "rcutils"
  "std_msgs")
install(TARGETS param_to_topic DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/demo.py DESTINATION lib/${PROJECT_NAME})

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

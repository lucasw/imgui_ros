cmake_minimum_required(VERSION 3.5)
project(imgui_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(OpenCV REQUIRED)
find_package(SDL2 REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(PCL COMPONENTS common REQUIRED)
find_package(glm REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
cv_bridge
dynamic_reconfigure
geometry_msgs
imgui_ros_msgs
pcl_conversions
roscpp
sensor_msgs
shape_msgs
std_msgs
tf2
tf2_geometry_msgs
tf2_msgs
tf2_ros
visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS nodelet roscpp sensor_msgs std_msgs  # imgui_ros_msgs
)

# TODO(lucasw) currently have to symlink imgui in,
# later could make it a subrepo, or git clone it right here
# from cmake.
include_directories(
  include
  imgui
  imgui/examples
  imgui/examples/libs/gl3w
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRECTORIES}
  ${SDL2_INCLUDE_DIRS}
)

add_library(imgui SHARED
  src/imgui_impl_opengl3.cpp
  imgui/imgui.cpp
  imgui/imgui_demo.cpp
  imgui/imgui_draw.cpp
  imgui/imgui_widgets.cpp
  imgui/examples/imgui_impl_sdl.cpp
  # imgui/examples/imgui_impl_opengl3.cpp
  imgui/examples/libs/gl3w/GL/gl3w.c
)
set_source_files_properties(imgui/examples/libs/gl3w/GL/gl3w.c PROPERTIES COMPILE_FLAGS -Wno-pedantic)

add_library(imguiros SHARED
  src/bag_console.cpp
  src/camera.cpp
  src/cube_camera.cpp
  src/projector.cpp
  src/pub.cpp
  src/shaders.cpp
  src/sub.cpp
  src/surface.cpp
  src/tf.cpp
  src/utility.cpp
  src/viz3d.cpp

  src/dynamic_reconfigure.cpp
  src/graph.cpp
  src/image.cpp
  src/image_transfer.cpp
  src/imgui_ros.cpp
  src/node.cpp
  src/window.cpp
)

if(false)
add_library(imguiros SHARED
  src/param.cpp
  src/point_cloud.cpp
  # TODO(lucasw) maybe there should be a separate test library?
  src/test/generate_pointcloud2.cpp
  src/viz2d.cpp
  # src/imgui_impl_opengl3.cpp
)
endif(false)

target_link_libraries(imguiros imgui
  ${Boost_SYSTEM_LIBRARY}  # pcl_conversions requires this
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${SDL2_LIBRARIES}
  ${catkin_LIBRARIES}
  GL
  dl
  yaml-cpp
)

install(TARGETS imguiros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(imgui_ros_node src/imgui_ros.cpp)
target_link_libraries(imgui_ros_node imguiros imgui)
# install(TARGETS imgui_ros_node DESTINATION lib/${PROJECT_NAME})

# only optionally build usb_cam node, need right version of usb_cam
# https://github.com/lucasw/usb_cam/tree/internal_pub_sub
# find_package(usb_cam)
# if (usb_cam_FOUND_AMENT_PACKAGE)
#   add_executable(usb_cam_viewer src/nodes/usb_cam_viewer.cpp)
#   target_link_libraries(usb_cam_viewer imguiros)
#   ament_target_dependencies(usb_cam_viewer rclcpp imguiros usb_cam)
#   install(TARGETS usb_cam_viewer DESTINATION lib/${PROJECT_NAME})
# endif()

install(DIRECTORY data DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  # TODO(lucasw) is this redundant with install package above, or necessary to run this from ros2 run?
  # imgui_ros/add_shaders.py
  # imgui_ros/cameras.py
  # imgui_ros/demo_imgui_ros.py
  # imgui_ros/demo_gui.py
  # imgui_ros/pub_shape.py
  scripts/cube_camera_fragment.glsl
  scripts/cube_camera_vertex.glsl
  scripts/demo2.py
  scripts/depth_fragment.glsl
  scripts/depth_vertex.glsl
  scripts/fragment.glsl
  # scripts/graph.py
  # scripts/usb_cam_viewer.py
  scripts/vertex.glsl
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY imgui/examples/libs/gl3w/GL/ DESTINATION include/GL)
install(FILES
  imgui/imconfig.h
  imgui/imgui.h
  # imgui/imgui_internal.h
  imgui/imstb_rectpack.h
  imgui/imstb_textedit.h
  imgui/imstb_truetype.h
  DESTINATION include
)

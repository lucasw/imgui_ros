cmake_minimum_required(VERSION 2.8.3)
project(imgui_ros)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  # nodelet
  geometry_msgs
  message_generation
  roscpp
  roslint
  sensor_msgs
  spectrogram_paint_ros
  std_msgs
)

set (CMAKE_CXX_STANDARD 11)

roslint_cpp()

find_package(SDL2 REQUIRED)

# generate_dynamic_reconfigure_options(
#   cfg/Lambda.cfg
# )

# add_message_files(
#   FILES
#   AudioSource.msg
# )

add_service_files(
  FILES
  Image.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS nodelet roscpp sensor_msgs std_msgs
)

include_directories(
  imgui
  imgui/examples
  imgui/examples/libs/gl3w
  include
  ${catkin_INCLUDE_DIRS}
  ${SDL2_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
  src/dynamic_reconfigure.cpp
  src/imgui_ros.cpp
  src/image.cpp
  imgui/imgui.cpp
  imgui/imgui_demo.cpp
  imgui/imgui_draw.cpp
  imgui/imgui_widgets.cpp
  imgui/examples/imgui_impl_sdl.cpp
  imgui/examples/imgui_impl_opengl3.cpp
  imgui/examples/libs/gl3w/GL/gl3w.c
)
# add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
# trailing whitespace is a problem in 16.04
# https://stackoverflow.com/questions/45730098/cmake-leading-or-trailing-whitespace-policy-cmp0004
string(STRIP ${SDL2_LIBRARIES} SDL2_LIBRARIES)
target_link_libraries(${PROJECT_NAME}
  ${SDL2_LIBRARIES}
  ${catkin_LIBRARIES}
  GL
  dl
)

add_executable(${PROJECT_NAME}_node src/imgui_ros_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

# TODO(lucasw) is this needed?
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(FILES
#   nodelet_${PROJECT_NAME}.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

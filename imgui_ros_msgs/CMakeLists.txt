cmake_minimum_required(VERSION 3.5)
project(imgui_ros_msgs)

find_package(catkin REQUIRED COMPONENTS
geometry_msgs
message_generation
sensor_msgs
shape_msgs
std_msgs
tf2_geometry_msgs
tf2_msgs
tf2_ros
visualization_msgs
)

add_message_files(
  FILES
  Camera.msg
  Projector.msg
  Vertex.msg
  Widget.msg
  TexturedShape.msg
  TfWidget.msg
)

add_service_files(
  FILES
  AddCamera.srv
  AddCubeCamera.srv
  AddProjector.srv
  AddTf.srv
  AddTexture.srv
  AddWindow.srv
  AddShape.srv
  AddShaders.srv
)

generate_messages(
  DEPENDENCIES geometry_msgs sensor_msgs shape_msgs std_msgs
)

catkin_package(
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_generation sensor_msgs std_msgs
)

install(DIRECTORY include/std_msgs/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN "*.h")

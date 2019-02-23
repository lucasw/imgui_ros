cmake_minimum_required(VERSION 3.5)
project(internal_pub_sub)

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

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rcutils)
find_package(rcl_interfaces REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NodeSettings.msg"
  "msg/Remapping.msg"
  "msg/Republisher.msg"
  "srv/AddNode.srv"
  "srv/AddRepublisher.srv"
  DEPENDENCIES rcl_interfaces sensor_msgs std_msgs
)

include_directories(
  include
)

add_library(internalpubsub SHARED
  src/internal_pub_sub.cpp
)
ament_target_dependencies(internalpubsub
  "rcl_interfaces"
  "rclcpp"
  "rcutils"
  "sensor_msgs"
  "std_msgs"
)
rosidl_target_interfaces(internalpubsub ${PROJECT_NAME} "rosidl_typesupport_cpp")
install(TARGETS internalpubsub
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(node_loader src/node_loader.cpp)
ament_target_dependencies(node_loader
  "ament_index_cpp"
  "class_loader"
  "rcl_interfaces"
  "rclcpp"
  "rcutils"
  "sensor_msgs"
  "std_msgs"
)
target_link_libraries(node_loader internalpubsub)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_link_libraries(node_loader "stdc++fs")
endif()
install(TARGETS node_loader DESTINATION lib/${PROJECT_NAME})

add_executable(demo_internal_pub_sub src/demo_internal_pub_sub.cpp)
ament_target_dependencies(demo_internal_pub_sub
  "rcl_interfaces"
  "rclcpp"
  "rcutils"
  "sensor_msgs"
  "std_msgs"
)
target_link_libraries(demo_internal_pub_sub internalpubsub)
install(TARGETS demo_internal_pub_sub DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(internal_pub_sub)

install(PROGRAMS
  scripts/demo_add_node.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

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

ament_export_include_directories(
  include
)
ament_export_libraries(internalpubsub)

ament_package()

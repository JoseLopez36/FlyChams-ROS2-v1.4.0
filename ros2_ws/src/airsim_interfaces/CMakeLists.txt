cmake_minimum_required(VERSION 3.5)

project(airsim_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Global services and messages
  "srv/Reset.srv"
  "srv/Run.srv"
  "srv/Pause.srv"
  # Vehicle services and messages
  "msg/CameraFovCmd.msg"
  "msg/GimbalAngleCmd.msg"
  # Window services and messages
  "msg/WindowImageCmdGroup.msg"
  "msg/WindowRectangleCmd.msg"
  "msg/WindowStringCmd.msg"
  # Tracking services and messages
  "srv/AddTargetGroup.srv"
  "srv/AddClusterGroup.srv"
  "msg/UpdateTargetCmdGroup.msg"
  "msg/UpdateClusterCmdGroup.msg"
  DEPENDENCIES std_msgs geometry_msgs LIBRARY_NAME ${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

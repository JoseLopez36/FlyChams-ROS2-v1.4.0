cmake_minimum_required(VERSION 3.8)
project(flychams_interfaces)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  # Basic messages
  "msg/Element.msg"
  "msg/Crop.msg"
  # Global messages
  "msg/Registration.msg"
  "msg/GlobalMetrics.msg"
  # Agent messages
  "msg/AgentStatus.msg"
  "msg/AgentClusters.msg"
  "msg/AgentHeadSetpoints.msg"
  "msg/AgentWindowSetpoints.msg"
  "msg/AgentMetrics.msg"
  # Target messages
  "msg/TargetMetrics.msg"
  # Cluster messages
  "msg/ClusterGeometry.msg"
  "msg/ClusterMetrics.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

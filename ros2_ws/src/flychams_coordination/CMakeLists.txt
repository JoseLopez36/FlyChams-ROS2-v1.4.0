cmake_minimum_required(VERSION 3.8)
project(flychams_coordination)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(flychams_core REQUIRED)

# Add external libraries
add_subdirectory(external/nlopt ${CMAKE_CURRENT_BINARY_DIR}/nlopt_build)

# Create coordination library
set(COORD_SRC
  src/assignment/agent_assignment.cpp
  src/assignment/assignment_solver.cpp
  src/analysis/agent_analysis.cpp
  src/positioning/agent_positioning.cpp
  src/positioning/position_solver.cpp
  src/tracking/agent_tracking.cpp
)
add_library(${PROJECT_NAME} SHARED ${COORD_SRC})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Export dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp flychams_core
)

# Link to required libraries
target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  nlopt
)

# Create executable for agent assignment node
add_executable(agent_assignment_node src/agent_assignment_node.cpp)
target_link_libraries(agent_assignment_node ${PROJECT_NAME})
ament_target_dependencies(agent_assignment_node rclcpp flychams_core)

# Create executable for agent analysis node
add_executable(agent_analysis_node src/agent_analysis_node.cpp)
target_link_libraries(agent_analysis_node ${PROJECT_NAME})
ament_target_dependencies(agent_analysis_node rclcpp flychams_core)

# Create executable for agent positioning node
add_executable(agent_positioning_node src/agent_positioning_node.cpp)
target_link_libraries(agent_positioning_node ${PROJECT_NAME})
ament_target_dependencies(agent_positioning_node rclcpp flychams_core)

# Create executable for agent tracking node
add_executable(agent_tracking_node src/agent_tracking_node.cpp)
target_link_libraries(agent_tracking_node ${PROJECT_NAME})
ament_target_dependencies(agent_tracking_node rclcpp flychams_core)

# Install targets and headers
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install and export NLopt
install(TARGETS nlopt
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS agent_assignment_node agent_analysis_node agent_positioning_node agent_tracking_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Export package
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  flychams_core
)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_package()
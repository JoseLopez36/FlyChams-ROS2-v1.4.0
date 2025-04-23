cmake_minimum_required(VERSION 3.8)
project(flychams_perception)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(flychams_core REQUIRED)

# Create perception library
set(PERCEPTION_SRC
  src/clustering/target_clustering.cpp
  src/analysis/cluster_analysis.cpp
  external/hungarian-algorithm-cpp/Hungarian.cpp
)
add_library(${PROJECT_NAME} SHARED ${PERCEPTION_SRC})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/hungarian-algorithm-cpp>
  $<INSTALL_INTERFACE:include>
)

# Export dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp flychams_core
)

# Link to required libraries
target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
)

# Create executable for target clustering node
add_executable(target_clustering_node src/target_clustering_node.cpp)
target_link_libraries(target_clustering_node ${PROJECT_NAME})
ament_target_dependencies(target_clustering_node rclcpp flychams_core)

# Create executable for cluster analysis node
add_executable(cluster_analysis_node src/cluster_analysis_node.cpp)
target_link_libraries(cluster_analysis_node ${PROJECT_NAME})
ament_target_dependencies(cluster_analysis_node rclcpp flychams_core)

# Install targets and headers
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS target_clustering_node cluster_analysis_node
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
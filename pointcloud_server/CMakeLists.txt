cmake_minimum_required(VERSION 3.8)
project(pointcloud_server)

# Set C++ standard to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(pointcloud_server_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(OpenMP REQUIRED)

# Locate nanoflann header
find_path(NANOFLLAN_INCLUDE_DIR nanoflann.hpp)
if(NOT NANOFLLAN_INCLUDE_DIR)
  message(FATAL_ERROR "nanoflann.hpp not found. Please install libnanoflann-dev.")
endif()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${NANOFLLAN_INCLUDE_DIR}
)

# -------------------------------------------------------------------
# 1) pointcloud_server_node as library (component) + executable
# -------------------------------------------------------------------
add_library(pointcloud_server_component SHARED
  src/node.cpp
  src/RollingGrid.cpp
  src/Utilities.cpp
)
ament_target_dependencies(pointcloud_server_component
  rclcpp
  rclcpp_components
  pointcloud_server_interfaces
  Eigen3
  pcl_conversions
  pcl_ros
)
target_link_libraries(pointcloud_server_component
  ${PCL_LIBRARIES}
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(pointcloud_server_component OpenMP::OpenMP_CXX)
endif()

# Register node(s) in node.cpp
rclcpp_components_register_nodes(pointcloud_server_component
  "pointcloud_server::PointcloudServerNode"
)

# The standalone executable main
add_executable(pointcloud_server_node
  src/main.cpp
)

target_link_libraries(pointcloud_server_node
  pointcloud_server_component
)
install(TARGETS pointcloud_server_node
  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------------------------------------------------
# 2) filter_node as library (component) + executable
# -------------------------------------------------------------------
add_library(filter_node_component SHARED
  src/filter_node.cpp
)
ament_target_dependencies(filter_node_component
  rclcpp
  rclcpp_components
  pcl_ros
  pcl_conversions
  tf2_ros
  tf2_eigen
)
target_link_libraries(filter_node_component
  ${PCL_LIBRARIES}
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(filter_node_component OpenMP::OpenMP_CXX)
endif()

rclcpp_components_register_nodes(filter_node_component
  "pointcloud_server::FilterNode"
)

add_executable(filter_node
  src/main_filter_node.cpp
)
target_link_libraries(filter_node
  filter_node_component
)
install(TARGETS filter_node
  DESTINATION lib/${PROJECT_NAME}
)

# -------------------------------------------------------------------
# 3) freespace_detection_node as library (component) + executable
# -------------------------------------------------------------------
add_library(freespace_detection_component SHARED
  src/freespace_detection_node.cpp
)
ament_target_dependencies(freespace_detection_component
  rclcpp
  rclcpp_components
  pcl_ros
  pcl_conversions
  tf2_ros
  tf2_eigen
)
target_link_libraries(freespace_detection_component
  ${PCL_LIBRARIES}
)

add_executable(freespace_detection_node
  src/main_freespace_detection_node.cpp
)
target_link_libraries(freespace_detection_node
  freespace_detection_component
)
install(TARGETS freespace_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

# Register Freedetection
rclcpp_components_register_nodes(freespace_detection_component
  "freespace_detection::FreespaceDetectionNode"
)

# -------------------------------------------------------------------
# Common Installs
# -------------------------------------------------------------------
install(TARGETS
  pointcloud_server_component
  filter_node_component
  freespace_detection_component
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

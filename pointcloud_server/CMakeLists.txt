cmake_minimum_required(VERSION 3.8)
project(pointcloud_server)

# Set C++ standard to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(pointcloud_server_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
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

# Build the main executable
add_executable(pointcloud_server_node
  src/main.cpp
  src/node.cpp
  src/RollingGrid.cpp
  src/Utilities.cpp
)

add_executable(filter_node
  src/main_filter_node.cpp
  src/filter_node.cpp
)

ament_target_dependencies(pointcloud_server_node
  rclcpp
  pointcloud_server_interfaces
  Eigen3
  pcl_conversions
  pcl_ros
)

target_link_libraries(pointcloud_server_node
  ${PCL_LIBRARIES}
)

ament_target_dependencies(filter_node
  rclcpp
  pcl_ros
  pcl_conversions
  tf2_ros
  tf2_eigen
)

target_link_libraries(filter_node
  ${PCL_LIBRARIES}
)

if(OpenMP_CXX_FOUND)
  target_link_libraries(pointcloud_server_node OpenMP::OpenMP_CXX)
  target_link_libraries(filter_node OpenMP::OpenMP_CXX)
endif()

install(TARGETS pointcloud_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS filter_node
  DESTINATION lib/${PROJECT_NAME}
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

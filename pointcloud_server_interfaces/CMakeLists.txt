cmake_minimum_required(VERSION 3.8)
project(pointcloud_server_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
)

set(srv_files
  "srv/Add.srv"
  "srv/BuildKdTree.srv"
  "srv/BuildSubMap.srv"
  "srv/ClearPoints.srv"
  "srv/Clear.srv"
  "srv/EmptyAroundPoint.srv"
  "srv/Get.srv"
  "srv/GetSubMap.srv"
  "srv/KnnSearch.srv"
  "srv/LabelNewPoints.srv"
  "srv/Reset.srv"
  "srv/Roll.srv"
  "srv/Save.srv"
  "srv/SetGridSize.srv"
  "srv/SetLeafSize.srv"
  "srv/SetVoxelResolution.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  ${msg_files}
  DEPENDENCIES sensor_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

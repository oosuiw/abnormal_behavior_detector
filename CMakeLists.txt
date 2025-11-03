cmake_minimum_required(VERSION 3.14)
project(abnormal_behavior_detector)

# Generate custom messages first
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(rosidl_generate_interfaces_SKIP_INSTALL TRUE)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AbnormalBehaviorStatus.msg"
  "msg/ObjectDebugInfo.msg"
  DEPENDENCIES
    std_msgs
    geometry_msgs
  LIBRARY_NAME ${PROJECT_NAME}
)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Dependencies
find_package(Eigen3 REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_extension REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Abnormal Behavior Detector Node
ament_auto_add_library(abnormal_behavior_detector_node SHARED
  src/abnormal_behavior_detector_node.cpp
)

target_link_libraries(abnormal_behavior_detector_node
  ${lanelet2_core_LIBRARIES}
  ${lanelet2_extension_LIBRARIES}
)

# Link custom messages
rosidl_target_interfaces(abnormal_behavior_detector_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rclcpp_components_register_node(abnormal_behavior_detector_node
  PLUGIN "abnormal_behavior_detector::AbnormalBehaviorDetectorNode"
  EXECUTABLE abnormal_behavior_detector_exe
)

# Install
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)

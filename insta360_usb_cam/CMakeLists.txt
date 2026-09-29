cmake_minimum_required(VERSION 3.10)
project(insta360_usb_cam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 의존성
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/InstaPanTiltMsgs.msg"
  "msg/PanTiltStatusMsgs.msg"
  DEPENDENCIES std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 실행 파일
# ============================================================
add_executable(pan_tilt_camera_node
  src/usb_cam/insta/pan_tilt_camera_node.cpp
  src/usb_cam/insta/usb_camera.cpp)

ament_target_dependencies(pan_tilt_camera_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  sensor_msgs
  std_msgs
  cv_bridge
)

target_link_libraries(pan_tilt_camera_node
  ${OpenCV_LIBRARIES}
  "${cpp_typesupport_target}"
)

add_dependencies(pan_tilt_camera_node ${PROJECT_NAME})

install(TARGETS
  pan_tilt_camera_node
  DESTINATION lib/${PROJECT_NAME}
)
# ============================================================

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

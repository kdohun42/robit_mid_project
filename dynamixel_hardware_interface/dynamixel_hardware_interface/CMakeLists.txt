cmake_minimum_required(VERSION 3.8)
project(dynamixel_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_hardware_msgs REQUIRED)

# Executable
add_executable(dynamixel_hardware_interface_node
  src/dynamixel_hardware_interface.cpp
  src/motor_setting.cpp
  src/motor_status.cpp
)

target_include_directories(dynamixel_hardware_interface_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dynamixel_hardware_interface_node
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  dynamixel_sdk
  dynamixel_hardware_msgs
)

# Executable install
install(
  TARGETS dynamixel_hardware_interface_node
  DESTINATION lib/${PROJECT_NAME}
)

# Header install
install(
  DIRECTORY include/
  DESTINATION include
)

# Config and launch install
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

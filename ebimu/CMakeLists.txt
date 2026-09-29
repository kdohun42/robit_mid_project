cmake_minimum_required(VERSION 3.8)
project(ebimu)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTOMOC OFF)
set(CMAKE_AUTORCC ON)

#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(rclcpp REQUIRED)
find_package(Boost 1.71.0 REQUIRED COMPONENTS thread)
find_package(Qt5 COMPONENTS Widgets Core Gui Network REQUIRED)
find_package(humanoid_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_interfaces REQUIRED)
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(
  rclcpp
  REQUIRED
)

# Qt GUI 빌드용 소스 (e2box_imu_node.cpp은 main() 없이 포함)
set(PRO_SOURCE_DIR src/main.cpp src/main_window.cpp src/qnode.cpp)
file(GLOB PRO_UI_DIR RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  ui/*.ui)
file(GLOB PRO_INCLUDE_DIR RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  include/${PROJECT_NAME}/*.h include/${PROJECT_NAME}/*.hpp)
file(GLOB PRO_RESOURCE_DIR RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  resource/*.qrc)
qt5_wrap_ui(QT_UI_HPP ${PRO_UI_DIR})
qt5_wrap_cpp(QT_MOC_HPP ${PRO_INCLUDE_DIR})
qt5_add_resources(QT_RESOUCE_HPP ${PRO_RESOURCE_DIR})
include_directories(
	include/
	include/ebimu/
	include/ebimu/e2box_imu_node/
	include/ebimu/serial_manager/
	${CMAKE_CURRENT_BINARY_DIR}
  ${humanoid_interfaces_INCLUDE_DIRS}
)
add_executable(ebimu
    ${PRO_SOURCE_DIR}
    ${PRO_INCLUDE_DIR}
    ${PRO_UI_DIR}
    ${QT_MOC_HPP}
    ${QT_RESOUCE_HPP}
    ${QT_UI_HPP}
    src/e2box_imu_node/e2box_imu_node.cpp
    src/serial_manager/serial_manager.cpp
  )
ament_target_dependencies(ebimu rclcpp Boost humanoid_interfaces std_msgs sensor_msgs vision_interfaces)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets Qt5::Core Qt5::Gui Qt5::Network ${OpenCV_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

# Qt 없는 순수 ROS2 노드
add_executable(e2box_imu_node
    src/e2box_imu_node/e2box_imu_node.cpp
    src/serial_manager/serial_manager.cpp
)
ament_target_dependencies(e2box_imu_node rclcpp Boost humanoid_interfaces std_msgs sensor_msgs vision_interfaces)
target_compile_definitions(e2box_imu_node PRIVATE BUILD_STANDALONE)
install(TARGETS e2box_imu_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install the executables
install(TARGETS
  ebimu
  DESTINATION lib/${PROJECT_NAME})

ament_package()

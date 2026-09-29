cmake_minimum_required(VERSION 3.5)
project(gamecontroller)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC OFF)
set(CMAKE_AUTOMOC OFF)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost 1.71.0 REQUIRED COMPONENTS thread)
find_package(Qt5 COMPONENTS Widgets Core Gui Network REQUIRED)
find_package(std_msgs REQUIRED)
find_package(humanoid_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(
  rclcpp
  REQUIRED
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Gamecontroldata.msg"
  "msg/Gamecontrolreturndata.msg"
  "msg/Robocupcontroller.msg"
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

file(GLOB PRO_SOURCE_DIR RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  src/*.cpp)
file(GLOB PRO_UI_DIR RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  ui/*.ui)
file(GLOB PRO_INCLUDE_DIR RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  include/${PROJECT_NAME}/*.h include/${PROJECT_NAME}/*.hpp)
file(GLOB PRO_RESOURCE_DIR RELATIVE ${CMAKE_SOURCE_DIR} FOLLOW_SYMLINKS  resource/*.qrc)
qt5_wrap_ui(QT_UI_HPP ${PRO_UI_DIR})
qt5_wrap_cpp(QT_MOC_HPP ${PRO_INCLUDE_DIR})
qt5_add_resources(QT_RESOUCE_HPP ${PRO_RESOURCE_DIR})
include_directories(
	include/
	${CMAKE_CURRENT_BINARY_DIR}
)
set(EXE_NAME ${PROJECT_NAME}_app)
add_executable(${EXE_NAME}
    ${PRO_SOURCE_DIR}
    ${PRO_INCLUDE_DIR}
    ${PRO_UI_DIR}
    ${QT_MOC_HPP}
    ${QT_RESOUCE_HPP}
    ${QT_UI_HPP}
  )
ament_target_dependencies(${EXE_NAME} rclcpp Boost std_msgs humanoid_interfaces)
target_link_libraries(${EXE_NAME} Qt5::Widgets Qt5::Core Qt5::Gui Qt5::Network ${OpenCV_LIBRARIES} ${cpp_typesupport_target})

# GC 출력을 로봇 N대로 나누는 노드. Qt 앱과 무관해서 위 glob(src/*.cpp)에
# 걸리지 않도록 src/fanout/ 아래에 둔다 — 같이 걸리면 main 이 충돌한다.
set(FANOUT_EXE_NAME gc_fanout_node)
add_executable(${FANOUT_EXE_NAME}
    src/fanout/gc_fanout_node.cpp
    src/fanout/gc_fanout_sim.cpp
  )
ament_target_dependencies(${FANOUT_EXE_NAME} rclcpp)
target_link_libraries(${FANOUT_EXE_NAME} ${cpp_typesupport_target})

install(TARGETS ${EXE_NAME} ${FANOUT_EXE_NAME} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()

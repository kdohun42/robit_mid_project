cmake_minimum_required(VERSION 3.8)
project(humanoid_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}

  "msg/IkAngleSimMsg.msg"
  "msg/IkComMsg.msg"
  "msg/IkCoordMsg.msg"
  "msg/IkEndMsg.msg"
  "msg/IkLTCMsg.msg"
  "msg/IkPatternMsg.msg"

  "msg/ImuMsg.msg"
  "msg/Master2IkMsg.msg"
  "msg/Tune2IkMsg.msg"
  "msg/ZmpMsg.msg"

  "msg/Master2vision.msg"

  "msg/Master2localization.msg"

  "msg/Master2udp.msg"
  "msg/Udp2master.msg"


  "msg/MotionOperator.msg"
  
  "msg/Pidtuning.msg"

  DEPENDENCIES std_msgs
  DEPENDENCIES geometry_msgs
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)


  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

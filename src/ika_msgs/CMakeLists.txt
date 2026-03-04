cmake_minimum_required(VERSION 3.8)
project(ika_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/VehicleState.msg"
  "msg/SafetyStatus.msg"
  "msg/SystemMode.msg"
  "msg/StageInfo.msg"
  "msg/Cone.msg"
  "msg/ConeArray.msg"
  "msg/TargetDetection.msg"
  "msg/SlidingObstacle.msg"
)

set(srv_files
  "srv/SetMode.srv"
  "srv/FireLaser.srv"
  "srv/SkipStage.srv"
)

set(action_files
  "action/ExecuteStage.action"
  "action/AutoAim.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
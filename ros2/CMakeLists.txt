cmake_minimum_required(VERSION 3.8)
project(gr0x_motor_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# yaml-cpp for joints.yaml loading
find_package(yaml-cpp REQUIRED)

# gr0x motor library (built separately in ~/gr0x-motor/build)
set(GR0X_MOTOR_DIR /home/gr0x-pi/gr0x-motor)

add_executable(gr0x_motor_node
  src/gr0x_motor_node.cpp
)

target_include_directories(gr0x_motor_node PRIVATE
  ${GR0X_MOTOR_DIR}/include
)

target_link_libraries(gr0x_motor_node
  ${GR0X_MOTOR_DIR}/build/libgr0x_motor.a
  yaml-cpp
)

ament_target_dependencies(gr0x_motor_node
  rclcpp
  sensor_msgs
)

install(TARGETS gr0x_motor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY ${GR0X_MOTOR_DIR}/config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
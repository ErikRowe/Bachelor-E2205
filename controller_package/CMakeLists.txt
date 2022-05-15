cmake_minimum_required(VERSION 3.8)
project(controller_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(controller_node
  src/controller_node.cpp
  src/joy_to_action.cpp
  src/controller.cpp
  src/setpoint_holder.cpp
  src/logging.cpp
)

set(dependencies
    rclcpp
    eigen3_cmake_module
    Eigen3
    sensor_msgs
    nav_msgs
)
ament_target_dependencies(controller_node ${dependencies})
ament_export_dependencies(${dependencies})

target_include_directories(controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(controller_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  controller_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})
  install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME})
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

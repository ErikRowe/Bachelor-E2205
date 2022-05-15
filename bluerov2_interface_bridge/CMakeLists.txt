cmake_minimum_required(VERSION 3.8)
project(bluerov2_interface_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(bluerov_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

add_executable(bridge_node
  src/bluerov2_interface_node.cpp
)

set(dependencies
    rclcpp
    eigen3_cmake_module
    Eigen3
    geometry_msgs
    bluerov_interfaces
)
ament_target_dependencies(bridge_node ${dependencies})
ament_export_dependencies(${dependencies})

target_include_directories(bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(bridge_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
bridge_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

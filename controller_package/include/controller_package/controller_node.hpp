#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdio>


// Ros includes, these need to be included in dependencies
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

// Control group includes
#include "controller_package/common.hpp"
#include "bluerov_interfaces/msg/actuator_input.hpp"
#include "controller_package/joy_to_action.h"
#include "controller_package/controller_complete.h"
#include "controller_package/control_actuator.hpp"


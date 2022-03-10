
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdio>

// Ros includes, these need to be included in dependencies
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>

// Control group includes
#include "bluerov_interfaces/msg/actuator_input.hpp"
#include "controller_package/controller_ros2.h"
#include "controller_package/control_actuator.hpp"



using std::placeholders::_1;
using namespace std::chrono_literals;

class ControlNode : public rclcpp::Node
{
private:
  // Private variables
  // double linear_move[3], angular_move[3]; // These variables are each arrays, containing 3 numbers representing linear or angular movement

  // Private functions
  void joystick_callback(const sensor_msgs::msg::Joy::SharedPtr msg) //const
  {
    /**
     * @brief Callback for the joystick reading
     * 
     * TODO: make code to determine what input from the controller is sendt to controller input
     * 
     */
    
    
  }

  // Private ROS2 declerations
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<bluerov_interfaces::msg::ActuatorInput >::SharedPtr act_pub_;
  Actuation actuation_; // Calls actuation class as an object
  rclcpp::Clock clock_;

  
public:
  // double test; // test variable
  // Actuator stuff
    bluerov_interfaces::msg::ActuatorInput actuation_message_;

  ControlNode()
  : Node("Control_Node")
  {
    /**
     * @brief stuff
     * 
     * TODO: Make a subscriber in the same node to handle actuation data that is to be sendt out from the node.
     * 
     */
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joystick_topic", 10, std::bind(&ControlNode::joystick_callback, this, _1));
    act_pub_ = this->create_publisher<bluerov_interfaces::msg::ActuatorInput>("/control/actuation", 10);
  }
  void send_actuation(Eigen::vector6d tau)
  {
    Eigen::Vector8d thrusters_ = actuation_.build_actuation(tau);
    // Actuator stuff
    actuation_message_.header.stamp = clock_.now(); // This needs to be time now. is ros format
    actuation_message_.thrust1 = thrusters_(0);
    actuation_message_.thrust2 = thrusters_(1);
    actuation_message_.thrust3 = thrusters_(2);
    actuation_message_.thrust4 = thrusters_(3);
    actuation_message_.thrust5 = thrusters_(4);
    actuation_message_.thrust6 = thrusters_(5);
    actuation_message_.thrust7 = thrusters_(6);
    actuation_message_.thrust8 = thrusters_(7);
    act_pub_->publish(actuation_message_);
  }
};



// Main initiates the node, and keeps it alive
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}

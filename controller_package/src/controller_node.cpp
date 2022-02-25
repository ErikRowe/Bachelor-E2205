
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

// Control group includes
#include "controller_package/controller_ros2.h"
//#include "controller_package/control_actuator.h"
#include "controller_package/control_input.h"
#include "controller_package/joy_to_action.h"


using std::placeholders::_1;
using namespace std::chrono_literals;

class ControlNode : public rclcpp::Node
{
private:
  // Private variables
  // double linear_move[3], angular_move[3]; // These variables are each arrays, containing 3 numbers representing linear or angular movement
  // Private functions

  void joystick_callback(const sensor_msgs::msg::Joy msg) //const
  {
    /**
     * @brief Callback for the joystick reading
     * 
     * TODO: make code to determine what input from the controller is sendt to controller input
     * 
     */
    double *actions;
    actions = joystickInputClass.joystickToActions(msg.axes, msg.buttons);
    PIDInput.changeSetPoint(actions);
    RCLCPP_INFO(this->get_logger(), "Test: '%f'", actions[0]);
    RCLCPP_INFO(this->get_logger(), "Surge: '%f'", PIDInput.x_d[0]);
    RCLCPP_INFO(this->get_logger(), "Sway: '%f'", PIDInput.x_d[1]);
    RCLCPP_INFO(this->get_logger(), "Heave: '%f'", PIDInput.x_d[2]);
    
    
  }

  void position_callback(const geometry_msgs::msg::Point msg){

    PIDInput.readPosAtt(msg.x, msg.y, msg.z);
  }

  // Private ROS2 declerations
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr pos_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr act_pub_;
  //Actuation actuation_object; // Calls actuation code as an object
  
public:
  double test; // test variable
  UserJoystickInput joystickInputClass;
  PIDInputClass PIDInput;
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
      "joy", 10, std::bind(&ControlNode::joystick_callback, this, _1));
    pos_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "position", 10, std::bind(&ControlNode::position_callback, this, _1));
    act_pub_ = this->create_publisher<std_msgs::msg::String>("/control/actuation", 10);

    // START teststuff
    //std::array<double, 6> data = {1,2,3,4,5,6};
    //test = actuation_object.actuation(data);
    //std::cout << test;
    // END teststuff
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

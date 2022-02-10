
#include <functional>
#include <memory>
#include <cstdio>

// Ros includes
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// Control group includes
#include "controller_package/control_actuator.h"


using std::placeholders::_1;

class ControlNode : public rclcpp::Node
{
private:
  void joystick_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    /**
     * @brief Callback for the joystick reading
     * 
     * TODO: make code to determine what input from the controller is sendt to controller input
     * 
     */
    printf("got callback. This will only be printed at sysexit")
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  
public:
  ControlNode()
  : Node("Control_Node")
  {
    /**
     * @brief stuff
     * 
     * TODO: Make a subscriber in the same node to handle actuation data that is to be sendt out from the node.
     * 
     */
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "joystick_topic", 10, std::bind(&ControlNode::joystick_callback, this, _1)
    §);
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

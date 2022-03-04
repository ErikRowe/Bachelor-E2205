#include <eigen3/Eigen/Dense>
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
  // Private functions

  void joystick_callback(const sensor_msgs::msg::Joy msg) //const
  {
    /**
     * @brief Callback for the joystick reading
     * 
     * TODO: make code to determine what input from the controller is sendt to controller input
     * 
     */
    joystickInputClass.joystickToActions(msg.axes, msg.buttons);
    PIDInput.changeSetPoint(joystickInputClass.actions);
    
  }

  void estimate_callback(const nav_msgs::msg::Odometry msg){
    auto pos = msg.pose.pose.position;
    auto att = msg.pose.pose.orientation;
    auto lin = msg.twist.twist.linear;
    auto ang = msg.twist.twist.angular;
    Eigen::Vector3d x = Eigen::Vector3d(pos.x, pos.y, pos.z);
    Eigen::Quaterniond q = Eigen::Quaterniond(att.w, att.x, att.y, att.z);
    Eigen::Vector6d velocity = Eigen::Vector6d(lin.x, lin.y, lin.z, ang.x, ang.y, ang.z);
    PIDInput.updateGlobalParameters(x, q, velocity);
  }

  void reference_publisher(){
    auto message = geometry_msgs::msg::PoseStamped();
    message.header.stamp = clock_.now();
    message.header.frame_id = "map";
    message.pose.position.x = PIDInput.x_d[0];
    message.pose.position.y = PIDInput.x_d[1];
    message.pose.position.z = PIDInput.x_d[2];
    message.pose.orientation.w = PIDInput.q_d.w();
    message.pose.orientation.x = PIDInput.q_d.x();
    message.pose.orientation.y = PIDInput.q_d.y();
    message.pose.orientation.z = PIDInput.q_d.z();
    ref_pub_->publish(message);
  }

  // Private ROS2 declerations
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_estim_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ref_pub_;
  //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr act_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock clock_;
  
public:
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
    state_estim_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "state_estimate", 10, std::bind(&ControlNode::estimate_callback, this, _1));

    ref_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/reference/pose", 10);
    //act_pub_ = this->create_publisher<std_msgs::msg::String>("/actuation", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&ControlNode::reference_publisher, this));
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

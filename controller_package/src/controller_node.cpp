#include "controller_package/controller_node.hpp"

ControlNode::ControlNode ()
: Node("Control_Node")
{
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&ControlNode::joystick_callback, this, _1));
    //state_estim_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //  "state_estimate", 10, std::bind(&ControlNode::estimate_callback, this, _1));

    ref_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/reference/pose", 10);
    act_pub_ = this->create_publisher<bluerov_interfaces::msg::ActuatorInput>("/actuation", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&ControlNode::reference_publisher, this));
    PIDTimer_ = this->create_wall_timer(30ms, std::bind(&ControlNode::sample_PID, this));
}

void ControlNode::send_actuation(Eigen::Vector6d tau)
{
  Eigen::Vector8d thrusters_ = actuation_.build_actuation(tau);
  actuation_message_.header.stamp = clock_.now();
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

void ControlNode::joystick_callback(const sensor_msgs::msg::Joy msg)
{
    joystick_handler_.joystickToActions(msg.axes, msg.buttons);
    reference_handler_.update_setpoint(&joystick_handler_.movement, &joystick_handler_.active_buttons, q, x);
    //auto outputVar = (int)joystick_handler_.active_buttons[0];
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%i'", outputVar);
}

void ControlNode::moveEntity(Eigen::Vector6d tau)
{
    double dt = 0.030;
    
    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector3d v_lin = R * Eigen::Vector3d(tau[0], tau[1], tau[2]) * dt;
    Eigen::Vector3d v_ang = R * Eigen::Vector3d(tau[3], tau[4], tau[5]) * dt;
    Eigen::Quaterniond q_new;
    Eigen::Vector3d x_new;
    x_new << x[0] + v_lin[0], x[1] + v_lin[1], x[2] + v_lin[2];

    Eigen::Quaterniond q_relativeChange;
    q_relativeChange = Eigen::AngleAxisd(v_ang[0], Eigen::Vector3d::UnitX())
                    *Eigen::AngleAxisd(v_ang[1], Eigen::Vector3d::UnitY())
                    *Eigen::AngleAxisd(v_ang[2], Eigen::Vector3d::UnitZ());
    q_relativeChange.normalize();
    q_new = q_relativeChange * q;
    q.normalize();
    q = q_new;
    x = x_new;
}

void ControlNode::reference_publisher()
{
    auto message = geometry_msgs::msg::PoseStamped();
    message.header.stamp = clock_.now();
    message.header.frame_id = "map";
    message.pose.position.x = x[0];
    message.pose.position.y = x[1];
    message.pose.position.z = x[2];
    message.pose.orientation.w = q.w();
    message.pose.orientation.x = q.x();
    message.pose.orientation.y = q.y();
    message.pose.orientation.z = q.z();
    ref_pub_->publish(message);
}

void ControlNode::sample_PID()
{
    Eigen::Vector6d tau = PID_.main(q, reference_handler_.q_d, x, reference_handler_.x_d, v);
    moveEntity(tau);
}

// Main initiates the node, and keeps it alive
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}

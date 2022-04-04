#include "controller_package/controller_node.hpp"

ControlNode::ControlNode(const rclcpp::NodeOptions &options)
    : Node("Control_Node", options),
      gravitational_force(declare_parameter<double>("G_force", 1.0)),
      buoyancy_weight(declare_parameter<double>("Buoancy_and_Weight", 1.0)),
      scaling_linear_proportional_gain(declare_parameter<double>("Proportional_gain_linear", 5.0)),
      scaling_angular_proportional_gain(declare_parameter<double>("Proportional_gain_angular", 10.0)),
      scaling_linear_integral_gain(declare_parameter<double>("Integral_gain_linear", 5.0)),
      scaling_angular_integral_gain(declare_parameter<double>("Integral_gain_angular", 10.0)),
      maximum_integral_windup_attitude(declare_parameter<double>("Windup_max_attitude", 1.0)),
      maximum_integral_windup_position(declare_parameter<double>("Windup_max_position", 5.0)),
      scaling_derivative_gain(declare_parameter<double>("Derivative_gain", 1.0)),
      scaling_surge(declare_parameter<double>("Scaling_surge", 1.0)),
      scaling_sway(declare_parameter<double>("Scaling_sway", 1.0)),
      scaling_heave(declare_parameter<double>("Scaling_heave", 1.0)),
      control_mode(declare_parameter<int>("Control_mode", 0)),
      centre_of_gravity(declare_parameter<std::vector<double>>("Centre_of_gravity", {0.0, 0.0, 0.0})),
      center_of_buoyancy(declare_parameter<std::vector<double>>("Centre_of_buoyancy", {0.0, 0.0, 0.0}))
{
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&ControlNode::joystick_callback, this, _1));
    //state_estim_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //  "state_estimate", 10, std::bind(&ControlNode::estimate_callback, this, _1));

    ref_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/reference/pose", 10);
    act_pub_ = this->create_publisher<bluerov_interfaces::msg::ActuatorInput>("/actuation", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&ControlNode::reference_publisher, this));
    PIDTimer_ = this->create_wall_timer(30ms, std::bind(&ControlNode::sample_PID, this));
    LoggingTimer_ = this->create_wall_timer(30ms, std::bind(&ControlNode::logging, this));
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
    // update params in PID and Joystick
    joystick_handler_.update_params(scaling_surge, scaling_sway, scaling_heave);
    PID_.update_params(scaling_linear_proportional_gain, scaling_linear_integral_gain, scaling_derivative_gain,
                       centre_of_gravity, center_of_buoyancy, gravitational_force,
                       buoyancy_weight, scaling_angular_proportional_gain, scaling_angular_integral_gain,
                       maximum_integral_windup_attitude, maximum_integral_windup_position, control_mode);

    // Run PID
    Eigen::Vector6d tau = PID_.main(q, reference_handler_.q_d, x, reference_handler_.x_d, v);
    z_logging = PID_.getErrorVector(q, reference_handler_.q_d, x, reference_handler_.x_d);
    tau_logging = tau;
    moveEntity(tau);
}

void ControlNode::logging()
{
    rclcpp::Time time = clock_.now();
    Logg_.data_logger(tau_logging, z_logging, q, reference_handler_.q_d, x, reference_handler_.x_d, v, time.seconds());
}

// Main initiates the node, and keeps it alive
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}

#include "controller_package/controller_node.hpp"

ControlNode::ControlNode(const rclcpp::NodeOptions &options)
    : Node("Control_Node", options),
      use_ROS2_topic_as_setpoint(declare_parameter<bool>("Load_setpoint_from_topic", false)),
      enable_controller(declare_parameter<bool>("Enable_controller", false)),
      compensate_NED(declare_parameter<bool>("Compensate_NED", false)),
      weight(declare_parameter<double>("Weight", 0.0)),
      buoyancy(declare_parameter<double>("Buoancy", 0.0)),
      centre_of_gravity(declare_parameter<std::vector<double>>("Centre_of_gravity", {0.0, 0.0, 0.0})),
      centre_of_buoyancy(declare_parameter<std::vector<double>>("Centre_of_buoyancy", {0.0, 0.0, 0.0})),
      scaling_linear_proportional_gain(declare_parameter<double>("Proportional_gain_linear", 0.0)),
      scaling_angular_proportional_gain(declare_parameter<double>("Proportional_gain_angular", 0.0)),
      scaling_linear_integral_gain(declare_parameter<double>("Integral_gain_linear", 0.0)),
      scaling_angular_integral_gain(declare_parameter<double>("Integral_gain_angular", 0.0)),
      scaling_derivative_gain(declare_parameter<double>("Derivative_gain", 0.0)),
      max_windup_linear(declare_parameter<double>("Integral_windup_linear", 0.0)),
      max_windup_angular(declare_parameter<double>("Integral_windup_angular", 0.0)),
      use_integrator(declare_parameter<bool>("Enable_integrator", false)),
      use_linear_control_xy(declare_parameter<bool>("Linear_control_xy", false)),
      use_linear_control_z(declare_parameter<bool>("Linear_control_z", false)),
      m_scale(declare_parameter<double>("Manual_control_scaling", 40.0))
{
    //Activate subscriptions
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&ControlNode::joystick_callback, this, _1));
    state_estim_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "CSEI/observer/odom", 10, std::bind(&ControlNode::estimate_callback, this, _1));
    setpoint_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "controller/input/setpoint", 10, std::bind(&ControlNode::setpoint_callback, this, _1));

    //Activate publishers
    forces_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>("/controller/output/desired_forces", 10);

    //Activate timers
    SampleTimer_ = this->create_wall_timer(25ms, std::bind(&ControlNode::controller_node_main, this));
    ROS2ParamTimer_ = this->create_wall_timer(1000ms, std::bind(&ControlNode::get_ros2_params, this));
}


void ControlNode::joystick_callback(const sensor_msgs::msg::Joy msg)
{
    joystick_handler_.joystickToActions(msg.axes, msg.buttons);
    if (compensate_NED){
        // joystick_handler_.movement[0] = - joystick_handler_.movement[0];
        joystick_handler_.movement[1] = - joystick_handler_.movement[1];
        joystick_handler_.movement[2] = - joystick_handler_.movement[2];
        // joystick_handler_.movement[3] = - joystick_handler_.movement[3];
        joystick_handler_.movement[4] = - joystick_handler_.movement[4];
        joystick_handler_.movement[5] = - joystick_handler_.movement[5];
    }

    joy_axes_logging = msg.axes;
}

void ControlNode::estimate_callback(const nav_msgs::msg::Odometry msg){
    auto pos = msg.pose.pose.position;
    auto att = msg.pose.pose.orientation;
    auto lin = msg.twist.twist.linear;
    auto ang = msg.twist.twist.angular;
    x = Eigen::Vector3d(pos.x, pos.y, pos.z);
    v << lin.x, lin.y, lin.z, ang.x, ang.y, ang.z;
    q = Eigen::Quaterniond(att.w, att.x, att.y, att.z);
    q.normalize();
}

void ControlNode::setpoint_callback(const geometry_msgs::msg::Pose msg){
    topic_attitude_setpoint.w() = msg.orientation.w;
    topic_attitude_setpoint.x() = msg.orientation.x;
    topic_attitude_setpoint.y() = msg.orientation.y;
    topic_attitude_setpoint.z() = msg.orientation.z;
    topic_position_setpoint[0] = msg.position.x;
    topic_position_setpoint[1] = msg.position.y;
    topic_position_setpoint[2] = msg.position.z;   
}

void ControlNode::controller_node_main()
{
    //Logic to allow setpoint changes when releasing joystick
    setpoint_changes = Eigen::Vector6d::Zero();
    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector3d I_frame_lin = R * Eigen::Vector3d(joystick_handler_.movement[0], joystick_handler_.movement[1], joystick_handler_.movement[2]);
    for (int i = 0; i < 3; i++){
        active_actions[i] = (bool)I_frame_lin[i];
        active_actions[i + 3] = (bool)joystick_handler_.movement[i + 3];
        if (!active_actions[i] && last_tick_active_actions[i]){
            setpoint_changes[i] = true;
        }
        if (!active_actions[i + 3] && last_tick_active_actions[i + 3]){
            setpoint_changes[i + 3] = true;
        }
    }

    //Check and perform setpoint changes from joystick release
    //Also check if buttons are pressed to activate standard operations
    setpoint_holder_.update_setpoint(setpoint_changes, joystick_handler_.active_buttons, q, x);

    //Create smooth transition between manual control and controller
    if (!last_tick_is_controller_active && enable_controller){
        setpoint_holder_.x_d = x;
        setpoint_holder_.q_d = q;
        last_tick_is_controller_active = true;
    }
    else if (!enable_controller){
        last_tick_is_controller_active = false;
    }

    if (use_ROS2_topic_as_setpoint){ //If using ROS2 parameter setpoint, override joystick setpoint
        setpoint_holder_.q_d = topic_attitude_setpoint;
        setpoint_holder_.x_d = topic_position_setpoint;
    }
    if (!use_linear_control_xy){
        setpoint_holder_.x_d[0] = x[0];
        setpoint_holder_.x_d[1] = x[1];
        v[0] = 0;
        v[1] = 0;
    }
    if (!use_linear_control_z){
        setpoint_holder_.x_d[2] = x[2];
        v[2] = 0;
    }

    setpoint_holder_.q_d.normalize(); //Make sure the quaternion setpoint is normalized

    Eigen::Vector6d tau_final; //Variable to be published

    for (int i = 0; i < 3; i++){
        tau_final[i] = joystick_handler_.movement[i] * m_scale;
        tau_final[i + 3] = joystick_handler_.movement[i + 3] * m_scale / 2;
    }

    if (enable_controller){
        //Sample controller
        Eigen::Vector6d tau_controller = Controller_.main(q, setpoint_holder_.q_d, x, setpoint_holder_.x_d, v);
        for (int i = 0; i < 6; i++){
            if (!(bool) joystick_handler_.movement[i]){
                tau_final[i] = tau_controller[i];
            }
        }
    }

    publish_forces(tau_final); // Sends tau to a function that publishes to ROS
    z_logging = Controller_.getErrorVector(q, setpoint_holder_.q_d, x, setpoint_holder_.x_d); //Store z for logging
    tau_logging = tau_final; //Store tau for logging

    logging(); //Call logging function

    for (int i = 0; i < 6; i++){
        last_tick_active_actions[i] = active_actions[i];
    }
}

void ControlNode::publish_forces(Eigen::Vector6d tau)
{
  geometry_msgs::msg::Wrench forces = geometry_msgs::msg::Wrench();
  forces.force.x = tau[0];
  forces.force.y = tau[1];
  forces.force.z = tau[2];
  forces.torque.x = tau[3];
  forces.torque.y = tau[4];
  forces.torque.z = tau[5];
  forces_pub_->publish(forces);
}


void ControlNode::logging()
{
    rclcpp::Time time = clock_.now(); //Get current timestamp
    if (joystick_handler_.active_buttons[1] && !last_tick_logg_button_active){
        activateD = !activateD;
    }
    last_tick_logg_button_active = joystick_handler_.active_buttons[1];
    
    if (activateD == true && enable == false)           //Count teller up by one everytime activateD is true
    {
        enable = true;
        Logg_.teller +=1;
        Logg_.OnlyOnce = true;
    }
    else if (activateD == false)
    {
        enable = false;
    }


    if (activateD == true)
    {
        Logg_.data_logger2(tau_logging, z_logging, q, setpoint_holder_.q_d, x, setpoint_holder_.x_d, v, joy_axes_logging, Logg_.teller, Logg_.buffer2);    
    }
    Logg_.data_logger(tau_logging, z_logging, q, setpoint_holder_.q_d, x, setpoint_holder_.x_d, v, joy_axes_logging, time.seconds(), Logg_.buffer);
}

void ControlNode::get_ros2_params(){

    //Logic parameters
    this->get_parameter("Load_setpoint_from_topic", use_ROS2_topic_as_setpoint);
    this->get_parameter("Enable_controller", enable_controller);
    this->get_parameter("Compensate_NED", compensate_NED);
    this->get_parameter("Use_linear_control_xy", use_linear_control_xy);
    this->get_parameter("Use_linear_control_z", use_linear_control_z);
    this->get_parameter("Manual_control_scaling", m_scale);

    //Controller parameters
    this->get_parameter("Weight", weight);
    this->get_parameter("Buoancy", buoyancy);
    this->get_parameter("Centre_of_gravity", centre_of_gravity);
    this->get_parameter("Centre_of_buoyancy", centre_of_buoyancy);
    this->get_parameter("Proportional_gain_linear", scaling_linear_proportional_gain);
    this->get_parameter("Proportional_gain_angular", scaling_angular_proportional_gain);
    this->get_parameter("Integral_gain_linear", scaling_linear_integral_gain);
    this->get_parameter("Integral_gain_angular", scaling_angular_integral_gain);
    this->get_parameter("Derivative_gain", scaling_derivative_gain);
    this->get_parameter("Integral_windup_linear", max_windup_linear);
    this->get_parameter("Integral_windup_angular", max_windup_angular);
    this->get_parameter("Enable_integrator", use_integrator);

    // update params in controller
    Controller_.update_params(scaling_linear_proportional_gain, scaling_derivative_gain,
                              centre_of_gravity, centre_of_buoyancy, weight,
                              buoyancy, scaling_angular_proportional_gain, use_integrator, scaling_linear_integral_gain,
                              scaling_angular_integral_gain, max_windup_linear, max_windup_angular);
}

// Main initiates the node, and keeps it alive
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}

#include "controller_package/controller_node.hpp"

ControlNode::ControlNode(const rclcpp::NodeOptions &options)
    : Node("Control_Node", options),
      use_ROS2_topic_as_setpoint(declare_parameter<bool>("Use_ROS2_topic_as_setpoint", false)),
      use_imu_directly(declare_parameter<bool>("Use_imu_directly", false)),
      control_mode(declare_parameter<int>("Control_mode", 0)),
      world_frame_type(declare_parameter<int>("World_frame_type", 0)),
      ros2_param_attitude_setpoint(declare_parameter<std::vector<double>>("Attitude_setpoint", {1, 0, 0, 0})),
      ros2_param_position_setpoint(declare_parameter<std::vector<double>>("Position_setpoint", {0, 0, 0})),
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
      use_integrator(declare_parameter<bool>("Use_integrator", false)),
      use_linear_control_xy(declare_parameter<bool>("Use_linear_control_xy", false)),
      use_linear_control_z(declare_parameter<bool>("Use_linear_control_z", false)),
      m_scale(declare_parameter<double>("Manual_control_scaling", 40.0))
{
    //Activate subscriptions
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&ControlNode::joystick_callback, this, _1));
    //Vortex:
    state_estim_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered", 10, std::bind(&ControlNode::estimate_callback, this, _1));
    // state_estim_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //   "CSEI/observer/odom", 10, std::bind(&ControlNode::estimate_callback, this, _1));
    imu_estim_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "bno055/imu", 10, std::bind(&ControlNode::imu_callback, this, _1));
    setpoint_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "controller/setpoint", 10, std::bind(&ControlNode::setpoint_callback, this, _1));

    //Activate publishers
    forces_pub_ = this->create_publisher<geometry_msgs::msg::Wrench>("/thrust/desired_forces", 10);

    //Activate timers
    SampleTimer_ = this->create_wall_timer(25ms, std::bind(&ControlNode::controller_node_main, this));
    ROS2ParamTimer_ = this->create_wall_timer(1000ms, std::bind(&ControlNode::get_ros2_params, this));
}


void ControlNode::joystick_callback(const sensor_msgs::msg::Joy msg)
{
    joystick_handler_.joystickToActions(msg.axes, msg.buttons);
    if (world_frame_type == 1){ //If world_frame_type = 1, compensate for NED representation
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
    if (!use_imu_directly){
        x = Eigen::Vector3d(pos.x, pos.y, pos.z);
        v << lin.x, lin.y, lin.z, ang.x, ang.y, ang.z;
        q = Eigen::Quaterniond(att.w, att.x, att.y, att.z);
        q.normalize();
    }
    v[0] = 0;
    v[1] = 0;
    v[2] = 0;
}

void ControlNode::imu_callback(const sensor_msgs::msg::Imu msg){
    auto att = msg.orientation;
    auto ang = msg.angular_velocity;
    
    if (use_imu_directly){
        q = Eigen::Quaterniond(att.w, att.x, att.y, att.z);
        q.normalize();
        v << 0, 0, 0, ang.x, ang.y, ang.z;
    }
}

void ControlNode::setpoint_callback(const geometry_msgs::msg::Pose msg){
    // ros2_param_attitude_setpoint[0] = msg.orientation.w;
    // ros2_param_attitude_setpoint[1] = msg.orientation.x;
    // ros2_param_attitude_setpoint[2] = msg.orientation.y;
    // ros2_param_attitude_setpoint[3] = msg.orientation.z;
    // ros2_param_position_setpoint[0] = msg.position.x;
    // ros2_param_position_setpoint[1] = msg.position.y;
    // ros2_param_position_setpoint[2] = msg.position.z;   
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
    reference_handler_.update_setpoint(setpoint_changes, joystick_handler_.active_buttons, q, x);

    //Create smooth transition between manual control and PD controller
    if (!last_tick_is_controller_active && control_mode != 0){
        reference_handler_.x_d = x;
        reference_handler_.q_d = q;
        last_tick_is_controller_active = true;
    }
    else if (control_mode == 0){
        last_tick_is_controller_active = false;
    }

    if (use_ROS2_topic_as_setpoint){ //If using ROS2 parameter setpoint, override joystick setpoint
        reference_handler_.q_d.w() = ros2_param_attitude_setpoint[0];
        reference_handler_.q_d.x() = ros2_param_attitude_setpoint[1];
        reference_handler_.q_d.y() = ros2_param_attitude_setpoint[2];
        reference_handler_.q_d.z() = ros2_param_attitude_setpoint[3];
        reference_handler_.x_d[0] = ros2_param_position_setpoint[0];
        reference_handler_.x_d[1] = ros2_param_position_setpoint[1];
        reference_handler_.x_d[2] = ros2_param_position_setpoint[2];
    }
    if (!use_linear_control_xy){
        reference_handler_.x_d[0] = x[0];
        reference_handler_.x_d[1] = x[1];
    }
    if (!use_linear_control_z){
        reference_handler_.x_d[2] = x[2];
    }

    reference_handler_.q_d.normalize(); //Make sure the quaternion setpoint is normalized

    Eigen::Vector6d tau_final; //Variable to be published

    for (int i = 0; i < 3; i++){
        tau_final[i] = joystick_handler_.movement[i] * m_scale;
        tau_final[i + 3] = joystick_handler_.movement[i + 3] * m_scale / 2;
    }

    if (control_mode == 1){
        //Sample controller
        Eigen::Vector6d tau_controller = Controller_.main(q, reference_handler_.q_d, x, reference_handler_.x_d, v);
        for (int i = 0; i < 6; i++){
            if (!(bool) joystick_handler_.movement[i]){
                tau_final[i] = tau_controller[i];
            }
        }
    }

    publish_forces(tau_final); // Sends tau to a function that publishes to ROS
    z_logging = Controller_.getErrorVector(q, reference_handler_.q_d, x, reference_handler_.x_d); //Store z for logging
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
        Logg_.data_logger2(tau_logging, z_logging, q, reference_handler_.q_d, x, reference_handler_.x_d, v, joy_axes_logging, Logg_.teller, Logg_.buffer2);    
    }
    Logg_.data_logger(tau_logging, z_logging, q, reference_handler_.q_d, x, reference_handler_.x_d, v, joy_axes_logging, time.seconds(), Logg_.buffer);
}

void ControlNode::get_ros2_params(){

    //Logic parameters
    this->get_parameter("Use_ROS2_topic_as_setpoint", use_ROS2_topic_as_setpoint);
    this->get_parameter("Use_imu_directly", use_imu_directly);
    this->get_parameter("Control_mode", control_mode);
    this->get_parameter("World_frame_type", world_frame_type);
    this->get_parameter("Attitude_setpoint", ros2_param_attitude_setpoint);
    this->get_parameter("Position_setpoint", ros2_param_position_setpoint);
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
    this->get_parameter("Use_integrator", use_integrator);

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

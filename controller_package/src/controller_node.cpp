#include "controller_package/controller_node.hpp"

ControlNode::ControlNode(const rclcpp::NodeOptions &options)
    : Node("Control_Node", options),
      buoyancy(declare_parameter<double>("Buoancy", 0.0)),
      weight(declare_parameter<double>("Weight", 0.0)),
      scaling_linear_proportional_gain(declare_parameter<double>("Proportional_gain_linear", 0.0)),
      scaling_angular_proportional_gain(declare_parameter<double>("Proportional_gain_angular", 0.0)),
      scaling_derivative_gain(declare_parameter<double>("Derivative_gain", 0.0)),
      control_mode(declare_parameter<int>("Control_mode", 0)),
      setpoint_input_mode(declare_parameter<int>("Setpoint_input_mode", 0)),
      world_frame_type(declare_parameter<int>("World_frame_type", 0)),
      centre_of_gravity(declare_parameter<std::vector<double>>("Centre_of_gravity", {0.0, 0.0, 0.0})),
      centre_of_buoyancy(declare_parameter<std::vector<double>>("Centre_of_buoyancy", {0.0, 0.0, 0.0})),
      ros2_param_attitude_setpoint(declare_parameter<std::vector<double>>("Attitude_setpoint", {1, 0, 0, 0})),
      ros2_param_position_setpoint(declare_parameter<std::vector<double>>("Position_setpoint", {0, 0, 0}))
{
    //Activate subscriptions
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&ControlNode::joystick_callback, this, _1));
    state_estim_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "CSEI/observer/odom", 10, std::bind(&ControlNode::estimate_callback, this, _1));
    imu_estim_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "bno055/imu", 10, std::bind(&ControlNode::imu_callback, this, _1));

    //Activate publishers
    act_pub_ = this->create_publisher<bluerov_interfaces::msg::ActuatorInput>("/actuation", 10);
    act_pub_br2 = this->create_publisher<bluerov_interfaces::msg::ActuatorInput>("/actuation/bluerov2_standard", 10);
    data_pub_ = this->create_publisher<std_msgs::msg::Float32>("/logg_data", 10);
    data_pub_2 = this->create_publisher<std_msgs::msg::Float32>("/logg_data2", 10);
    data_pub_3 = this->create_publisher<std_msgs::msg::Float32>("/logg_data3", 10);
    data_pub_4 = this->create_publisher<std_msgs::msg::Float32>("/logg_data4", 10);

    //Activate timers
    SampleTimer_ = this->create_wall_timer(30ms, std::bind(&ControlNode::sample_controller, this));
    ROS2ParamTimer_ = this->create_wall_timer(1000ms, std::bind(&ControlNode::get_ros2_params, this));
}


void ControlNode::joystick_callback(const sensor_msgs::msg::Joy msg)
{
    joystick_handler_.joystickToActions(msg.axes, msg.buttons);
    if (world_frame_type == 1){ //If world_frame_type = 1, compensate for NED representation
        // joystick_handler_.movement[0] = - joystick_handler_.movement[0];
        joystick_handler_.movement[1] = - joystick_handler_.movement[1];
        joystick_handler_.movement[2] = - joystick_handler_.movement[2];
        joystick_handler_.movement[3] = - joystick_handler_.movement[3];
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
    q = Eigen::Quaterniond(att.w, att.x, att.y, att.z);
    q.normalize();
    v << lin.x, lin.y, lin.z, ang.x, ang.y, ang.z;
}

void ControlNode::imu_callback(const sensor_msgs::msg::Imu msg){
    auto att = msg.orientation;
    auto ang = msg.angular_velocity;
    q = Eigen::Quaterniond(att.w, att.x, att.y, att.z);
    q.normalize();
    v << 0, 0, 0, ang.x, ang.y, ang.z;
}

void ControlNode::sample_controller()
{
    std::vector<bool> setpoint_changes = {0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 6; i++){
        active_actions[i] = (bool)joystick_handler_.movement[i];
        if (!active_actions[i] && last_frame_active_actions[i]){
            setpoint_changes[i] = true;
        }
        else{
            setpoint_changes[i] = false;
        }
    }
    reference_handler_.update_setpoint(setpoint_changes, joystick_handler_.active_buttons, q, x);

    // Sample PID
    Eigen::Vector6d tau_controller = Controller_.main(q, reference_handler_.q_d, x, reference_handler_.x_d, v);
    Eigen::Vector6d tau_final;
    for (int i = 0; i < 3; i++){ //Manual input, may be overridden
        tau_final[i] = joystick_handler_.movement[i] * 40;
        tau_final[i + 3] = joystick_handler_.movement[i + 3] * 20;
    }

    switch (control_mode){
        case (0): // Manual control
            break;
        case (1): // PD Control
            for (int i = 0; i < 6; i++){
                if (!active_actions[i]){
                    tau_final[i] = tau_controller[i];
                }
            }
    }
    //Temp change when linear motion is not readable
    tau_final[0] = joystick_handler_.movement[0] * 40;
    tau_final[1] = joystick_handler_.movement[1] * 40;
    tau_final[2] = joystick_handler_.movement[2] * 40;
    bluerov2_standard_actuation(tau_final);
    send_actuation(tau_final);
    z_logging = Controller_.getErrorVector(q, reference_handler_.q_d, x, reference_handler_.x_d);
    tau_logging = tau_final;

    logg_data_publisher();
    logging();

    for (int i = 0; i < 6; i++){
        last_frame_active_actions[i] = active_actions[i];
    }
}

void ControlNode::send_actuation(Eigen::Vector6d tau)
{
  bluerov_interfaces::msg::ActuatorInput actuation_message_ = bluerov_interfaces::msg::ActuatorInput();
  Eigen::Vector8d thrusters_ = actuator_builder_.build_actuation(tau);
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

void ControlNode::bluerov2_standard_actuation(Eigen::Vector6d tau){
    bluerov_interfaces::msg::ActuatorInput actuation_message_ = bluerov_interfaces::msg::ActuatorInput();
    actuation_message_.header.stamp = clock_.now();
    actuation_message_.thrust1 = tau(0);
    actuation_message_.thrust2 = tau(1);
    actuation_message_.thrust3 = tau(2);
    actuation_message_.thrust4 = tau(3);
    actuation_message_.thrust5 = tau(4);
    actuation_message_.thrust6 = tau(5);
    actuation_message_.thrust7 = 0;
    actuation_message_.thrust8 = 0;
    act_pub_br2->publish(actuation_message_);
}



void ControlNode::logg_data_publisher()
{
    quat_to_euler();
    auto message = std_msgs::msg::Float32();
    auto message2 = std_msgs::msg::Float32();
    auto message3 = std_msgs::msg::Float32();
    auto message4 = std_msgs::msg::Float32();
    message.data = reference_handler_.q_d.w();
    message2.data = reference_handler_.q_d.x();
    message3.data = reference_handler_.q_d.y();
    message4.data = reference_handler_.q_d.z();

    data_pub_->publish(message);
    data_pub_2->publish(message2);
    data_pub_3->publish(message3);
    data_pub_4->publish(message4);
}
void ControlNode::quat_to_euler()
{
    euler << q.toRotationMatrix().eulerAngles(0,1,2); // roll, pith, yaw
}


void ControlNode::logging()
{
    rclcpp::Time time = clock_.now();
    Logg_.data_logger(tau_logging, z_logging, q, reference_handler_.q_d, x, reference_handler_.x_d, v, joy_axes_logging, time.seconds());
}

void ControlNode::get_ros2_params(){
    this->get_parameter("Weight", weight);
    this->get_parameter("Buoancy", buoyancy);
    this->get_parameter("Proportional_gain_linear", scaling_linear_proportional_gain);
    this->get_parameter("Proportional_gain_angular", scaling_angular_proportional_gain);
    this->get_parameter("Derivative_gain", scaling_derivative_gain);
    this->get_parameter("Control_mode", control_mode);
    this->get_parameter("Setpoint_input_mode", setpoint_input_mode);
    this->get_parameter("World_frame_type", world_frame_type);
    this->get_parameter("Centre_of_gravity", centre_of_gravity);
    this->get_parameter("Centre_of_buoyancy", centre_of_buoyancy);
    this->get_parameter("Attitude_setpoint", ros2_param_attitude_setpoint);
    this->get_parameter("Position_setpoint", ros2_param_position_setpoint);

    if (setpoint_input_mode == 1){
        reference_handler_.q_d.w() = ros2_param_attitude_setpoint[0];
        reference_handler_.q_d.x() = ros2_param_attitude_setpoint[1];
        reference_handler_.q_d.y() = ros2_param_attitude_setpoint[2];
        reference_handler_.q_d.z() = ros2_param_attitude_setpoint[3];
        reference_handler_.x_d[0] = ros2_param_position_setpoint[0];
        reference_handler_.x_d[1] = ros2_param_position_setpoint[1];
        reference_handler_.x_d[2] = ros2_param_position_setpoint[2];
        reference_handler_.q_d.normalize();
    }

    //Create smooth transition between manual control and PD controller
    if (!last_frame_is_controller_active && control_mode != 0){
        reference_handler_.x_d = x;
        reference_handler_.q_d = q;
        last_frame_is_controller_active = true;
    }
    else if (last_frame_is_controller_active && control_mode == 0){
        last_frame_is_controller_active = false;
    }

        // update params in controller
    Controller_.update_params(scaling_linear_proportional_gain, scaling_derivative_gain,
                              centre_of_gravity, centre_of_buoyancy, weight,
                              buoyancy, scaling_angular_proportional_gain, control_mode);
}

// Main initiates the node, and keeps it alive
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}

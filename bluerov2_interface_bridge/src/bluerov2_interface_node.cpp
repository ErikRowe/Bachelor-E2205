#include "bluerov2_interface_bridge/bluerov2_interface_node.hpp"

Bluerov2_interface_node::Bluerov2_interface_node(const rclcpp::NodeOptions &options)
    : Node("Bluerov2_interface_node", options)
{

    //Subscribe to setpoint topic and convert to controller-readable setpoints
    setpoint_subscriber = this->create_subscription<bluerov_interfaces::msg::Reference>(
      "CSE/references", 10, std::bind(&Bluerov2_interface_node::setpoint_callback, this, _1));

    //Subscribe to controller output and convert to bluerov2 interface message
    controller_subscriber = this->create_subscription<geometry_msgs::msg::Wrench>(
      "thrust/desired_forces", 10, std::bind(&Bluerov2_interface_node::controller_callback, this, _1));

    //Activate publishers
    bluerov2_thruster_publisher = this->create_publisher<bluerov_interfaces::msg::ActuatorInput>("/bluerov/u", 10);
    setpoint_publisher = this->create_publisher<geometry_msgs::msg::Pose>("/controller/setpoint", 10);


    //Initiate thruster message builder
    LENGTHS_THRUSTERS << 0.156, 0.156, -0.156, -0.156, 0.12, 0.12, -0.12, -0.12,
                        0.111, -0.111, 0.111, -0.111, 0.218, -0.218, 0.218, -0.218,
                        0.085, 0.085, 0.085, 0.085, 0, 0, 0, 0; 

    // The next two for loops builds the geometry message
    for (int i = 0; i < 4; i++)
    {
        double theta_thruster = M_PI / 180 * Thruster_install_angles[i];
        Eigen::Matrix3d z_rotation_matrix;
        z_rotation_matrix << cos(theta_thruster), -sin(theta_thruster), 0,
                sin(theta_thruster), cos(theta_thruster), 0,
                0, 0, 1;
        Eigen::Vector3d thrust_dir = Thruster_spin_direction[i] * z_rotation_matrix * e_1;
        B_.col(i) << thrust_dir, -thrust_dir.cross(LENGTHS_THRUSTERS.col(i));
    }
    for (int i = 4; i < 8; i++)
    {
        Eigen::Vector3d thrust_dir = - Thruster_spin_direction[i] * e_3;
        B_.col(i) << thrust_dir, -thrust_dir.cross(LENGTHS_THRUSTERS.col(i)); 
    }

    // Actual inversion of B_
    B_pinv_ = B_.completeOrthogonalDecomposition().pseudoInverse();    
}



void Bluerov2_interface_node::setpoint_callback(const bluerov_interfaces::msg::Reference msg){
  geometry_msgs::msg::Pose setpoint = geometry_msgs::msg::Pose();
  setpoint.position.x = msg.pos.x;
  setpoint.position.y = msg.pos.y;
  setpoint.position.z = msg.pos.z;
  setpoint.orientation = msg.quat;
  setpoint_publisher->publish(setpoint);
}

void Bluerov2_interface_node::controller_callback(const geometry_msgs::msg::Wrench msg)
{
  bluerov_interfaces::msg::ActuatorInput thruster_message = bluerov_interfaces::msg::ActuatorInput();
  Eigen::Vector6d tau = Eigen::Vector6d(msg.force.x, msg.force.y, msg.force.z,
                                        msg.torque.x, msg.torque.y, msg.torque.z);
                                        
  Eigen::Vector8d thrusters_ = B_pinv_ * tau;
  thruster_message.header.stamp = clock_.now();
  thruster_message.thrust1 = thrusters_(0);
  thruster_message.thrust2 = thrusters_(1);
  thruster_message.thrust3 = thrusters_(2);
  thruster_message.thrust4 = thrusters_(3);
  thruster_message.thrust5 = thrusters_(4);
  thruster_message.thrust6 = thrusters_(5);
  thruster_message.thrust7 = thrusters_(6);
  thruster_message.thrust8 = thrusters_(7);
  bluerov2_thruster_publisher->publish(thruster_message);
}

// Main initiates the node, and keeps it alive
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Bluerov2_interface_node>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}

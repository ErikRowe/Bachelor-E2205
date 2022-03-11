#include "controller_package/controller_node.hpp"



using std::placeholders::_1;
using namespace std::chrono_literals;

class ControlNode : public rclcpp::Node
{
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
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&ControlNode::joystick_callback, this, _1));
    //state_estim_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //  "state_estimate", 10, std::bind(&ControlNode::estimate_callback, this, _1));

    ref_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/reference/pose", 10);
    act_pub_ = this->create_publisher<bluerov_interfaces::msg::ActuatorInput>("/actuation", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&ControlNode::reference_publisher, this));
    PIDTimer_ = this->create_wall_timer(30ms, std::bind(&ControlNode::sample_PID, this));
  }

  void send_actuation(Eigen::Vector6d tau)
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
    PID.changeSetPoint(joystickInputClass.actions);
    
  }
/*
  void estimate_callback(const nav_msgs::msg::Odometry msg){
    auto pos = msg.pose.pose.position;
    auto att = msg.pose.pose.orientation;
    auto lin = msg.twist.twist.linear;
    auto ang = msg.twist.twist.angular;
    Eigen::Vector3d x = Eigen::Vector3d(pos.x, pos.y, pos.z);
    Eigen::Quaterniond q = Eigen::Quaterniond(att.w, att.x, att.y, att.z);
    Eigen::Vector6d velocity = Eigen::Vector6d(lin.x, lin.y, lin.z, ang.x, ang.y, ang.z);
    PID.updateGlobalParameters(x, q, velocity);
  }
  */

  void moveEntity(Eigen::Vector6d tau){
    double dt = 0.030;
    Eigen::Quaterniond q = PID.q;
    Eigen::Quaterniond q_new;
    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector3d v_lin = R * Eigen::Vector3d(tau[0], tau[1], tau[2]) * dt;
    Eigen::Vector3d v_ang = R * Eigen::Vector3d(tau[3], tau[4], tau[5]) * dt;
    Eigen::Vector3d x_new;
    x_new << PID.x[0] + v_lin[0], PID.x[1] + v_lin[1], PID.x[2] + v_lin[2];

    //Eigen::Vector3d v_B = R * v_ang;
    Eigen::Quaterniond q_relativeChange;
    q_relativeChange = Eigen::AngleAxisd(v_ang[0], Eigen::Vector3d::UnitX())
                      *Eigen::AngleAxisd(v_ang[1], Eigen::Vector3d::UnitY())
                      *Eigen::AngleAxisd(v_ang[2], Eigen::Vector3d::UnitZ());
    q_relativeChange.normalize();
    q_new = q_relativeChange * q;
    q.normalize();
    PID.q = q_new;
    PID.x = x_new;
  }

  void reference_publisher(){
    auto message = geometry_msgs::msg::PoseStamped();
    message.header.stamp = clock_.now();
    message.header.frame_id = "map";
    message.pose.position.x = PID.x[0];
    message.pose.position.y = PID.x[1];
    message.pose.position.z = PID.x[2];
    message.pose.orientation.w = PID.q.w();
    message.pose.orientation.x = PID.q.x();
    message.pose.orientation.y = PID.q.y();
    message.pose.orientation.z = PID.q.z();
    ref_pub_->publish(message);
  }

  void sample_PID(){
    Eigen::Vector6d tau = PID.main();
    send_actuation(tau);
  }

  // Private ROS2 declerations
  //rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_estim_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ref_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_; // Initiates subscriber to joy
  rclcpp::Publisher<bluerov_interfaces::msg::ActuatorInput >::SharedPtr act_pub_; // Initiates publisher to actuation driver
  bluerov_interfaces::msg::ActuatorInput actuation_message_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr PIDTimer_;
  UserJoystickInput joystickInputClass;
  PIDClass PID;
  Actuation actuation_; // Calls actuation class as an object
  rclcpp::Clock clock_; // Makes a clock for ros2
};



// Main initiates the node, and keeps it alive
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}

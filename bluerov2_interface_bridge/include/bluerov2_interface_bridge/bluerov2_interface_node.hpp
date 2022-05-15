
// Ros includes, these need to be included in dependencies
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/pose.hpp>

// Control group includes
#include "bluerov_interfaces/msg/actuator_input.hpp"
#include "bluerov_interfaces/msg/reference.hpp"

#include <vector>
#include <eigen3/Eigen/Dense>

namespace Eigen{
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
    typedef Eigen::Matrix<double, 8, 1> Vector8d;
    typedef Eigen::Matrix<double, 3, 8> Matrix38d;
}



using std::placeholders::_1;
using namespace std::chrono_literals;

class Bluerov2_interface_node : public rclcpp::Node
{
    public:
        explicit Bluerov2_interface_node(const rclcpp::NodeOptions& options);  
        std::vector<int> Thruster_spin_direction = {1, 1, -1, -1, 1, -1, -1, 1}; // This relates to the actuators and wether they run clockwize=-1 or anti-clockwize=1
        std::vector<int> Thruster_install_angles = {-45, 45, -135, 135};         // This relates to the angle at which the four angled rotors are placed

        Eigen::Matrix38d LENGTHS_THRUSTERS;         // Length of thrusters to center of drone. This needs to be verified
        Eigen::Vector3d e_1 = {1, 0, 0};            // Expect it to be î
        Eigen::Vector3d e_3 = {0, 0, 1};            // k

        Eigen::MatrixXd B_{6, 8};                   // Geometry matrix for the thrusters
        Eigen::MatrixXd B_pinv_;                    // The inverse of the B_ matrix
            
        rclcpp::Publisher<geometry_msgs::msg::Pose >::SharedPtr setpoint_publisher;
        rclcpp::Publisher<bluerov_interfaces::msg::ActuatorInput >::SharedPtr bluerov2_thruster_publisher;     // Publishes actuation message
        rclcpp::Subscription<bluerov_interfaces::msg::Reference >::SharedPtr setpoint_subscriber;
        rclcpp::Subscription<geometry_msgs::msg::Wrench >::SharedPtr controller_subscriber;
        rclcpp::Clock clock_;                                                               // Makes a clock for ros2



        /**
         * @brief Callback msg to read controller output and publish thruster values
         * 
         * @param msg Contains information regarding controller force and torque
         */
        void controller_callback(const geometry_msgs::msg::Wrench msg);

        /**
         * @brief Callback msg to read setpoint and publish general values to controller
         * 
         * @param msg Contains setpoint for linear and positional control
         */
        void setpoint_callback(const bluerov_interfaces::msg::Reference msg);
        
};
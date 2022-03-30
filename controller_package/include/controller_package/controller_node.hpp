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
#include "controller_package/common.hpp"
#include "bluerov_interfaces/msg/actuator_input.hpp"
#include "controller_package/joy_to_action.hpp"
#include "controller_package/control_actuator.hpp"
#include "controller_package/control_PID.hpp"
#include "controller_package/control_reference.hpp"



using std::placeholders::_1;
using namespace std::chrono_literals;

class ControlNode : public rclcpp::Node
{
    public:
        explicit ControlNode(const rclcpp::NodeOptions& options);        

    private:
        // Class declarations
        //rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_estim_sub_;        // Subscribes to state estimation
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ref_pub_;             // Publishes current state (WILL BE REMOVED)
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;                    // Subscribes to joy input
        rclcpp::Publisher<bluerov_interfaces::msg::ActuatorInput >::SharedPtr act_pub_;     // Publishes actuation message
        bluerov_interfaces::msg::ActuatorInput actuation_message_;                          // Actuation message object building
        rclcpp::TimerBase::SharedPtr timer_;                                                // ROS2 Timer for reference publish (WILL BE REMOVED)
        rclcpp::TimerBase::SharedPtr PIDTimer_;                                             // Timer to sample PID
        UserJoystickInput joystick_handler_;                                                // Instance of joystick input handler
        PIDClass PID_;                                                                      // Instance of PID logic handler
        Actuation actuation_;                                                               // Instance of actuation message handler
        ReferenceClass reference_handler_;                                                  // Instance of reference frame handler
        rclcpp::Clock clock_;                                                               // Makes a clock for ros2

        //params
        double gravitational_force;
        double buoyancy_weight;
        double scaling_linear_proportional_gain;
        double scaling_angular_proportional_gain;
        double scaling_linear_integral_gain;
        double scaling_angular_integral_gain;
        double maximum_integral_windup_attitude;
        double maximum_integral_windup_position;
        double scaling_derivative_gain;
        double scaling_surge;
        double scaling_sway;
        double scaling_heave;
        int control_mode;
        std::vector<double> centre_of_gravity;
        std::vector<double> center_of_buoyancy;



        Eigen::Vector3d x = Eigen::Vector3d::Zero();                    //Locally stored position vector
        Eigen::Quaterniond q = Eigen::Quaterniond(1, 0, 0, 0);          //Locally stored attitude vector
        Eigen::Vector6d v = Eigen::Vector6d::Zero();                    //Locally stored velocity vector

        /**
         * @brief builds an actuation message from PID output
         * 
         * @param tau sixD vector containing angular[3] and linear[3] movement
         */
        void send_actuation(Eigen::Vector6d tau);

        /**
         * @brief callback message to joy topic message. Sends message values to joystick input handler
         * 
         * @param msg Joy message with axes and button values
         */
        void joystick_callback(const sensor_msgs::msg::Joy msg); //const

        /**
         * @brief (WILL BE REMOVED) Function to test pipeline in RVIZ2
         * 
         * @param tau 
         */
        void moveEntity(Eigen::Vector6d tau);

        /**
         * @brief (WILL BE REMOVED) Function to publish state
         * 
         */
        void reference_publisher();

        /**
         * @brief Function to sample PID and retrieve tau
         * 
         */
        void sample_PID();

        // /**
        //  * @brief Callback msg to read global state estimaton
        //  * 
        //  * @param msg Contains information regarding velocities, position and attitude
        //  */
        // void estimate_callback(const nav_msgs::msg::Odometry msg);

        
};
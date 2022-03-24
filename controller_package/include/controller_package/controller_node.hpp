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
#include "controller_package/joy_to_action.h"
#include "controller_package/controller_complete.h"
#include "controller_package/control_actuator.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

class ControlNode : public rclcpp::Node
{
    public:
        ControlNode();        

    private:
        // Class declarations
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

        /**
         * @brief builds an actuation message from PID output
         * 
         * @param tau sixD vector containing angular[3] and linear[3] movement
         */
        void send_actuation(Eigen::Vector6d tau);

        /**
         * @brief callback message to joy topic message. 
         * Will transform joystick data to actions,
         * and use this to change the setpoint
         * 
         * @param msg 
         */
        void joystick_callback(const sensor_msgs::msg::Joy msg); //const

        /**
         * @brief 
         * 
         * @param tau 
         */
        void moveEntity(Eigen::Vector6d tau);

        /**
         * @brief 
         * 
         */
        void reference_publisher();

        /**
         * @brief 
         * 
         */
        void sample_PID();

        // /**
        //  * @brief 
        //  * 
        //  * @param msg 
        //  */
        // void estimate_callback(const nav_msgs::msg::Odometry msg);

        
};
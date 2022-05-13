#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdio>


// Ros includes, these need to be included in dependencies
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <nav_msgs/msg/odometry.hpp>

// Control group includes
#include "controller_package/common.hpp"
#include "bluerov_interfaces/msg/actuator_input.hpp"
#include "controller_package/joy_to_action.hpp"
#include "controller_package/controller_actuation_builder.hpp"
#include "controller_package/controller.hpp"
#include "controller_package/controller_reference.hpp"
#include "controller_package/logging.hpp"



using std::placeholders::_1;
using namespace std::chrono_literals;

class ControlNode : public rclcpp::Node
{
    public:
        explicit ControlNode(const rclcpp::NodeOptions& options);        

    private:
        // ROS2 Declarations
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_estim_sub_;          // Subscribes to state estimation
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_estim_sub_;              //Temp shit probably
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;                    // Subscribes to joy input
        rclcpp::Publisher<bluerov_interfaces::msg::ActuatorInput >::SharedPtr act_pub_;     // Publishes actuation message
        rclcpp::Publisher<bluerov_interfaces::msg::ActuatorInput >::SharedPtr act_pub_br2;  // Publishes actuation message to standard bluerov2
        rclcpp::Publisher<geometry_msgs::msg::Wrench >::SharedPtr vortex_pub;
        rclcpp::TimerBase::SharedPtr SampleTimer_;                                          // Timer to sample node main function
        rclcpp::TimerBase::SharedPtr ROS2ParamTimer_;                                       // Timer to update from ROS2 params
        rclcpp::Clock clock_;                                                               // Makes a clock for ros2

        // Class declarations
        UserJoystickInput joystick_handler_;                                                // Instance of joystick input handler
        ControllerClass Controller_;                                                        // Instance of PD logic handler
        LoggingClass Logg_;                                                                 // Instance of logging to file handler
        ActuationBuilderClass actuator_builder_;                                            // Instance of actuation message handler
        ReferenceClass reference_handler_;                                                  // Instance of reference logic handler

        //Logging variable decleration
        Eigen::Vector6d tau_logging = Eigen::Vector6d::Zero();                              // Save tau as a class variable to be used for logging
        Eigen::Vector6d z_logging = Eigen::Vector6d::Zero();                                // Save z as a class variable to be used for logging
        std::vector<float> joy_axes_logging = {0, 0, 0, 0, 0, 0, 0, 0};                     // Save axes inputs as a class variable to be used for logging
        bool activateD = false;
        bool last_tick_logg_button_active = false;
        bool enable = true;

        // ROS2 Parameters
        bool use_param_file_setpoint;
        bool use_imu_directly;
        int control_mode;
        int world_frame_type;
        std::vector<double> ros2_param_attitude_setpoint;
        std::vector<double> ros2_param_position_setpoint;
        double weight;
        double buoyancy;
        std::vector<double> centre_of_gravity;
        std::vector<double> centre_of_buoyancy;
        double scaling_linear_proportional_gain;
        double scaling_angular_proportional_gain;
        double scaling_linear_integral_gain;
        double scaling_angular_integral_gain;
        double scaling_derivative_gain;
        double max_windup_linear;
        double max_windup_angular;
        bool use_integrator;


        //Initialization of local variables
        Eigen::Vector3d x = Eigen::Vector3d::Zero();                    //Locally stored position vector
        Eigen::Quaterniond q = Eigen::Quaterniond(1, 0, 0, 0);          //Locally stored attitude vector
        Eigen::Vector6d v = Eigen::Vector6d::Zero();                    //Locally stored velocity vector
        bool last_tick_is_controller_active = false;                    //Variable to compare controller activity
        std::vector<bool> last_tick_active_actions = {false, false, false, false, false, false};          //Tracker if last tick had action input
        std::vector<bool> active_actions = {false, false, false, false, false, false};                    //Tracker if current tick has action input
        Eigen::Vector6d setpoint_changes;                                                               //Tracker for which actions are eligible for setpoint change


        /**
         * @brief callback message to joy topic message. Sends message values to joystick input handler
         * 
         * @param msg Joy message with axes and button values
         */
        void joystick_callback(const sensor_msgs::msg::Joy msg);

        /**
         * @brief Callback msg to read global state estimaton
         * 
         * @param msg Contains information regarding velocities, position and attitude
         */
        void estimate_callback(const nav_msgs::msg::Odometry msg);

        /**
         * @brief Callback msg to read global attitude estimation
         * 
         * @param msg Contains information regarding velocities and attitude
         */
        void imu_callback(const sensor_msgs::msg::Imu msg);

        /**
         * @brief Function to sample controller and handle general logic
         * 
         */
        void controller_node_main();

        /**
         * @brief builds an actuation message from PID output
         * 
         * @param tau sixD vector containing angular[3] and linear[3] movement (forces)
         */
        void send_actuation(Eigen::Vector6d tau);
        void send_vortex(Eigen::Vector6d tau);

        
        /**
         * @brief Function for logging parameters to files
         * 
         */
        void logging();


        /**
         * @brief Sends output in the form of surge, sway, heave, roll, pitch, yaw. Used by bluerov2 Mavlink messages
         * 
         * @param tau sixD vector containing angular[3] and linear[3] movement
         */
        void bluerov2_standard_actuation(Eigen::Vector6d tau);

        /**
         * @brief Reads ROS2 parameter changes and updates locally
         * 
         */
        void get_ros2_params();
        
};
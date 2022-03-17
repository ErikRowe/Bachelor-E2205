#include <eigen3/Eigen/Dense>
#include <vector>

#include "controller_package/eigen_typedef.h"

using std::placeholders::_1;

class PIDClass{
    public:

        Eigen::Vector3d x_d = Eigen::Vector3d::Zero();              //Setpoint position
        Eigen::Quaterniond q_d = Eigen::Quaterniond(1, 0, 0, 0);    //Setpoint attitude in quaternions
        Eigen::Vector3d x = Eigen::Vector3d::Zero();                //Current position
        Eigen::Quaterniond q = Eigen::Quaterniond(1, 0, 0, 0);      //Current attitude in quaternions
        Eigen::Vector6d v = Eigen::Vector6d::Zero();                //Current body-fixed velocities

        /**
         * @brief Performs PID operations to produce tau
         * 
         * @return 6x1 vector with linear force and angular torque
         * 
         */
        Eigen::Vector6d main();

        /**
         * @brief Read global variables and update local copies
         * 
         * @param position Global position in x, y, z
         * @param orientation Global orientation in Quaternion representation
         * @param velocity Body-frame linear and angular velocities
         * 
         */

        void updateGlobalParameters(Eigen::Vector3d position, Eigen::Quaterniond orientation,
                                    Eigen::Vector6d velocity);

        /**
         * @brief Reads user actions and converts to setpoint change
         * 
         * @param actions Joystick inputs from axes and buttons
         * 
         */
        void changeSetPoint(std::vector<double> actions);

    private:
        /**
         * @brief Calculates proportional gain
         * 
         * @param R Current attitude as rotational matrix
         * 
         * @return Proportional gain as 6x6 matrix
         * 
         */
        Eigen::Matrix6d proportionalGain(Eigen::Matrix3d R);


        /**
         * @brief Uses position and attitude to compute an error vector
         *
         * @return 6x1 vector with position and attitude error
         */
        Eigen::Vector6d getErrorVector();
        //Eigen::Matrix6d integralGain();

        /**
         * @brief Finds the sign of a value
         * 
         * @param x A number to find the sign of
         * @return 1 or -1
         */
        int signum(double x);
        
        Eigen::Matrix3d Kx = Eigen::Matrix3d::Identity() * 5;       //Scaling of proportional gain
        Eigen::Matrix6d Kd = Eigen::Matrix6d::Identity();           //Scaling of derivative gain
        Eigen::Vector3d rg = Eigen::Vector3d::Zero();               //Centre of gravity
        Eigen::Vector3d rb = Eigen::Vector3d::Zero();               //Centre of buoyancy
        double W = 1;        //Gravitational force mg
        double B = 1;        //Weight and buoyancy
        double c = 10.0;     //Scaling constant for proportional gain
        bool last_frame_active_actions[4];  //Tracker if last frame had action input
};
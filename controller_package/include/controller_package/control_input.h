#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

using std::placeholders::_1;
namespace Eigen{
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
}

class PIDInputClass{
    public:
        Eigen::Vector3d x_d = Eigen::Vector3d::Zero();              //Setpoint position
        Eigen::Quaterniond q_d = Eigen::Quaterniond(1, 0, 0, 0);    //Setpoint attitude in quaternions
        Eigen::Vector3d x = Eigen::Vector3d::Zero();                //Current position
        Eigen::Quaterniond q = Eigen::Quaterniond(1, 0, 0, 0);      //Current attitude in quaternions
        Eigen::Vector6d v = Eigen::Vector6d::Zero();                //Current body-fixed velocities

        /**
         * @brief Read global variables and update copies
         * 
         * @param position Global position in x, y, z
         * @param orientation Global orientation in Quaternion representation
         * @param velocity Body-frame linear and angular velocities
         * 
         */

        void updateGlobalParameters(Eigen::Vector3d position, Eigen::Quaterniond orientation,
                                    Eigen::Vector6d velocity);

        /**
         * @brief Uses position and attitude to compute an error vector
         *
         * @return 6x1 vector with position and attitude error
         */
        Eigen::Vector6d getErrorVector();


        /**
         * @brief Reads user actions and converts to setpoint change
         * 
         * @param actions Joystick inputs from axes and buttons
         * 
         */
        void changeSetPoint(std::vector<double> actions);

    private:
        bool last_frame_active_actions[4];  //Tracker if last frame had action input

        /**
         * @brief Finds the sign of a value
         * 
         * @param x A number to find the sign of
         * @return 1 or -1
         */
        int signum(double x);
};
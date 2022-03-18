#include <vector>

#include "controller_package/eigen_typedef.h"

using std::placeholders::_1;

class PIDClass{
    public:

        /**
         * @brief Performs PID operations to produce tau
         * 
         * @param q     Current attitude in quaternion representation
         * @param q_d   Setpoint attitude in quaternion representation
         * @param x     Current position in world frame
         * @param x_d   Setpoint position in world frame
         * @param v     Current linear and angular velocities in the body frame
         * 
         * @return 6x1 vector with linear force and angular torque
         * 
         */
        Eigen::Vector6d main(const Eigen::Quaterniond &q, const Eigen::Quaterniond &q_d,
                             const Eigen::Vector3d &x, const Eigen::Vector3d &x_d,
                             const Eigen::Vector6d &v);

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
         * @param q     Current attitude in quaternion representation
         * @param q_d   Setpoint attitude in quaternion representation
         * @param x     Current position in world frame
         * @param x_d   Setpoint position in world frame
         *
         * @return 6x1 vector with position and attitude error
         */
        Eigen::Vector6d getErrorVector(const Eigen::Quaterniond &q, const Eigen::Quaterniond &q_d,
                                       const Eigen::Vector3d &x, const Eigen::Vector3d &x_d);
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
};
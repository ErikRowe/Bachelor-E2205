#include "controller_package/common.hpp"

class ControllerClass{
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

        /**
         * @brief Function updates the parameters of the controller from params.yaml
         * 
         * @param _Kx 
         * @param _Kd 
         * @param _rG 
         * @param _rB 
         * @param _W 
         * @param _B 
         * @param _c
         * @param _control_mode
         */
        void update_params(double _Kx, double _Kd, std::vector<double> _rG, std::vector<double> _rB,
                           double _W, double _B, double _c, int _control_mode);

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
         * @brief Calculates integral gain
         * 
         * @param R Current attitude as rotational matrix
         * 
         * @return Integral gain as 6x6 matrix
         * 
         */
        Eigen::Matrix6d integralGain(Eigen::Matrix3d R);

        /**
         * @brief Finds the sign of a value
         * 
         * @param x A number to find the sign of
         * @return 1 or -1
         */
        int signum(double x);
        
        Eigen::Matrix3d Kx;         //Scaling of linear proportional gain
        Eigen::Matrix6d Kd;         //Scaling of derivative gain
        Eigen::Vector3d rg;         //Centre of gravity
        Eigen::Vector3d rb;         //Centre of buoyancy
        double W;                   //Gravitational force mg
        double B;                   //Weight and buoyancy
        double c;                   //Scaling constant for angular proportional gain
        int control_mode;           //Type of control (open loop, PD etc)

        Eigen::Matrix3d Kx_i;       //Scaling of linear integral gain
        Eigen::Vector6d integral;   //Stores integrated value
        double c_i;                 //Scaling constant for angular integral gain
        double windup_linear;       //Maximum value for linear integrator gain
        double windup_angular;      //Maximum value for angular integrator gain
};

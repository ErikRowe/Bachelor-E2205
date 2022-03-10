
#include "controller_package/control_actuator.hpp"







Eigen::Vector8d Actuation::build_actuation(Eigen::vector6d tau){
    LENGTHS_THRUSTERS << 0.156, 0.156, -0.156, -0.156, 0.12, 0.12, -0.12, -0.12,
                        0.111, -0.111, 0.111, -0.111, 0.218, -0.218, 0.218, -0.218,
                        0.085, 0.085, 0.085, 0.085, 0, 0, 0, 0; 
    for (int i = 0; i < 4; i++)
    {
        double temp2 = kPi / 180 * ALPHA[i];
        Eigen::Matrix3d temp3;
        temp3 << cos(temp2), -sin(temp2), 0,
                 sin(temp2), cos(temp2), 0,
                 0, 0, 1;

        
        Eigen::Vector3d thrust_dir = MIXER[i] * temp3 * e_1;
        B_.col(i) << thrust_dir, -thrust_dir.cross(LENGTHS_THRUSTERS.col(i));
    }
    for (int i = 4; i < 8; i++)
    {
        Eigen::Vector3d thrust_dir = - MIXER[i] * e_3;
        B_.col(i) << thrust_dir, -thrust_dir.cross(LENGTHS_THRUSTERS.col(i)); 
    }

    B_pinv_ = B_.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::Vector8d thrust_ = B_pinv_ * tau;
    return thrust_;
}
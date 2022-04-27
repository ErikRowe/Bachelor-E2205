
#include "controller_package/controller_actuation_builder.hpp"





ActuationBuilderClass::ActuationBuilderClass(){
    LENGTHS_THRUSTERS << 0.156, 0.156, -0.156, -0.156, 0.12, 0.12, -0.12, -0.12,
                        0.111, -0.111, 0.111, -0.111, 0.218, -0.218, 0.218, -0.218,
                        0.085, 0.085, 0.085, 0.085, 0, 0, 0, 0; 

    // The next two for loops Builds the geometry message
    for (int i = 0; i < 4; i++)
    {
        double theta_thruster = M_PI / 180 * Thruster_install_angles[i];
        Eigen::Matrix3d z_rotation_matrix;
        z_rotation_matrix << cos(theta_thruster), -sin(theta_thruster), 0,
                sin(theta_thruster), cos(theta_thruster), 0,
                0, 0, 1;
        Eigen::Vector3d thrust_dir = Thruster_spin_direction[i] * z_rotation_matrix * e_1;
        B_.col(i) << thrust_dir, -thrust_dir.cross(LENGTHS_THRUSTERS.col(i));
    }
    for (int i = 4; i < 8; i++)
    {
        Eigen::Vector3d thrust_dir = - Thruster_spin_direction[i] * e_3;
        B_.col(i) << thrust_dir, -thrust_dir.cross(LENGTHS_THRUSTERS.col(i)); 
    }

    // Actual inversion of B_
    B_pinv_ = B_.completeOrthogonalDecomposition().pseudoInverse();
}

Eigen::Vector8d ActuationBuilderClass::build_actuation(Eigen::Vector6d tau){
    Eigen::Vector8d thrust_ = B_pinv_ * tau;
    return thrust_;
}
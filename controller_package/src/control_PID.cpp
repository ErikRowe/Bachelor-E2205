#include "controller_package/control_PID.hpp"



Eigen::Vector6d PIDClass::main(const Eigen::Quaterniond &q, const Eigen::Quaterniond &q_d,
                               const Eigen::Vector3d &x, const Eigen::Vector3d &x_d,
                               const Eigen::Vector6d &v){
    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Matrix6d Kp = proportionalGain(R);
    Eigen::Vector6d z = getErrorVector(q, q_d, x, x_d);

    Eigen::Vector3d fg = R.transpose() * Eigen::Vector3d(0, 0, W);
    Eigen::Vector3d fb = R.transpose() * Eigen::Vector3d(0, 0, B);

    Eigen::Vector6d g;
    Eigen::Vector6d tau;
    g << fb - fg, rb.cross(fb) - rg.cross(fg);
    tau = -Kd * v - Kp * z + g;
    return tau;
}


Eigen::Matrix6d PIDClass::proportionalGain(Eigen::Matrix3d R){
    Eigen::Matrix6d Kp;
    Eigen::Matrix3d zero = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d I3x3 = Eigen::Matrix3d::Identity();

    Kp << R.transpose() * Kx, zero, zero, c * I3x3;

    return Kp;
}

int PIDClass::signum(double x){
    if(x >= 0){
        return 1;
    }
    return -1;
}

Eigen::Vector6d PIDClass::getErrorVector(const Eigen::Quaterniond &q, const Eigen::Quaterniond &q_d,
                                         const Eigen::Vector3d &x, const Eigen::Vector3d &x_d){
    Eigen::Vector3d x_tilde = x - x_d;
    Eigen::Quaterniond q_tilde = q_d.conjugate()*q;
    q_tilde.normalize();
    int nu = signum(q_tilde.w());

    Eigen::Vector6d z(6);
    z << x_tilde, nu * q_tilde.vec();
    return z;
}


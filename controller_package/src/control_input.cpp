#include <eigen3/Eigen/Dense>
#include <iostream>

namespace Eigen{
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
}


int signum(double x){
    if(x >= 0){
        return 1;
    }
    return -1;
}

Eigen::Vector6d error_vector(Eigen::Quaterniond q, Eigen::Quaterniond q_d,
                             Eigen::Vector3d x, Eigen::Vector3d x_d){
    Eigen::Vector3d x_tilde = x - x_d;
    Eigen::Quaterniond q_tilde = q_d.conjugate()*q;
    q_tilde.normalize();
    int mu = signum(q_tilde.w());

    Eigen::Vector6d z(6);
    z << x_tilde, q_tilde.vec();
    return z;
}
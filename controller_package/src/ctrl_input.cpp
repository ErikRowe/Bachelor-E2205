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

int main(){
    Eigen::Quaterniond q(1, 0, 0, 0);
    Eigen::Quaterniond q_d(0, 0, 0, 1);
    Eigen::Vector3d x(3, 7, -2);
    Eigen::Vector3d x_d(-5, 10, 8);

    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector6d z = error_vector(q, q_d, x, x_d);
    std::cout << "R=" << std::endl << R << std::endl;
    std::cout << "Error vector z=" << std::endl << z << std::endl;
    return 0;
}
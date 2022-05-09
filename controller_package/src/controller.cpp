#include "controller_package/controller.hpp"



Eigen::Vector6d ControllerClass::main(const Eigen::Quaterniond &q, const Eigen::Quaterniond &q_d,
                                      const Eigen::Vector3d &x, const Eigen::Vector3d &x_d,
                                      const Eigen::Vector6d &v){
    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Matrix6d Kp = proportionalGain(R);
    Eigen::Vector6d z = getErrorVector(q, q_d, x, x_d);
    
    Eigen::Vector3d fg = R.transpose() * Eigen::Vector3d(0, 0, W);
    Eigen::Vector3d fb = R.transpose() * Eigen::Vector3d(0, 0, B);
  
    Eigen::Matrix6d Ki = integralGain(R);
    integral += Ki * z;
  
    for (int i = 0; i < 3; i++){
      if (abs(integral[i]) > windup_linear){
        integral[i] = signum(integral[i]) * windup_linear;
      }
      if (abs(integral[i + 3]) > windup_angular){
        integral[i + 3] = signum(integral[i + 3]) * windup_angular;
      }
    }

    Eigen::Vector6d g;
    Eigen::Vector6d tau;
    g << fb - fg, rb.cross(fb) - rg.cross(fg);

    tau = -Kd * v - Kp * z + g - integral;
    return tau;
}

Eigen::Matrix6d ControllerClass::proportionalGain(Eigen::Matrix3d R){
    Eigen::Matrix6d Kp;
    Eigen::Matrix3d zero = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d I3x3;
    I3x3.diagonal() << 2, 2, 1;

    Kp << R.transpose() * Kx, zero, zero, c * I3x3;

    return Kp;
}

Eigen::Matrix6d ControllerClass::integralGain(Eigen::Matrix3d R){
    Eigen::Matrix6d Ki;
    Eigen::Matrix3d zero = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d I3x3;
    I3x3.diagonal() << 2, 2, 1;

    Ki << R.transpose() * Kx_i, zero, zero, c_i * I3x3;

    return Ki;
}

int ControllerClass::signum(double x){
    if(x >= 0){
        return 1;
    }
    return -1;
}

Eigen::Vector6d ControllerClass::getErrorVector(const Eigen::Quaterniond &q, const Eigen::Quaterniond &q_d,
                                                const Eigen::Vector3d &x, const Eigen::Vector3d &x_d){
    Eigen::Vector3d x_tilde = x - x_d;
    Eigen::Quaterniond q_tilde = q_d.conjugate()*q;
    q_tilde.normalize();
    int nu = signum(q_tilde.w());

    Eigen::Vector6d z(6);
    z << x_tilde, nu * q_tilde.vec();
    return z;
}

void ControllerClass::update_params(double _Kx,double _Kd, std::vector<double> _rG, std::vector<double> _rB,
                                    double _W, double _B, double _c, int _control_mode)
{   
    Kx = Eigen::Matrix3d::Identity() * _Kx;
    Kd = Eigen::Matrix6d::Identity() * _Kd;
    rg = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(_rG.data(), _rG.size());
    rb = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(_rB.data(), _rB.size());
    W = _W;
    B = _B;
    c = _c;
    control_mode = _control_mode;
  
    Kx_i = Kx;
    c_i = c / 10;
    windup_linear = 10.0;
    windup_angular = 10.0;
}


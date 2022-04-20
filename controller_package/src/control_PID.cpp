#include "controller_package/control_PID.hpp"



Eigen::Vector6d PIDClass::main(const Eigen::Quaterniond &q, const Eigen::Quaterniond &q_d,
                               const Eigen::Vector3d &x, const Eigen::Vector3d &x_d,
                               const Eigen::Vector6d &v){
    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Matrix6d Kp = proportionalGain(R);
    Eigen::Vector6d z = getErrorVector(q, q_d, x, x_d);
    
    Eigen::Matrix6d Ki = integralGain(R);
    integral += Ki * z;
    limit_integral_windup();
    

    Eigen::Vector3d fg = R.transpose() * Eigen::Vector3d(0, 0, W);
    Eigen::Vector3d fb = R.transpose() * Eigen::Vector3d(0, 0, B);

    Eigen::Vector6d g;
    Eigen::Vector6d tau;
    g << fb - fg, rb.cross(fb) - rg.cross(fg);

    //If PD -> I = 0
    if (control_mode != 2){
        integral = Eigen::Vector6d::Zero();
    }
    tau = -Kd * v - Kp * z - integral + g;
    return tau;
}

Eigen::Matrix6d PIDClass::proportionalGain(Eigen::Matrix3d R){
    Eigen::Matrix6d Kp;
    Eigen::Matrix3d zero = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d I3x3 = Eigen::Matrix3d::Identity();

    Kp << R.transpose() * Kx, zero, zero, c * I3x3;

    return Kp;
}

Eigen::Matrix6d PIDClass::integralGain(Eigen::Matrix3d R){
    Eigen::Matrix6d Ki;
    Eigen::Matrix3d zero = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d I3x3 = Eigen::Matrix3d::Identity();

    Ki << R.transpose() * Kxi, zero, zero, c_i * I3x3;

    return Ki;
}

void PIDClass::limit_integral_windup(){
    for (int i = 0; i < 3; i++){
        if (abs(integral[i]) > W_max_pos){
            integral[i] = W_max_pos * signum(integral[i]);
        }
        if (abs(integral[i + 3]) > W_max_att){
            integral[i + 3] = W_max_att * signum(integral[i + 3]);
        }
    }
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

void PIDClass::update_params(double _Kx, double _Kxi, double _Kd, std::vector<double> _rG, std::vector<double> _rB,
                             double _W, double _B, double _c, double _c_i, double _windup_att, double _windup_pos, int _control_mode)
{
    Kx = Eigen::Matrix3d::Identity() * _Kx;
    Kxi = Eigen::Matrix3d::Identity() * _Kxi;
    Kd = Eigen::Matrix6d::Identity() * _Kd;
    rg = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(_rG.data(), _rG.size());
    rb = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(_rB.data(), _rB.size());
    W = _W;
    B = _B;
    c = _c;
    c_i = _c_i;
    W_max_att = _windup_att;
    W_max_pos = _windup_pos;
    control_mode = _control_mode;
}


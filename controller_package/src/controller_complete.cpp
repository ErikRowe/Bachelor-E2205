#include "controller_package/controller_complete.h"



Eigen::Vector6d PIDClass::main(){
    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Matrix6d Kp = proportionalGain(R);
    Eigen::Vector6d z = getErrorVector();

    Eigen::Vector3d fg = R.transpose() * Eigen::Vector3d(0, 0, W);
    Eigen::Vector3d fb = R.transpose() * Eigen::Vector3d(0, 0, B);

    Eigen::Vector6d g;
    Eigen::Vector6d tau;
    g << fb - fg, rb.cross(fb) - rg.cross(fg);
    tau = -Kd * v - Kp * z + g;
    return tau;
}

void PIDClass::updateGlobalParameters(Eigen::Vector3d position, Eigen::Quaterniond orientation,
                                           Eigen::Vector6d velocity){
    x = position;
    q = orientation;
    v = velocity;
}

void PIDClass::changeSetPoint(std::vector<double> actions){

    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector3d B_frame_linear_action = R * Eigen::Vector3d(actions[0], actions[1], actions[2]);
    bool orientChange = false;
    for (int i = 0; i < 3; i++){
        if (B_frame_linear_action[i] != 0){
            last_frame_active_actions[i] = true;
            x_d[i] = x[i] + B_frame_linear_action[i];
        }
        else if (last_frame_active_actions[i] == true){
            last_frame_active_actions[i] = false;
            x_d[i] = x[i];
        }

        if (actions[i + 3] != 0){
            orientChange = true;
        }
    }
    
    if (orientChange){
        Eigen::Vector3d B_frame_angular_action = R * Eigen::Vector3d(actions[3], actions[4], actions[5]);
        Eigen::Quaterniond q_relativeChange;
        q_relativeChange = Eigen::AngleAxisd(B_frame_angular_action[0], Eigen::Vector3d::UnitX())
                          *Eigen::AngleAxisd(B_frame_angular_action[1], Eigen::Vector3d::UnitY())
                          *Eigen::AngleAxisd(B_frame_angular_action[2], Eigen::Vector3d::UnitZ());
        q_relativeChange.normalize();
        q_d = q_relativeChange * q;
        last_frame_active_actions[4] = true;
    }
    else if (last_frame_active_actions[4] == true){
        q_d = q;
        last_frame_active_actions[4] = false;
    }
    q_d.normalize();
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

Eigen::Vector6d PIDClass::getErrorVector(){
    Eigen::Vector3d x_tilde = x - x_d;
    Eigen::Quaterniond q_tilde = q_d.conjugate()*q;
    q_tilde.normalize();
    int nu = signum(q_tilde.w());

    Eigen::Vector6d z(6);
    z << x_tilde, nu * q_tilde.vec();
    return z;
}


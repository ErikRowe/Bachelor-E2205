
#include "controller_package/control_input.h"


void PIDInputClass::updateGlobalParameters(Eigen::Vector3d position, Eigen::Quaterniond orientation,
                                           Eigen::Vector6d velocity){
    x = position;
    q = orientation;
    v = velocity;
}

void PIDInputClass::changeSetPoint(std::vector<double> actions){

    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector3d B_frame_action = R.transpose() * Eigen::Vector3d(actions[0], actions[1], actions[2]);
    bool orientChange = false;
    for (int i = 0; i < 3; i++){
        if (B_frame_action[i] != 0){
            last_frame_active_actions[i] = true;
            x_d[i] = x[i] + B_frame_action[i];
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
        Eigen::Quaterniond q_relativeChange;
        q_relativeChange = Eigen::AngleAxisd(actions[3], Eigen::Vector3d::UnitX())
                        *Eigen::AngleAxisd(actions[4], Eigen::Vector3d::UnitY())
                        *Eigen::AngleAxisd(actions[5], Eigen::Vector3d::UnitZ());
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



int PIDInputClass::signum(double x){
    if(x >= 0){
        return 1;
    }
    return -1;
}

Eigen::Vector6d PIDInputClass::getErrorVector(){
    Eigen::Vector3d x_tilde = x - x_d;
    Eigen::Quaterniond q_tilde = q_d.conjugate()*q;
    q_tilde.normalize();
    int nu = signum(q_tilde.w());

    Eigen::Vector6d z(6);
    z << x_tilde, nu * q_tilde.vec();
    return z;
}



#include "controller_package/control_input.h"


PIDInputClass::PIDInputClass(){
        x_d = {0, 0, 0};
        q_d = {1, 0, 0, 0};
        x = {0, 0, 0};
        q = {1, 0, 0, 0};
}

void PIDInputClass::readPosAtt(double x_global, double y_global, double z_global){
    x << x_global, y_global, z_global;
}

void PIDInputClass::changeSetPoint(double actions[6]){

    for (int i = 0; i < 3; i++){
        if (actions[i] != 0){
            last_frame_active_actions[i] = true;
            x_d[i] = x[i] + actions[i];
        }
        else if (last_frame_active_actions[i] == true){
            last_frame_active_actions[i] = false;
            x_d[i] = x[i];
        }
    }
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


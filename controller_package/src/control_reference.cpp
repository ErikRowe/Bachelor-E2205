#include "controller_package/control_reference.hpp"

void ReferenceClass::changeSetPoint(std::vector<double> movement, const Eigen::Quaterniond &q,
                                    const Eigen::Vector3d &x){

    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector3d I_frame_linear_action = R * Eigen::Vector3d(movement[0], movement[1], movement[2]);
    bool orientChange = false;
    for (int i = 0; i < 3; i++){
        if (I_frame_linear_action[i] != 0){
            last_frame_active_actions[i] = true;
            x_d[i] = x[i] + I_frame_linear_action[i];
        }
        else if (last_frame_active_actions[i] == true){
            last_frame_active_actions[i] = false;
            x_d[i] = x[i];
        }

        if (movement[i + 3] != 0){
            orientChange = true;
        }
    }
    
    if (orientChange){
        Eigen::Quaterniond q_relativeChange;
        q_relativeChange = Eigen::AngleAxisd(movement[3], Eigen::Vector3d::UnitX())
                          *Eigen::AngleAxisd(movement[4], Eigen::Vector3d::UnitY())
                          *Eigen::AngleAxisd(movement[5], Eigen::Vector3d::UnitZ());
        q_relativeChange.normalize();

        q_d = q * q_relativeChange;
        last_frame_active_actions[4] = true;
    }
    else if (last_frame_active_actions[4] == true){
        q_d = q;
        last_frame_active_actions[4] = false;
    }
    q_d.normalize();
}
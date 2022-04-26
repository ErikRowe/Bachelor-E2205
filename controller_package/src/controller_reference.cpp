#include "controller_package/controller_reference.hpp"

void ReferenceClass::update_setpoint(std::vector<bool> setpoint_changes, std::vector<bool> buttons,
                                     const Eigen::Quaterniond &q, const Eigen::Vector3d &x){
    
    Eigen::Vector3d setpoint_change_lin = x;
    Eigen::Quaterniond setpoint_change_att = q;

    handle_button_input(setpoint_change_lin, setpoint_change_att, buttons, q);

    setpoint_change_att.normalize();

    for (int i = 0; i < 3; i++){
        if (setpoint_changes[i]){
            x_d[i] = setpoint_change_lin[i];
        }
        if (setpoint_changes[i + 3]){
            q_d = setpoint_change_att;
        }
    }
}


void ReferenceClass::handle_button_input(Eigen::Vector3d &setpoint_change_lin, Eigen::Quaterniond &setpoint_change_att,
                                         std::vector<bool> buttons, const Eigen::Quaterniond &q){
    
    // auto return_to_surface = [&](){
    //     setpoint_change_att = Eigen::Quaterniond(1, 0, 0, 0);
    //     setpoint_change_lin.z() = 0;
    //     return 0;
    // };

    auto reset_attitude = [&](){
        setpoint_change_att = Eigen::Quaterniond(1, 0, 0, 0);
        return 0;
    };

    auto surge_step = [&](){
        return 0;
        Eigen::Matrix3d R = q.toRotationMatrix();
        Eigen::Vector3d surge = R * Eigen::Vector3d(5, 0, 0);
        setpoint_change_lin += surge;
        return 0;
    };

    surge_step();
    
    auto yaw_step = [&](){
        setpoint_change_att *= Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
                             *Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
                             *Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitZ());
        return 0;
    };

    //Array with action-functions to loop through
    //Needs to correspond to active_buttons in joy_to_action.cpp
    std::vector<std::function<int()>> actions = {reset_attitude, yaw_step};
    int counter = 1;
    //Loops through button inputs and corresponding action functions
    for (auto &action : actions){
        if(buttons[counter - 1] && !last_frame_active_buttons[counter]){
            action();
            last_frame_active_buttons[counter] = true;
        }
        else if(!buttons[counter - 1] && last_frame_active_buttons[counter]){
            last_frame_active_buttons[counter] = false;
        }
        counter++;
    }
}


#include "controller_package/controller_reference.hpp"

void ReferenceClass::update_setpoint(std::vector<bool> *setpoint_changes, std::vector<bool> *buttons,
                                     const Eigen::Quaterniond &q, const Eigen::Vector3d &x){
    
    Eigen::Vector3d setpoint_change_lin = x; //Copy current postion
    Eigen::Quaterniond setpoint_change_att = q; //Copy current attitude

    //Function to handle button presses and allow standard operations
    handle_button_input(setpoint_change_lin, setpoint_change_att, buttons, setpoint_changes);

    setpoint_change_att.normalize(); //Make sure the quaternion setpoint is normalized

    for (int i = 0; i < 3; i++){
        if (setpoint_changes->at(i)){ //If direction is eligible, perform setpoint change
            x_d[i] = setpoint_change_lin[i];
        }
        //If any orientational direction is eligible, perform setpoint change
        //Since the quaternion directions are coupled, one active action is enough to change setpoint
        if (setpoint_changes->at(i + 3)){ 
            q_d = setpoint_change_att;
        }
    }
}


void ReferenceClass::handle_button_input(Eigen::Vector3d &setpoint_change_lin, Eigen::Quaterniond &setpoint_change_att,
                                         std::vector<bool> *buttons, std::vector<bool> *setpoint_changes){
    
    auto reset_attitude = [&](){
        setpoint_change_att = Eigen::Quaterniond(1, 0, 0, 0);
        setpoint_changes->at(4) = true; //Allow orientation setpoint to be changed
        return 0;
    };
    
    auto yaw_step = [&](){
        setpoint_change_att *= Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
                             *Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
                             *Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitZ());//If any orientational direction is eligible, perform setpoint change
        setpoint_changes->at(4) = true; //Allow orientation setpoint to be changed
        return 0;
    };

    setpoint_change_lin = setpoint_change_lin; //Remove unused variable error

    //Array with action-functions to loop through
    //Needs to correspond to active_buttons in joy_to_action.cpp
    std::vector<std::function<int()>> actions = {reset_attitude, yaw_step};
    int counter = 1;
    //Loops through button inputs and corresponding action functions
    for (auto &action : actions){
        //If button is pressed this tick but not last tick
        if(buttons->at(counter - 1) && !last_tick_active_buttons[counter]){
            action();
            last_tick_active_buttons[counter] = true;
        }
        else if(!buttons->at(counter - 1) && last_tick_active_buttons[counter]){
            last_tick_active_buttons[counter] = false;
        }
        counter++;
    }
}


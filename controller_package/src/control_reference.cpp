#include "controller_package/control_reference.hpp"

void ReferenceClass::update_setpoint(std::vector<double> *movement, std::vector<bool> *buttons,
                                     const Eigen::Quaterniond &q, const Eigen::Vector3d &x, const int &world_frame){
    Eigen::Vector3d setpoint_change_lin = x_d;    //Temp variable for positional setpoint change
    Eigen::Quaterniond setpoint_change_att = q_d; //Temp variable for attitude setpoint change

    //Call function to handle button presses. Seperate function to improve readability
    handle_button_input(setpoint_change_lin, setpoint_change_att, buttons, q);

    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector3d I_frame_linear_action = R * Eigen::Vector3d(movement->at(0), movement->at(1), movement->at(2));
    //Compensate for NED orientation if world_frame is 1
    if (world_frame == 1){
        I_frame_linear_action[1] = -I_frame_linear_action[1];
        I_frame_linear_action[2] = -I_frame_linear_action[2];
    }
    bool orientChange = false;
    for (int i = 0; i < 3; i++){
        if (I_frame_linear_action[i] != 0){
            last_frame_active_actions[i] = true;
            setpoint_change_lin[i] = x[i] + I_frame_linear_action[i];
        }
        else if (last_frame_active_actions[i]){
            last_frame_active_actions[i] = false;
            setpoint_change_lin[i] = x[i];
        }

        if (movement->at(i + 3) != 0){
            orientChange = true;
        }
    }
    
    if (orientChange){
        Eigen::Quaterniond q_relativeChange;
        std::vector<double> rotation = {movement->at(3), movement->at(4), movement->at(5)};
        if (world_frame == 1){
            rotation[0] = -rotation[0];
            rotation[1] = -rotation[1];
            rotation[2] = -rotation[2];
        }
        q_relativeChange = Eigen::AngleAxisd(rotation[0], Eigen::Vector3d::UnitX())
                          *Eigen::AngleAxisd(rotation[1], Eigen::Vector3d::UnitY())
                          *Eigen::AngleAxisd(rotation[2], Eigen::Vector3d::UnitZ());
        q_relativeChange.normalize();

        setpoint_change_att = q * q_relativeChange;
        last_frame_active_actions[4] = true;
    }
    else if (last_frame_active_actions[4]){
        setpoint_change_att = q;
        last_frame_active_actions[4] = false;
    }


    setpoint_change_att.normalize();
    q_d = setpoint_change_att;  //Update setpoint
    x_d = setpoint_change_lin;  //Update setpoint
}


void ReferenceClass::handle_button_input(Eigen::Vector3d &setpoint_change_lin, Eigen::Quaterniond &setpoint_change_att,
                                         std::vector<bool> *buttons, const Eigen::Quaterniond &q){
    
    auto return_to_surface = [&](){
        setpoint_change_att = Eigen::Quaterniond(1, 0, 0, 0);
        setpoint_change_lin.z() = 0;
        return 0;
    };

    auto reset_attitude = [&](){
        setpoint_change_att = Eigen::Quaterniond(1, 0, 0, 0);
        return 0;
    };

    auto surge_step = [&](){
        Eigen::Matrix3d R = q.toRotationMatrix();
        Eigen::Vector3d surge = R * Eigen::Vector3d(5, 0, 0);
        setpoint_change_lin += surge;
        return 0;
    };

    auto yaw_step = [&](){
        setpoint_change_att = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
                             *Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
                             *Eigen::AngleAxisd(M_PI / 3, Eigen::Vector3d::UnitZ());
        return 0;
    };

    //Array with action-functions to loop through
    //Needs to correspond to active_buttons in joy_to_action.cpp
    std::vector<std::function<int()>> actions = {reset_attitude, return_to_surface, surge_step, yaw_step};
    int counter = 1;
    //Loops through button inputs and corresponding action functions
    for (auto &action : actions){
        if(buttons->at(counter - 1) && !last_frame_active_buttons[counter]){
            action();
            last_frame_active_buttons[counter] = true;
        }
        else if(!buttons->at(counter - 1) && last_frame_active_buttons[counter]){
            last_frame_active_buttons[counter] = false;
        }
        counter++;
    }
}


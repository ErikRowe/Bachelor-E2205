#include "controller_package/control_reference.hpp"

void ReferenceClass::update_setpoint(std::vector<double> *movement, std::vector<bool> *buttons,
                                     const Eigen::Quaterniond &q, const Eigen::Vector3d &x){
    Eigen::Vector3d setpoint_change_lin;    //Temp variable for positional setpoint change
    Eigen::Quaterniond setpoint_change_att; //Temp variable for attitude setpoint change
    setpoint_change_lin = x_d;
    setpoint_change_att = q_d;

    //Call function to handle button presses. Seperate function to improve readability
    handle_button_input(setpoint_change_lin, setpoint_change_att, buttons, q);

    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector3d I_frame_linear_action = R * Eigen::Vector3d(movement->at(0), movement->at(1), movement->at(2));
    bool orientChange = false;
    for (int i = 0; i < 3; i++){
        if (I_frame_linear_action[i] != 0){
            last_frame_active_actions[i] = true;
            setpoint_change_lin[i] = x[i] + I_frame_linear_action[i];
        }
        else if (last_frame_active_actions[i] == true){
            last_frame_active_actions[i] = false;
            setpoint_change_lin[i] = x[i];
        }

        if (movement->at(i + 3) != 0){
            orientChange = true;
        }
    }
    
    if (orientChange){
        Eigen::Quaterniond q_relativeChange;
        q_relativeChange = Eigen::AngleAxisd(movement->at(3), Eigen::Vector3d::UnitX())
                          *Eigen::AngleAxisd(movement->at(4), Eigen::Vector3d::UnitY())
                          *Eigen::AngleAxisd(movement->at(5), Eigen::Vector3d::UnitZ());
        q_relativeChange.normalize();

        setpoint_change_att = q * q_relativeChange;
        last_frame_active_actions[4] = true;
    }
    else if (last_frame_active_actions[4] == true){
        setpoint_change_att = q;
        last_frame_active_actions[4] = false;
    }


    setpoint_change_att.normalize();
    q_d = setpoint_change_att;  //Update setpoint
    x_d = setpoint_change_lin;  //Update setpoint
}


void ReferenceClass::handle_button_input(Eigen::Vector3d &setpoint_change_lin, Eigen::Quaterniond &setpoint_change_att,
                                         std::vector<bool> *buttons, const Eigen::Quaterniond &q){
    
    auto reset_attitude = [&](){
        setpoint_change_att = Eigen::Quaterniond(1, 0, 0, 0);
        return 0;
    };

    auto return_to_surface = [&](){
        setpoint_change_att = Eigen::Quaterniond(1, 0, 0, 0);
        setpoint_change_lin.z() = 0;
        return 0;
    };

    auto surge_step = [&](){
        Eigen::Matrix3d R = q.toRotationMatrix();
        Eigen::Vector3d surge = R * Eigen::Vector3d(5, 0, 0);
        setpoint_change_lin += surge;
        return 0;
    };

    //Array with action-functions to loop through
    std::vector<std::function<int()>> actions = {reset_attitude, return_to_surface, surge_step};
    int counter = 0;
    //Loops through button inputs and corresponding action functions
    for (auto &action : actions){
        if(buttons->at(counter) && !last_frame_active_buttons[counter]){
            action();
            last_frame_active_buttons[counter] = true;
        }
        else if(!buttons->at(counter) && last_frame_active_buttons[counter]){
            last_frame_active_buttons[counter] = false;
        }
        counter++;
    }

    if(buttons->at(0)){
        reset_attitude();
    }
}


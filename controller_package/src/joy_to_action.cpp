#include "controller_package/joy_to_action.h"

UserJoystickInput::UserJoystickInput(){
        roll_scaling    = 1.0;
        pitch_scaling   = 1.0;
        yaw_scaling     = 1.0;
}

void UserJoystickInput::joystickToActions(const std::vector<float> axes_input, const std::vector<int> button_input){
    std::map<std::string, double> axes;
    std::map<std::string, double> buttons;

    for (int i = 0; i < 8; i++){
        axes.insert({axes_mapping[i], axes_input[i]});
    }
    for (int j = 0; j < 11; j++){
        buttons.insert({button_mapping[j], button_input[j]});
    }

    double surge    = axes.find("LStickUD")->second * surge_scaling;
    double sway     = axes.find("LStickLR")->second * sway_scaling;
    double heave    = (axes.find("LTrigger")->second - axes.find("RTrigger")->second)/2 * heave_scaling;
    double roll     = (buttons.find("RBumper")->second - buttons.find("LBumper")->second)/2 * roll_scaling;
    double pitch    = axes.find("RStickUD")->second * pitch_scaling;
    double yaw      = axes.find("RStickLR")->second * yaw_scaling;
    actions = {surge, sway, heave, roll, pitch, yaw};
}

void UserJoystickInput::update_params(double surge_s, double sway_s, double heave_s)
{
    surge_scaling   = surge_s;
    sway_scaling    = sway_s;
    heave_scaling   = heave_s;
}
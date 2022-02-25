#include "controller_package/joy_to_action.h"

UserJoystickInput::UserJoystickInput(){
        surge_scaling   = 1.0;
        sway_scaling    = 1.0;
        heave_scaling   = 1.0;
        roll_scaling    = 1.0;
        pitch_scaling   = 1.0;
        yaw_scaling     = 1.0;
}

double * UserJoystickInput::joystickToActions(const std::vector<float> axes_input, const std::vector<int> button_input){
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
    double heave    = (axes.find("RTrigger")->second - axes.find("LTrigger")->second)/2 * heave_scaling;
    double roll     = (buttons.find("RBumper")->second - buttons.find("LBumper")->second)/2 * roll_scaling;
    double pitch    = axes.find("RStickUD")->second * pitch_scaling;
    double yaw      = axes.find("RStickLR")->second * yaw_scaling;

    static double actions[6] = {surge, sway, heave, roll, pitch, yaw};
    return actions;
}
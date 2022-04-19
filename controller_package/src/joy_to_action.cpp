#include "controller_package/joy_to_action.hpp"

void UserJoystickInput::joystickToActions(const std::vector<float> axes_input, const std::vector<int> button_input){
    std::map<std::string, double> axes;
    std::map<std::string, bool> buttons;

    for (int i = 0; i < 8; i++){
        axes.insert({axes_mapping[i], axes_input[i]});
    }
    for (int j = 0; j < 11; j++){
        buttons.insert({button_mapping[j], (bool)button_input[j]});
    }

    double surge    = axes.find("LStickUD")->second * surge_scaling;
    double sway     = axes.find("LStickLR")->second * sway_scaling;
    double heave    = (axes.find("LTrigger")->second - axes.find("RTrigger")->second)/2 * heave_scaling;
    double roll     = ((int)buttons.find("RBumper")->second - (int)buttons.find("LBumper")->second)/2;
    double pitch    = axes.find("RStickUD")->second;
    double yaw      = axes.find("RStickLR")->second;
    movement = {surge, sway, heave, roll, pitch, yaw};

    bool reset_attitude     = buttons.find("X")->second;
    bool return_to_surface  = buttons.find("B")->second;
    bool surge_step         = buttons.find("A")->second;
    bool yaw_step           = buttons.find("Y")->second;
    return_to_surface = 0;
    surge_step = 0;
    active_buttons = {reset_attitude, return_to_surface, surge_step, yaw_step};
}

void UserJoystickInput::update_params(double surge_s, double sway_s, double heave_s)
{
    surge_scaling   = surge_s;
    sway_scaling    = sway_s;
    heave_scaling   = heave_s;
}
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

    double surge    = axes.find("LStickUD")->second;
    double sway     = axes.find("LStickLR")->second;
    double heave    = (axes.find("RTrigger")->second - axes.find("LTrigger")->second)/2;
    double roll     = (double)((int)buttons.find("RBumper")->second - (int)buttons.find("LBumper")->second);
    double pitch    = axes.find("RStickUD")->second;
    double yaw      = axes.find("RStickLR")->second;
    movement = {surge, sway, heave, roll, pitch, yaw};

    bool reset_attitude     = buttons.find("A")->second;
    bool toggle_logging      = buttons.find("B")->second;
    active_buttons = {reset_attitude, toggle_logging};
}
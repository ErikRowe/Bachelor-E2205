#include <iostream>
#include <map>
#include <vector>

class UserJoystickInput{
    public:
        UserJoystickInput();
        const std::string axes_mapping[8] = {"LStickLR", "LStickUD", "LTrigger", "RStickLR",
                                             "RStickUD", "RTrigger", "DpadLR", "DPadUD"};

        const std::string button_mapping[11] = {"A", "B", "X", "Y", "LBumper", "RBumper",
                                                "Select", "Start", "XBoxButton",
                                                "LStickButton", "RStickButton"};

    double * joystickToActions(const std::vector<float> axes_input, const std::vector<int> button_input);

    private:
        double surge_scaling;
        double sway_scaling;
        double heave_scaling;
        double roll_scaling;
        double pitch_scaling;
        double yaw_scaling;

};
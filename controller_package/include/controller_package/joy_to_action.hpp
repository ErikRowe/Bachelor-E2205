#include <iostream>
#include <map>
#include <vector>

class UserJoystickInput{
    public:
        //Order of Joy axes array and corresponding joystick on the XBox One Controller
        const std::string axes_mapping[8] = {"LStickLR", "LStickUD", "LTrigger", "RStickLR",
                                             "RStickUD", "RTrigger", "DpadLR", "DPadUD"};

        //Order of Joy button array and corresponding button on the XBox One Controller
        const std::string button_mapping[11] = {"A", "B", "X", "Y", "LBumper", "RBumper",
                                                "Select", "Start", "XBoxButton",
                                                "LStickButton", "RStickButton"};


        /**
         * @brief Converts joystick input to desired action and updates actions variable
         * 
         * @param axes_input Float array with analog stick input from Joy
         * @param button_input Int array with button input from Joy
         * 
         */
        void joystickToActions(const std::vector<float> axes_input, const std::vector<int> button_input);

        std::vector <double> movement = {0, 0, 0, 0, 0, 0};                          //Stores information about current user movement
        std::vector <bool> active_buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};       //Stores information about current active button presses
};
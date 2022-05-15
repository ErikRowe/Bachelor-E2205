#include "controller_package/common.hpp"

class SetpointHolderClass{
    public:

        /**
        * @brief Reads user actions and converts to relative setpoint change
        * 
        * @param setpoint_changes   Movement axes eligible for setpoint changes (by joystick)
        * @param buttons            Button presses
        * @param q                  Current attitude in quaternion representation
        * @param x                  Current position in world frame
        * 
        */
        void update_setpoint(Eigen::Vector6d &setpoint_changes, std::vector<bool> buttons,
                             const Eigen::Quaterniond &q, const Eigen::Vector3d &x);

        Eigen::Vector3d x_d = Eigen::Vector3d::Zero();              //Setpoint position
        Eigen::Quaterniond q_d = Eigen::Quaterniond(1, 0, 0, 0);    //Setpoint attitude in quaternions

    private:
        /**
        * @brief Reads button input status and handles accordingly. Called  and used from update_setpoint
        * 
        * @param setpoint_change_lin    Temporary position setpoint change variable. Locally used in update_setpoint
        * @param setpoint_change_att    Temporary attitude setpoint change variable. Locally used in update_setpoint
        * @param buttons                Array with button input values
        * @param setpoint_changes       Movement axes eligible for setpoint changes (by joystick)
        * 
        */
        void handle_button_input(Eigen::Vector3d &setpoint_change_lin, Eigen::Quaterniond &setpoint_change_att,
                                 std::vector<bool> buttons, Eigen::Vector6d  &setpoint_changes);

        bool last_tick_active_buttons[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //Tracker if last frame had button input
};
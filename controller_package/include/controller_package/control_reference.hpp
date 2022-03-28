#include "controller_package/common.hpp"

class ReferenceClass{
    public:

        /**
        * @brief Reads user actions and converts to relative setpoint change
        * 
        * @param movement   Surge, sway, heave, roll, pitch, yaw
        * @param buttons    Button presses
        * @param q          Current attitude in quaternion representation
        * @param x          Current position in world frame
        * 
        */
        void update_setpoint(std::vector<double> *movement, std::vector<bool> *buttons,
                             const Eigen::Quaterniond &q, const Eigen::Vector3d &x);

        Eigen::Vector3d x_d = Eigen::Vector3d::Zero();              //Setpoint position
        Eigen::Quaterniond q_d = Eigen::Quaterniond(1, 0, 0, 0);    //Setpoint attitude in quaternions

    private:
        /**
        * @brief Reads button input status and handles accordingly. Called  and used from update_setpoint
        * 
        * @param setpoint_change_lin T  emporary position setpoint change variable. Locally used in update_setpoint
        * @param setpoint_change_att    Temporary attitude setpoint change variable. Locally used in update_setpoint
        * @param button_input           Array with button input values
        * 
        */
        void handle_button_input(Eigen::Vector3d &setpoint_change_lin, Eigen::Quaterniond &setpoint_change_att,
                                 std::vector<bool> *buttons, const Eigen::Quaterniond &q);

        bool last_frame_active_actions[4];  //Tracker if last frame had action input
        bool last_frame_active_buttons[11]; //Tracker if last frame had button input
};
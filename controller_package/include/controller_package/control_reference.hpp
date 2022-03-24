#include "controller_package/common.hpp"

class ReferenceClass{
    public:

        /**
        * @brief Reads user actions and converts to relative setpoint change
        * 
        * @param movement   Surge, sway, heave, roll, pitch, yaw
        * @param q          Current attitude in quaternion representation
        * @param x          Current position in world frame
        * 
        */
        void changeSetPoint(std::vector<double> movement, const Eigen::Quaterniond &q,
                            const Eigen::Vector3d &x);

        Eigen::Vector3d x_d = Eigen::Vector3d::Zero();              //Setpoint position
        Eigen::Quaterniond q_d = Eigen::Quaterniond(1, 0, 0, 0);    //Setpoint attitude in quaternions
        bool last_frame_active_actions[4];  //Tracker if last frame had action input
};
#include "controller_package/common.hpp"

class Actuation
{
    public:
        std::vector<int> Thruster_spin_direction = {1, 1, -1, -1, 1, -1, -1, 1}; // This relates to the actuators and wether they run clockwize=-1 or anti-clockwize=1
        std::vector<int> Thruster_install_angles = {-45, 45, -135, 135};         // This relates to the angle at which the four angled rotors are placed
        const double local_Pi = 3.14;                                            // constant PI
        

        /**
         * @brief Create a matrix that contains actuation for each thruster
         * 
         * @param tau // vector containing wanted angular and linear movement
         * @return Eigen::Vector8d containing actuation for each thruster
         */
        Eigen::Vector8d build_actuation(Eigen::Vector6d tau);

        Actuation();

    private:
        Eigen::Matrix38d LENGTHS_THRUSTERS;         // Length of thrusters to center of drone. This needs to be verified
        Eigen::Vector3d e_1 = {1, 0, 0};            // Expect it to be î
        Eigen::Vector3d e_3 = {0, 0, 1};            // k

        Eigen::MatrixXd B_{6, 8};                   // Geometry matrix for the thrusters
        Eigen::MatrixXd B_pinv_;                    // The inverse of the B_ matrix
};
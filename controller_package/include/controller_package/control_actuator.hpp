#include <functional>
#include <memory>
#include <stdlib.h>
#include <iostream>
#include <vector>

#include <eigen3/Eigen/Dense>
#include "sophus/geometry.hpp"
#include "bluerov_interfaces/msg/actuator_input.hpp"

namespace Eigen{
    typedef Eigen::Matrix<double, 6, 1> vector6d;
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
    typedef Eigen::Matrix<double, 8, 1> Vector8d;
    typedef Eigen::Matrix<double, 3, 8> Matrix38d;
}

class Actuation
{
    public:
        std::vector<int> MIXER = {1, 1, -1, -1, 1, -1, -1, 1}; // This relates to the actuators and wether they run clockwize or anti-clockwize
        std::vector<int> ALPHA = {-45, 45, -135, 135}; // This relates to the angle at which the four angled rotors are placed
        const double kPi = Sophus::Constants<double>::pi();
        

        /**
         * @brief Create a actutaion message object
         * 
         * @param tau 
         * @return Eigen::Matrix8d 
         */
        Eigen::Vector8d build_actuation(Eigen::vector6d tau);

    private:
        Eigen::Matrix38d LENGTHS_THRUSTERS;

        Eigen::Vector3d e_1 = {1, 0, 0};
        Eigen::Vector3d e_3 = {0, 0, 1};

        Eigen::MatrixXd B_{6, 8}; // Need to figure out what this is
        Eigen::MatrixXd B_pinv_; //This too
};
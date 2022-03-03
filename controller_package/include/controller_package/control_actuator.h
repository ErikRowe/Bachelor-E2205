#include <functional>
#include <memory>
#include <stdlib.h>
#include <iostream>
#include <vector>

#include <eigen3/Eigen/Dense>
#include "sophus/geometry.hpp"

namespace Eigen{
    typedef Eigen::Matrix<double, 6, 1> vector6d;
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
}



class Actuation
{
    public:
        std::vector<int> MIXER = {1, 1, -1, -1, 1, -1, -1, 1}; // This relates to the actuators and wether they are clockwize or anti-clockwize
        std::vector<int> ALPHA = {-45, 45, -135, 135}; // This relates to the angle at which the four angled rotors are placed
        
        /**
         * @brief actuation builder function
         * 
         * @param movement_data 
         * @return double 
         */
        double actuation(std::array<double, 6> movement_data);
    private:
        /**
         * @brief transform into pwm signal
         * 
         * @return double 
         */
        double make_pwm();
};
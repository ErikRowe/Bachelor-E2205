#include "controller_package/common.hpp"
#include <iostream>
#include <fstream>
using namespace std;

class LoggingClass{
    public:

        /**
         * @brief 
         * 
         */
        void data_logger(Eigen::Vector6d tau, Eigen::Vector6d z, Eigen::Quaterniond q, Eigen::Quaterniond q_d, 
                         Eigen::Vector3d x, Eigen::Vector3d x_d, Eigen::Vector6d v, double time);
};
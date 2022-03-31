#include "controller_package/common.hpp"
#include <iostream>
#include <fstream>
using namespace std;

class LoggingClass{
    public:
        void data_logger(Eigen::Vector6d data1, Eigen::Vector6d data2, Eigen::Vector3d data3);
};
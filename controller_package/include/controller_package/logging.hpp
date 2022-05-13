#include "controller_package/common.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <bits/stdc++.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <ctime>
#include <stdio.h>              //sprintf 
using namespace std;

class LoggingClass{
    public:
        time_t now = time(0);
        char buffer[100];
        char buffer2[100];
        char buffer3[100];
        char buffer4[100];
        char folder_char[100];
        bool man_log_active;
        int teller = 0;
        bool OnlyOnce = true;

        explicit LoggingClass();

        /**
         * @brief Logging to file when node is active
         * 
         * @param tau 
         * @param z 
         * @param q 
         * @param q_d 
         * @param x 
         * @param x_d 
         * @param v 
         * @param joy_axes_input 
         * @param time 
         * @param folder 
         */
        void data_logger(Eigen::Vector6d tau, Eigen::Vector6d z, Eigen::Quaterniond q, Eigen::Quaterniond q_d, 
                         Eigen::Vector3d x, Eigen::Vector3d x_d, Eigen::Vector6d v, std::vector<float> joy_axes_input, int64_t time, char folder[100]);


        /**
         * @brief Logging to file when manuelly activated
         * 
         * @param tau 
         * @param z 
         * @param q 
         * @param q_d 
         * @param x 
         * @param x_d 
         * @param v 
         * @param joy_axes_input 
         * @param teller 
         * @param folder 
         */
        void data_logger2(Eigen::Vector6d tau, Eigen::Vector6d z, Eigen::Quaterniond q, Eigen::Quaterniond q_d, 
                         Eigen::Vector3d x, Eigen::Vector3d x_d, Eigen::Vector6d v, std::vector<float> joy_axes_input, int teller, char folder[100]);
};
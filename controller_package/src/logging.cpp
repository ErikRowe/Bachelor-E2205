#include "controller_package/logging.hpp"


void LoggingClass::data_logger(Eigen::Vector6d data1, Eigen::Vector6d data2, Eigen::Vector3d data3){
    ofstream outfile;
    outfile.open("test_tau_file1.xlsx", ios::app);
    //While loop filing and spliting the tau vector coeffisients
    int i = 0;  
    while (i < 6){
        outfile << data1(i) << ", ";
        i++;
    }
    outfile << endl;
    outfile.close();

    outfile.open("test_z_file.xlsx", ios::app);
    //While loop filing and splitting the error vector(z) coeffisients
    int m = 0;
    while(m < 6){
        outfile << data2(m) << ", ";
        m++;
    }
    outfile << endl;
    outfile.close();

    outfile.open("test_setpoint_position.xlsx", ios::app);
    //While loop filing and splitting the pos setpoint(x_d) coeffisients
    int n = 0;
    while(n < 3){
        outfile << data3[n] << ", ";
        n++;
    }
    outfile << endl;
    outfile.close();
}

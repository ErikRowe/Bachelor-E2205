#include "controller_package/logging.hpp"

bool OnlyOnce = true;


void LoggingClass::data_logger(Eigen::Vector6d tau, Eigen::Vector6d z, Eigen::Quaterniond q, Eigen::Quaterniond q_d, 
                               Eigen::Vector3d x, Eigen::Vector3d x_d, Eigen::Vector6d v, std::vector<float> joy_axes_input, int64_t time)
    {
    ofstream outfile;
    outfile.open("OneWorld_OneFile.xlsx", ios::app);                                 //Opening file            
    int i = 0;
    if (OnlyOnce == true)
    {
    std::string header[38];
    header[0] = "Time";
    header[1] = "Tau";
    header[7] = "Z";
    header[13] = "Q";
    header[17] = "Q_D";
    header[21] = "X";
    header[24] = "X_D";
    header[27] = "V";
    header[33] = "Axes";
    const std::string Coeffs[41] = {"Time", "T1", "T2", "T3", "T4", "T5", "T6", "Z1", "Z2", "Z3", "Z4", "Z5", "Z6", 
                                   "Q1", "Q2", "Q3", "Q4", "Q_D1", "Q_D2", "Q_D3", "Q_D4", 
                                   "X1", "X2", "X3", "X_D1", "X_D2", "X_D3", "V1", "V2", "V3", "V4", "V5", "V6", "A1", "A2", "A3", "A4", "A5", "A6", "A7", "A8"
                                   };
    while (i < 41){
        outfile << header[i] << ", ";
        i++;
    }
    outfile << endl;

    i = 0;
    while (i < 41){
        outfile << Coeffs[i] << ", ";
        i++;
    }
    outfile << endl;
    OnlyOnce = false;
    }


    outfile << time << ", ";
    i = 0;  
    while (i < 6)                                                                   //Logging tau parameters to file               
    {
        outfile << tau(i) << ", ";
        i++;
    }

    i = 0;
    while(i < 6)                                                                    //Logging error parameters to file                                             
    {
        outfile << z(i) << ", ";
        i++;
    }

    i = 0;
    while(i < 4)                                                                     //Logging attitude parameters to file               
    {
        outfile << q.coeffs()(i) << ", ";
        i++;
    }

    i = 0;
    while(i < 4)                                                                     //Logging desired attitude parameters to file                
    {
        outfile << q_d.coeffs()(i) << ", ";
        i++;
    }

    i = 0;
    while(i < 3)                                                                     //Logging position parameters to file                
    {
        outfile << x(i) << ", ";
        i++;
    }

    i = 0;
    while(i < 3)                                                                      //Logging desired position parameters to file              
    {
        outfile << x_d[i] << ", ";
        i++;
    }

    i = 0;
    while(i < 6)                                                                     //Logging velocity parameters to file
    {
        outfile << v(i) << ", ";
        i++;
    }


    i = 0;
    while(i < 8)                                                                     //Logging joy axes input parameters to file
    {
        outfile << joy_axes_input[i] << ", ";
        i++;
    }

    outfile << endl;
    outfile.close();                                                                  //Closing file
    }

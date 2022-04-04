#include "controller_package/logging.hpp"


void LoggingClass::data_logger(Eigen::Vector6d tau, Eigen::Vector6d z, Eigen::Quaterniond q, Eigen::Quaterniond q_d, 
                               Eigen::Vector3d x, Eigen::Vector3d x_d, Eigen::Vector6d v, double time)
    {
    ofstream outfile;
    outfile.open("tau.xlsx", ios::app);                                             //Logging tau parameters to file
    outfile << time << ", ";
    int i = 0;  
    while (i < 6)                                                                                   
    {
        outfile << tau(i) << ", ";
        i++;
    }
    outfile << endl;
    outfile.close();

    outfile.open("error_vector_z.xlsx", ios::app);                                  //Logging error parameters to file
    outfile << time << ", ";
    i = 0;
    while(i < 6)                                                                                    
    {
        outfile << z(i) << ", ";
        i++;
    }
    outfile << endl;
    outfile.close();

    outfile.open("attitude_q.xlsx", ios::app);                                      //Logging attitude parameters to file
    outfile << time << ", ";
    i = 0;
    while(i < 4)                                                                                    
    {
        outfile << q.coeffs()(i) << ", ";
        i++;
    }
    outfile << endl;
    outfile.close();

    outfile.open("desired_attitude_q_d.xlsx", ios::app);                            //Logging desired attitude parameters to file
    outfile << time << ", ";
    i = 0;
    while(i < 4)                                                                                    
    {
        outfile << q_d.coeffs()(i) << ", ";
        i++;
    }
    outfile << endl;
    outfile.close();

    outfile.open("position_x.xlsx", ios::app);                                      //Logging position parameters to file
    outfile << time << ", ";
    i = 0;
    while(i < 3)                                                                                    
    {
        outfile << x(i) << ", ";
        i++;
    }
    outfile << endl;
    outfile.close();

    outfile.open("desired_position_x_d.xlsx", ios::app);                            //Logging desired position parameters to file
    outfile << time << ", ";
    i = 0;
    while(i < 3)                                                                                    
    {
        outfile << x_d[i] << ", ";
        i++;
    }
    outfile << endl;
    outfile.close();

    outfile.open("velocity_v.xlsx", ios::app);                                      //Logging velocity parameters to file
    outfile << time << ", ";
    i = 0;
    while(i < 3)
    {
        outfile << v(i) << ", ";
        i++;
    }
    outfile << endl;
    outfile.close();
}

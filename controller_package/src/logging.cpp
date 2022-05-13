#include "controller_package/logging.hpp"






LoggingClass::LoggingClass()
{

    //Ny mappe, med timestamp, lages hver gang noden aktiveres
    
    sprintf(buffer, "Data_folder/Node_activated %s",ctime(&now));          //Creates a character string (buffer) to use as directory to the active node folder   
    mkdir(buffer , 0777);                                                  // Creates i folder in choosen directory
    

    
    sprintf(buffer2, "%s/Intervalls", buffer);                             // Creates a character string (buffer2) to use as a director to the intervall folder
    mkdir(buffer2, 0777);                                                  // Creates i folder in choosen directory
    
}



void LoggingClass::data_logger(Eigen::Vector6d tau, Eigen::Vector6d z, Eigen::Quaterniond q, Eigen::Quaterniond q_d, 
                               Eigen::Vector3d x, Eigen::Vector3d x_d, Eigen::Vector6d v, std::vector<float> joy_axes_input, 
                               int64_t time, char folder[100])
    {
    sprintf(buffer3,"%s/Node_activated.xlsx", folder);
    //sprintf(buffer4,"%s/Node_activated.xlsx", folder);
    ofstream outfile;
    
    outfile.open(buffer3, ios::app);  
    
    
                               //Opening file            
    int i = 0;
    if (OnlyOnce == true)
    {
    std::string header[41];
    header[0] = "Time";
    header[1] = "Tau";
    header[7] = "Z";
    header[13] = "Q";
    header[17] = "Q_D";
    header[21] = "X";
    header[24] = "X_D";
    header[27] = "V";
    header[33] = "Axes";
    const std::string Coeffs[41] = {"Time", "T1", "T2", "T3", "T4", "T5", "T6", "X", "Y", "Z", "Z_1", "Z_2", "Z_3", 
                                   "Re", "Im_1", "Im_2", "Im_3", "Re", "Im_1", "Im_2", "Im_3", 
                                   "X", "Y", "Z", "X", "Y", "Z", "U", "V", "W", "P", "Q", "R", "Left/Right", "Up/Down", "LTrigger", 
                                   "RStickLR", "RStickUD", "RTrigger", "DPadLR", "PPadUD"
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






void LoggingClass::data_logger2(Eigen::Vector6d tau, Eigen::Vector6d z, Eigen::Quaterniond q, Eigen::Quaterniond q_d, 
                               Eigen::Vector3d x, Eigen::Vector3d x_d, Eigen::Vector6d v, std::vector<float> joy_axes_input, 
                               int teller, char folder[100])
    {
    int time = 0;
    sprintf(buffer4,"%s/Node_intervall_%d.xlsx", folder, teller);
    //sprintf(buffer4,"Data_folder/NODE/Intervall/Node_intervall_%d.xlsx", teller);
    ofstream outfile1;

    outfile1.open(buffer4, ios::app);  
    
    
                               //Opening file            
    int i = 0;
    if (OnlyOnce == true)
    {
    std::string header[41];
    header[0] = "Time";
    header[1] = "Tau";
    header[7] = "Z";
    header[13] = "Q";
    header[17] = "Q_D";
    header[21] = "X";
    header[24] = "X_D";
    header[27] = "V";
    header[33] = "Axes";
    const std::string Coeffs[41] = {"Time", "T1", "T2", "T3", "T4", "T5", "T6", "X", "Y", "Z", "Z_1", "Z_2", "Z_3", 
                                   "Re", "Im_1", "Im_2", "Im_3", "Re", "Im_1", "Im_2", "Im_3", 
                                   "X", "Y", "Z", "X", "Y", "Z", "U", "V", "W", "P", "Q", "R", "Left/Right", "Up/Down", "LTrigger", 
                                   "RStickLR", "RStickUD", "RTrigger", "DPadLR", "PPadUD"
                                   };
    while (i < 41){
        outfile1 << header[i] << ", ";
        i++;
    }
    outfile1 << endl;

    i = 0;
    while (i < 41){
        outfile1 << Coeffs[i] << ", ";
        i++;
    }
    outfile1 << endl;
    OnlyOnce = false;
    }


    outfile1 << time << ", ";
    i = 0;  
    while (i < 6)                                                                   //Logging tau parameters to file               
    {
        outfile1 << tau(i) << ", ";
        i++;
    }

    i = 0;
    while(i < 6)                                                                    //Logging error parameters to file                                             
    {
        outfile1 << z(i) << ", ";
        i++;
    }

    i = 0;
    while(i < 4)                                                                     //Logging attitude parameters to file               
    {
        outfile1 << q.coeffs()(i) << ", ";
        i++;
    }

    i = 0;
    while(i < 4)                                                                     //Logging desired attitude parameters to file                
    {
        outfile1 << q_d.coeffs()(i) << ", ";
        i++;
    }

    i = 0;
    while(i < 3)                                                                     //Logging position parameters to file                
    {
        outfile1 << x(i) << ", ";
        i++;
    }

    i = 0;
    while(i < 3)                                                                      //Logging desired position parameters to file              
    {
        outfile1 << x_d[i] << ", ";
        i++;
    }

    i = 0;
    while(i < 6)                                                                     //Logging velocity parameters to file
    {
        outfile1 << v(i) << ", ";
        i++;
    }


    i = 0;
    while(i < 8)                                                                     //Logging joy axes input parameters to file
    {
        outfile1 << joy_axes_input[i] << ", ";
        i++;
    }

    outfile1 << endl;
    outfile1.close();                                                                  //Closing file
    }

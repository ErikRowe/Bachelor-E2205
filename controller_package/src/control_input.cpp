#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>

using std::placeholders::_1;
namespace Eigen{
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
}


//Analog stick 1: Movement in local xy (horizontal)

//Analog stick 2: Pitch, Roll 

//DPad: Up/Down z-axis 

//Triggers: Yaw 

class UserInput{
    public:
        Eigen::Vector3d x_d;
        Eigen::Quaterniond q_d;

        double surge_scaling = 1.0;
        double sway_scaling = 1.0;
        double heave_scaling = 1.0;
        double roll_scaling = 1.0;
        double pitch_scaling = 1.0;
        double yaw_scaling = 1.0;

        double surge;
        double sway;
        double heave;
        double roll;
        double pitch;
        double yaw;

    private:

        const char *axes_mapping[8] = {"LStickLR", "LStickUD", "LTrigger", "RStickLR",
                                      "RStickUD", "RTrigger", "DpadLR", "DPadUD"};

        const char *button_mapping[11] = {"A", "B", "X", "Y", "LBumper", "RBumper",
                                         "Select", "Start", "XBoxButton",
                                         "LStickButton", "RStickButton"};

        

    void userInputToAction(double axes_input[8], int button_input[11]){
        std::map<char, double> axes;
        std::map<char, int> buttons;

        for (int i = 0; i < 8; i++){
            axes.insert({*axes_mapping[i], axes_input[i]});
        }
        for (int j = 0; j < 11; j++){
            buttons.insert({*button_mapping[j], button_input[j]});
        }

        surge   = axes.find(*"LStickUD")->second * surge_scaling;
        sway 	= axes.find(*"LStickLR")->second * sway_scaling;
		heave 	= (axes.find(*"RTrigger")->second - axes.find(*"LTrigger")->second)/2 * heave_scaling;
		roll 	= (buttons.find(*"RBumper")->second - buttons.find(*"LBumper")->second) * roll_scaling;
		pitch 	= axes.find(*"RStickUD")->second * pitch_scaling;
		yaw 	= axes.find(*"RStickLR")->second * yaw_scaling;
    }
};


int signum(double x){
    if(x >= 0){
        return 1;
    }
    return -1;
}

Eigen::Vector6d error_vector(Eigen::Quaterniond q, Eigen::Quaterniond q_d,
                             Eigen::Vector3d x, Eigen::Vector3d x_d){
    Eigen::Vector3d x_tilde = x - x_d;
    Eigen::Quaterniond q_tilde = q_d.conjugate()*q;
    q_tilde.normalize();
    int mu = signum(q_tilde.w());

    Eigen::Vector6d z(6);
    z << x_tilde, mu * q_tilde.vec();
    return z;
}

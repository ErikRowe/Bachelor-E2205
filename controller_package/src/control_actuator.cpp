
#include <controller_package/control_actuator.h>

double Actuation::actuation(std::array<double, 6> movement_data){
    std::array<double, 3> linear_mov = {    // Stores linear movement in array
        std::get<0>(movement_data),
        std::get<1>(movement_data),
        std::get<2>(movement_data)
    };
    std::array<double, 3> angular_mov = {   // Stores angular movement in array
        std::get<3>(movement_data),
        std::get<4>(movement_data),
        std::get<5>(movement_data)
    };
    double pwm_signal = make_pwm();
    return pwm_signal;
}

double Actuation::make_pwm(){
    return 6.55 + 2.11;
}


// class Actuation
// {
// private:
//     double make_pwm(){
//         return 6.55+2.11;
//     }
// public:
//     double actuation(std::array<double, 6> movement_data)
//     {
//         std::array<double, 3> linear_mov = {    // Stores linear movement in array
//             std::get<0>(movement_data),
//             std::get<1>(movement_data),
//             std::get<2>(movement_data)
//         };
//         std::array<double, 3> angular_mov = {   // Stores angular movement in array
//             std::get<3>(movement_data),
//             std::get<4>(movement_data),
//             std::get<5>(movement_data)
//         };
//         double pwm_signal = make_pwm();
//         return pwm_signal;
//     }
// };

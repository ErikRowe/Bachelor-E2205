#include <functional>
#include <memory>
#include <stdlib.h>

class Actuation
{
    public:
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
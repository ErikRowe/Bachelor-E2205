#include <eigen3/Eigen/Dense>
#include <iostream>

using std::placeholders::_1;
namespace Eigen{
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;
}

class PIDInputClass{
    public:
        PIDInputClass();
        Eigen::Vector3d x_d;
        Eigen::Quaterniond q_d;
        Eigen::Vector3d x;
        Eigen::Quaterniond q;

        void readPosAtt(double x_global, double y_global, double z_global);
        Eigen::Vector6d getErrorVector();
        void changeSetPoint(double actions[6]);

    private:
        bool last_frame_active_actions[6];

        int signum(double x);
};
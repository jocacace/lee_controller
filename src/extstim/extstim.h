#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "boost/thread.hpp"
#include <Eigen/Eigen>
#include "gazebo_msgs/ModelStates.h"
#include "std_msgs/Float32MultiArray.h"
#include "tf/tf.h"
#include <Eigen/Geometry>
#include "../utils.h"

using namespace std;
using namespace Eigen;

class EXTSTIM {

    public:

        EXTSTIM();

        void set_dyn_param( double mass_, double gravity_, Eigen::Matrix3d Ib_ );
        void set_gains( double zita_, double omega_lin_, double omega_z, double omega_tor );
        
        void stim();       
        void reset();
        void estimation(Eigen::Matrix3d eta, Eigen::Vector3d eta_dot, Eigen::Vector3d vel, double force, Eigen::Vector3d tau_b, double Tc,  Vector3d & extF, Vector3d & extT);
        void run();
        

    private:

        ros::NodeHandle _nh;
        
        //---Dyn param
        Eigen::Matrix3d _Ib;
        double _mass;
        double _gravity;
        //---

        //---Input pose
        Eigen::Vector3d _eta;
        Eigen::Vector3d _eta_dot;
        Eigen::Vector3d _vel;
        //---

        //---Gains
        Eigen::Matrix<double,6,6> _K1;
        Eigen::Matrix<double,6,6> _K2;
        //---

        Eigen::VectorXd _Fe;
        Eigen::VectorXd _Fe_integral;
        Eigen::VectorXd _Fe_integral_out;

        double _zita; // = 1.0;
        double _omega_lin; // = 5;
        double _omega_tor; // = 5;
        double _omega_z;

};



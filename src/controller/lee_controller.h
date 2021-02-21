#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "boost/thread.hpp"
#include <Eigen/Eigen>



class LEE_CONTROLLER {

    public:
        LEE_CONTROLLER();
        void controller(
                        Eigen::Vector3d mes_p, 
                        Eigen::Vector3d des_p,  
                        Eigen::Quaterniond mes_q,
                        Eigen::Vector3d mes_dp, 
                        Eigen::Vector3d des_dp,    
                        Eigen::Vector3d des_ddp,
                        double des_yaw,
                        Eigen::Vector3d mes_w,
                        Eigen::VectorXd* rotor_velocities,
                        Eigen::Vector4d* ft,
                        Eigen::Vector3d* perror,
                        Eigen::Vector3d* verror );

        void set_allocation_matrix( Eigen::MatrixXd allocation_M );
        void set_controller_gains(Eigen::Vector3d kp, Eigen::Vector3d kd, Eigen::Vector3d attitude_gain, Eigen::Vector3d angular_rate_gain );
        void set_uav_dynamics (int _motor_num, double mass, double gravity, Eigen::Matrix4d I);


    private:

        Eigen::Vector3d _kp;
        Eigen::Vector3d _kd;
        Eigen::Vector3d _attitude_gain;
        Eigen::Vector3d _angular_rate_gain;
        Eigen::MatrixXd _wd2rpm;
        Eigen::Matrix4d _I;
        double _mass;
        double _gravity;
        Eigen::Vector3d normalized_attitude_gain;
        Eigen::Vector3d normalized_angular_rate_gain;
        int _motor_num;
};
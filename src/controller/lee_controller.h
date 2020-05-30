#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "boost/thread.hpp"
#include <Eigen/Eigen>
#include <mav_msgs/Actuators.h>



class LEE_CONTROLLER {

    public:
        LEE_CONTROLLER();
        void controller(int _motor_num,
                        Eigen::Vector3d mes_p, 
                        Eigen::Vector3d des_p,  
                        Eigen::Quaterniond mes_q,
                        Eigen::Vector3d mes_dp, 
                        Eigen::Vector3d des_dp,    
                        Eigen::Vector3d des_ddp,
                        double des_yaw,
                        Eigen::Vector3d mes_w,
                        Eigen::Vector3d position_gain,
                        Eigen::Vector3d velocity_gain,
                        Eigen::Vector3d normalized_attitude_gain,
                        Eigen::Vector3d normalized_angular_rate_gain,
                        Eigen::MatrixXd wd2rpm,
                        double mass,
                        double gravity,                                    
                        Eigen::VectorXd* rotor_velocities );


    private:


};
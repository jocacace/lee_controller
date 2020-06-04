#include "ros/ros.h"
#include "controller/lee_controller.h"
#include "nav_msgs/Odometry.h"
#include "boost/thread.hpp"
#include <Eigen/Eigen>
#include <mav_msgs/Actuators.h>
#include "gazebo_msgs/ModelStates.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

using namespace std;

class CONTROLLER {

    public:
        CONTROLLER();
        void run();
        void OdometryCallback(const nav_msgs::Odometry odometry_msg);
        void ctrl_loop();
        bool get_allocation_matrix(Eigen::MatrixXd & allocation_M, int motor_size );
        void ModelStateCb( gazebo_msgs::ModelStates ms );

        void local_pose_cb( geometry_msgs::PoseStamped msg);
        void local_vel_cb( geometry_msgs::TwistStamped msg);

    private:
        ros::NodeHandle _nh;
        ros::Subscriber _odom_sub;
        ros::Subscriber _model_state_sub;
        ros::Subscriber _local_pose_cb;
        ros::Subscriber _local_vel_cb;
        
        ros::Publisher _motor_velocity_reference_pub;
        ros::Publisher _cmd_vel_motor_pub;
        nav_msgs::Odometry _odom;
        bool _first_pos;
        bool _first_vel;



        Eigen::Vector3d mes_p;        
        Eigen::Quaterniond mes_q;
        Eigen::Vector3d mes_dp;    
        Eigen::Vector3d mes_w;


};





CONTROLLER::CONTROLLER(): _first_pos(false), _first_vel(false) {

    //_odom_sub = _nh.subscribe("/iris/odometry_sensor1/odometry", 0, &CONTROLLER::OdometryCallback, this);
    _model_state_sub = _nh.subscribe("/gazebo/model_states", 0, &CONTROLLER::ModelStateCb, this);
    _motor_velocity_reference_pub = _nh.advertise<mav_msgs::Actuators>("/iris/command/motor_speed", 1);
    _cmd_vel_motor_pub = _nh.advertise<std_msgs::Float32MultiArray>("/iris_smc/cmd/motor_vel", 0);


    _local_pose_cb = _nh.subscribe("/iris_smc/local_pose", 0, &CONTROLLER::local_pose_cb, this);
    _local_vel_cb = _nh.subscribe("/iris_smc/local_vel", 0, &CONTROLLER::local_vel_cb, this);

}



void CONTROLLER::local_pose_cb( geometry_msgs::PoseStamped msg) {
     mes_p << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;        
     mes_q = Eigen::Quaterniond( msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z );

    _first_pos = true;
}

void CONTROLLER::local_vel_cb( geometry_msgs::TwistStamped msg) {
    mes_dp << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z;    
    mes_w << msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z;
    
    _first_vel = true;
}



void CONTROLLER::ModelStateCb( gazebo_msgs::ModelStates ms ) {
    

    bool found = false;
    int index = 0;

    nav_msgs::Odometry gazebo_odom;
    while(!found && index < ms.name.size()) {
        if( ms.name[index] == "iris_smc" ) {
            found = true;
        }
        else index++;
    }

    if( found ) {

        gazebo_odom.pose.pose.position.x = ms.pose[index].position.x;
        gazebo_odom.pose.pose.position.y = ms.pose[index].position.y;
        gazebo_odom.pose.pose.position.z = ms.pose[index].position.z;

        gazebo_odom.pose.pose.orientation.w = ms.pose[index].orientation.w;
        gazebo_odom.pose.pose.orientation.x = ms.pose[index].orientation.x;
        gazebo_odom.pose.pose.orientation.y = ms.pose[index].orientation.y;
        gazebo_odom.pose.pose.orientation.z = ms.pose[index].orientation.z;


        gazebo_odom.twist.twist.linear.x = ms.twist[index].linear.x;
        gazebo_odom.twist.twist.linear.y = ms.twist[index].linear.y;
        gazebo_odom.twist.twist.linear.z = ms.twist[index].linear.z;

        gazebo_odom.twist.twist.angular.x = ms.twist[index].angular.x;
        gazebo_odom.twist.twist.angular.y = ms.twist[index].angular.y;
        gazebo_odom.twist.twist.angular.z = ms.twist[index].angular.z;


        //_first_odom = true;
        _odom = gazebo_odom;
    }

}

bool CONTROLLER::get_allocation_matrix(Eigen::MatrixXd & allocation_M, int motor_size ) {

    allocation_M.resize(4, motor_size );

    int i=0;
    double rotor_angle = -0.533708;
    double arm_length = 0.255;
    double force_k = 8.54858e-06;
    double moment_k = 1.6e-2;
    double direction = 1.0;

    allocation_M(0, i) =  sin( rotor_angle ) * arm_length * force_k;
    allocation_M(1, i) = cos( rotor_angle ) * arm_length * force_k;
    allocation_M(2, i) = direction * force_k * moment_k;
    allocation_M(3, i) = -force_k;

    i++;
    rotor_angle = 2.565;
    arm_length = 0.238;
    direction = 1.0;
    allocation_M(0, i) =  sin( rotor_angle ) * arm_length * force_k;
    allocation_M(1, i) = cos( rotor_angle ) * arm_length * force_k;
    allocation_M(2, i) = direction * force_k * moment_k;
    allocation_M(3, i) = -force_k;


    i++;
    rotor_angle = 0.533708;
    arm_length = 0.255;
    direction = -1.0;
    allocation_M(0, i) =  sin( rotor_angle ) * arm_length * force_k;
    allocation_M(1, i) = cos( rotor_angle ) * arm_length * force_k;
    allocation_M(2, i) = direction * force_k * moment_k;
    allocation_M(3, i) = -force_k;


    i++; 
    rotor_angle = -2.565;
    arm_length = 0.238;
    direction = -1.0;
    allocation_M(0, i) =  sin( rotor_angle ) * arm_length * force_k;
    allocation_M(1, i) = cos( rotor_angle ) * arm_length * force_k;
    allocation_M(2, i) = direction * force_k * moment_k;
    allocation_M(3, i) = -force_k;
    /*
    for(int i=0; i<_motor_size; i++) {


        double rotor_angle;
        if( !_nh.getParam("rotor_configuration/" + std::to_string(i) + "/angle", rotor_angle) ) {
            ROS_ERROR("Problem getting angle parameter");
            return false;
        }

        double arm_length;
        if( !_nh.getParam("rotor_configuration/" + std::to_string(i) + "/arm_length", arm_length) ) {
            ROS_ERROR("Problem getting arm_length parameter");
            return false;
        }


        double force_k;
        if( !_nh.getParam("rotor_configuration/" + std::to_string(i) + "/force_k", force_k) ) {        
            ROS_ERROR("Problem getting force_k parameter");
            return false;
        }

        double moment_k;
        if( !_nh.getParam("rotor_configuration/" + std::to_string(i) + "/moment_k", moment_k) ) {
            ROS_ERROR("Problem getting moment_k parameter");    
            return false;
        }
    
        double direction;
        if( !_nh.getParam("rotor_configuration/" + std::to_string(i) + "/direction", direction) ) {
            ROS_ERROR("Problem getting direction parameter");   
            return false; 
        }
    
        allocation_M(0, i) =  sin( rotor_angle ) * arm_length * force_k;
        allocation_M(1, i) = -cos( rotor_angle ) * arm_length * force_k;
        allocation_M(2, i) = -direction * force_k * moment_k;
        allocation_M(3, i) = force_k;
    }
    */
    
    Eigen::FullPivLU<Eigen::Matrix4Xd> lu( allocation_M);
    if ( lu.rank() < 4 ) {
        ROS_ERROR("The allocation matrix rank is lower than 4. This matrix specifies a not fully controllable system, check your configuration");
        return false;
    }



    return true;
}


void CONTROLLER::ctrl_loop() {


  ros::Rate r(100);

  int _motor_num = 4;
  Eigen::Vector3d des_p;              
  Eigen::Vector3d des_dp; 
  Eigen::Vector3d des_ddp; 

  des_p << 0.0, 0.0, -1.0;
  des_dp << 0.0, 0.0, 0.0;
  des_ddp << 0.0, 0.0, 0.0;
  

  //Eigen::Vector3d mes_p;
  //Eigen::Quaterniond mes_q;
  //Eigen::Vector3d mes_dp;    
  //Eigen::Vector3d mes_w;


  Eigen::Matrix4Xd allocation_matrix;
  Eigen::Vector3d position_gain;
  Eigen::Vector3d velocity_gain;
  Eigen::Vector3d attitude_gain;
  Eigen::Vector3d angular_rate_gain;

  Eigen::Matrix3d inertia;
  Eigen::MatrixXd allocation_M;
  Eigen::MatrixXd wd2rpm;


  //inertia = Eigen::Matrix3d( Eigen::Vector3d( 0.0347563, 0.0458929, 0.0977 ).asDiagonal() );
    inertia = Eigen::Matrix3d( Eigen::Vector3d( 0.0347563, 0.0458929, 0.0977 ).asDiagonal() );

  attitude_gain << 2, 3, 0.15;
  angular_rate_gain << 0.4, 0.52, 0.18;



  Eigen::Vector3d normalized_attitude_gain;
  Eigen::Vector3d normalized_angular_rate_gain;
                                
  normalized_attitude_gain = attitude_gain.transpose() * inertia.inverse();
  normalized_angular_rate_gain = angular_rate_gain.transpose() * inertia.inverse();
   // normalized_attitude_gain = attitude_gain;
   // normalized_angular_rate_gain = angular_rate_gain;

                                
  position_gain << 6, 6, 6;
  velocity_gain << 4.7, 4.7, 4.7;
  attitude_gain << 2, 3, 0.15;
  angular_rate_gain << 0.4, 0.52, 0.18;
  
  double mass = 1.57;
  double gravity = 9.81;
  double des_yaw = 0.0;
  //while( !_first_odom ) usleep(0.1*1e6);
  while( !_first_vel || !_first_pos ) usleep(0.1*1e6);



  if( !get_allocation_matrix( allocation_M, _motor_num ) ) exit(0);
  else cout << "Matrix: " << allocation_M << endl;
  //exit(0);

  wd2rpm.resize( _motor_num, 4 );
  //Malloc dagger
  Eigen::Matrix4d I;
  I.setZero();
  I.block<3, 3>(0, 0) = inertia;
  I(3, 3) = 1;


    cout << "allocation_M: " << allocation_M << endl;
  wd2rpm = allocation_M.transpose() * (allocation_M*allocation_M.transpose()).inverse()*I;    
  LEE_CONTROLLER lc;

  std_msgs::Float32MultiArray motor_vel;
  motor_vel.data.resize(4);

  while( ros::ok() ) {




    //Eigen::Matrix3d R = mes_q.toRotationMatrix();
    //cout << "R: " << endl << R << endl;
    //Eigen::Vector3d ea = R.eulerAngles(0,1,2); 
    //cout << "to Euler angles:" << endl;
    //cout << ea << endl << endl;


    Eigen::VectorXd ref_rotor_velocities;
    //lee_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);


    //mes_p << _odom.pose.pose.position.x, _odom.pose.pose.position.y, _odom.pose.pose.position.z;
    //mes_dp << _odom.twist.twist.linear.x, _odom.twist.twist.linear.y, _odom.twist.twist.linear.z; 
    //mes_q = Eigen::Quaterniond( _odom.pose.pose.orientation.w, _odom.pose.pose.orientation.x, _odom.pose.pose.orientation.y, _odom.pose.pose.orientation.z );
    //mes_w << _odom.twist.twist.angular.x, _odom.twist.twist.angular.y, _odom.twist.twist.angular.z;  
    
    lc.controller( _motor_num, mes_p, des_p, mes_q, mes_dp, des_dp, des_ddp, des_yaw, mes_w,
                                            position_gain, velocity_gain, normalized_attitude_gain, normalized_angular_rate_gain, wd2rpm, mass, gravity, &ref_rotor_velocities);
    



    mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
    actuator_msg->angular_velocities.clear();
    for (int i = 0; i < ref_rotor_velocities.size(); i++)
      actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
    

    //cout << "ref_rotor_velocities: " << ref_rotor_velocities << endl;

    _motor_velocity_reference_pub.publish(actuator_msg);
    motor_vel.data[0] = ref_rotor_velocities[0];
    motor_vel.data[1] = ref_rotor_velocities[1];
    motor_vel.data[2] = ref_rotor_velocities[2];
    motor_vel.data[3] = ref_rotor_velocities[3];
    _cmd_vel_motor_pub.publish( motor_vel );

    r.sleep();
  }


}

void CONTROLLER::OdometryCallback(const nav_msgs::Odometry odometry_msg) {
    _odom = odometry_msg;
    //_first_odom = true;

}


void CONTROLLER::run() {
    boost::thread ctrl_loop_t( &CONTROLLER::ctrl_loop, this );
    ros::spin();
}




int main( int argc, char** argv ) {

    ros::init(argc, argv, "lee_controller");
    CONTROLLER c;
    c.run();

    return 0;
}
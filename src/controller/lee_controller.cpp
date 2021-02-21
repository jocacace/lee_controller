#include "lee_controller.h"


using namespace std;


LEE_CONTROLLER::LEE_CONTROLLER() {}


void LEE_CONTROLLER::set_allocation_matrix(  Eigen::MatrixXd allocation_M ) {
  _wd2rpm = allocation_M.transpose() * (allocation_M*allocation_M.transpose()).inverse()*_I;    
}

void LEE_CONTROLLER::set_uav_dynamics (int motor_num, double mass, double gravity, Eigen::Matrix4d I) {
  _mass = mass;
  _gravity = gravity;
  _I = I;
  _motor_num = motor_num;
}

void LEE_CONTROLLER::set_controller_gains(Eigen::Vector3d kp, Eigen::Vector3d kd, Eigen::Vector3d attitude_gain, Eigen::Vector3d angular_rate_gain ) {
  _kp = kp;
  _kd = kd;
  _attitude_gain = attitude_gain;
  _angular_rate_gain = angular_rate_gain;
}



void LEE_CONTROLLER::controller(    Eigen::Vector3d mes_p, 
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
                                    Eigen::Vector3d* verror ) {


    Eigen::Vector3d normalized_attitude_gain;
    Eigen::Vector3d normalized_angular_rate_gain;
    normalized_attitude_gain = _attitude_gain.transpose() * _I.block(0,0,3,3).inverse();
    normalized_angular_rate_gain = _angular_rate_gain.transpose() * _I.block(0,0,3,3).inverse();


    rotor_velocities->resize(_motor_num);
    Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());
  
    Eigen::Vector3d acceleration;
    Eigen::Vector3d position_error;
    position_error = mes_p - des_p;

    Eigen::Vector3d velocity_error;
    velocity_error = mes_dp - des_dp;

    acceleration = -(position_error.cwiseProduct(_kp)
      + velocity_error.cwiseProduct(_kd)) / _mass
      - _gravity * e_3 + des_ddp;
  
    Eigen::Vector3d angular_acceleration;
    Eigen::Matrix3d R = mes_q.toRotationMatrix();
    Eigen::Vector3d b1_des;
    double yaw = des_yaw;
    b1_des << cos(yaw), sin(yaw), 0;

    Eigen::Vector3d b3_des;
    b3_des = -acceleration / acceleration.norm();

    Eigen::Vector3d b2_des;
    b2_des = b3_des.cross(b1_des);
    b2_des.normalize();

    Eigen::Matrix3d R_des;
    R_des.col(0) = b2_des.cross(b3_des);
    R_des.col(1) = b2_des;
    R_des.col(2) = b3_des;

    // Angle error according to lee et al.
    Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
    Eigen::Vector3d angle_error;
    angle_error << angle_error_matrix(2, 1), angle_error_matrix(0,2), angle_error_matrix(1, 0);
    

    Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
    angular_rate_des[2] = 0.1*des_yaw;
 

    const Eigen::Matrix3d R_W_I = mes_q.toRotationMatrix();
    Eigen::Vector3d angular_rate_error = mes_w - R_des.transpose() * R * angular_rate_des;

    angular_acceleration = -1 * angle_error.cwiseProduct(normalized_attitude_gain)
                            - angular_rate_error.cwiseProduct(normalized_angular_rate_gain)
                            + mes_w.cross(mes_w); // we don't need the inertia matrix here



    double thrust = _mass * acceleration.dot( mes_q.toRotationMatrix().col(2));

    Eigen::Vector4d angular_acceleration_thrust;
    angular_acceleration_thrust.block<3, 1>(0, 0) = angular_acceleration;
    angular_acceleration_thrust(3) = thrust;

    *rotor_velocities = _wd2rpm * angular_acceleration_thrust;
    *ft = angular_acceleration_thrust;
    *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
    *rotor_velocities = rotor_velocities->cwiseSqrt();

    *perror = position_error;
    *verror = velocity_error;
  

    
}

#include "lee_controller.h"


using namespace std;

LEE_CONTROLLER::LEE_CONTROLLER() {

}

void LEE_CONTROLLER::controller(    int _motor_num,
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
                                    Eigen::VectorXd* rotor_velocities) {


    rotor_velocities->resize(_motor_num);
  
    Eigen::Vector3d acceleration;
    //ComputeDesiredAcceleration(&acceleration);

    Eigen::Vector3d position_error;
    //Body frame position error
    //position_error = mes_p - des_p;
    position_error = des_p - mes_p;
   
    position_error(0) = position_error(1) = 0.0;

    cout << "Mes: " << mes_p << endl;
    cout << "des: " << des_p << endl;
    cout << "pe: " << position_error << endl;

    // Transform velocity to world frame.
    const Eigen::Matrix3d R_W_I = mes_q.toRotationMatrix();
    Eigen::Vector3d velocity_W =  R_W_I * mes_dp;
    Eigen::Vector3d velocity_error;
    velocity_error = des_dp - velocity_W;
    Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());
    e_3 = -e_3;

    cout << "e_3_: " << e_3 << endl;
    acceleration = (position_error.cwiseProduct(position_gain)
      + velocity_error.cwiseProduct(velocity_gain)) / mass
      - gravity * e_3 - des_ddp;

    cout << "acceleration: " << acceleration.transpose() << endl;


    Eigen::Vector3d angular_acceleration;
    //ComputeDesiredAngularAcc(acceleration, &angular_acceleration);

    Eigen::Matrix3d R = mes_q.toRotationMatrix();

    // Get the desired rotation matrix.
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
    angular_rate_des[2] = des_yaw;

    Eigen::Vector3d angular_rate_error = mes_w - R_des.transpose() * R * angular_rate_des;

    angular_acceleration = -1 * angle_error.cwiseProduct(normalized_attitude_gain)
                            - angular_rate_error.cwiseProduct(normalized_angular_rate_gain)
                            + mes_w.cross(mes_w); // we don't need the inertia matrix here



    // Project thrust onto body z axis.
    double thrust = -mass * acceleration.dot( mes_q.toRotationMatrix().col(2));

    Eigen::Vector4d angular_acceleration_thrust;
    angular_acceleration_thrust.block<3, 1>(0, 0) = angular_acceleration;
    angular_acceleration_thrust(3) = thrust;

    //std::cout << "2rpm: " << wd2rpm << std::endl;
    //std::cout << "angular_acc_to_rotor_velocities_: " << angular_acc_to_rotor_velocities_ << std::endl;
    *rotor_velocities = wd2rpm * angular_acceleration_thrust;
    *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
    *rotor_velocities = rotor_velocities->cwiseSqrt();
  }

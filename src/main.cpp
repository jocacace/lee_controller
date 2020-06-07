#include "ros/ros.h"
#include "controller/lee_controller.h"
#include "nav_msgs/Odometry.h"
#include "boost/thread.hpp"
#include <Eigen/Eigen>
#include "gazebo_msgs/ModelStates.h"
#include "std_msgs/Float32MultiArray.h"
#include "motion_planner/generate_plan.h"
#include "tf/tf.h"

using namespace std;
using namespace Eigen;





inline Matrix3d QuatToMat(Vector4d Quat){
    Matrix3d Rot;
    float s = Quat[0];
    float x = Quat[1];
    float y = Quat[2];
    float z = Quat[3];
    Rot << 1-2*(y*y+z*z),2*(x*y-s*z),2*(x*z+s*y),
    2*(x*y+s*z),1-2*(x*x+z*z),2*(y*z-s*x),
    2*(x*z-s*y),2*(y*z+s*x),1-2*(x*x+y*y);
    return Rot;
}


	inline Vector4d rot2quat(Matrix3d R){

		float m00, m01, m02, m10, m11, m12, m20, m21, m22;

		m00 = R(0,0);
		m01 = R(0,1);
		m02 = R(0,2);
		m10 = R(1,0);
		m11 = R(1,1);
		m12 = R(1,2);
		m20 = R(2,0);
		m21 = R(2,1);
		m22 = R(2,2);

		float tr = m00 + m11 + m22;
		float qw, qx, qy, qz, S;
		Vector4d quat;

		if (tr > 0) { 
		  S = sqrt(tr+1.0) * 2; // S=4*qw 
		  qw = 0.25 * S;
		  qx = (m21 - m12) / S;
		  qy = (m02 - m20) / S; 
		  qz = (m10 - m01) / S; 
		} else if ((m00 > m11)&(m00 > m22)) { 
		  S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx 
		  qw = (m21 - m12) / S;
		  qx = 0.25 * S;
		  qy = (m01 + m10) / S; 
		  qz = (m02 + m20) / S; 
		} else if (m11 > m22) { 
		  S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
		  qw = (m02 - m20) / S;
		  qx = (m01 + m10) / S; 
		  qy = 0.25 * S;
		  qz = (m12 + m21) / S; 
		} else { 
		  S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
		  qw = (m10 - m01) / S;
		  qx = (m02 + m20) / S;
		  qy = (m12 + m21) / S;
		  qz = 0.25 * S;
		}

		quat << qw, qx, qy, qz;
		return quat;

	}
	
	

inline Vector3d R2XYZ(Matrix3d R) {
    double phi=0.0, theta=0.0, psi=0.0;
    Vector3d XYZ = Vector3d::Zero();
    
    theta = asin(R(0,2));
    
    if(fabsf(cos(theta))>pow(10.0,-10.0))
    {
        phi=atan2(-R(1,2)/cos(theta), R(2,2)/cos(theta));
        psi=atan2(-R(0,1)/cos(theta), R(0,0)/cos(theta));
    }
    else
    {
        if(fabsf(theta-M_PI/2.0)<pow(10.0,-5.0))
        {
            psi = 0.0;
            phi = atan2(R(1,0), R(2,0));
            theta = M_PI/2.0;
        }
        else
        {
            psi = 0.0;
            phi = atan2(-R(1,0), R(2,0));
            theta = -M_PI/2.0;
        }
    }
    
    XYZ << phi,theta,psi;
    return XYZ;
}

 

	inline Matrix3d XYZ2R(Vector3d angles) {
	  	
	  	Matrix3d R = Matrix3d::Zero(); 
	  	Matrix3d R1 = Matrix3d::Zero(); 
	  	Matrix3d R2 = Matrix3d::Zero(); 
	  	Matrix3d R3 = Matrix3d::Zero();

		float cos_phi = cos(angles[0]);
		float sin_phi = sin(angles[0]);
		float cos_theta = cos(angles[1]);
		float sin_theta = sin(angles[1]);
		float cos_psi = cos(angles[2]);
		float sin_psi = sin(angles[2]);

		R1  << 1, 0      , 0, 
			        0, cos_phi, -sin_phi, 
			        0, sin_phi, cos_phi;

		R2  << cos_theta , 0, sin_theta, 
			        0        , 1, 0       , 
			        -sin_theta, 0, cos_theta;

		R3  << cos_psi, -sin_psi, 0, 
			        sin_psi, cos_psi , 0,
			        0      , 0       , 1;

		R = R1*R2*R3;

		return R;
	}


class CONTROLLER {

    public:
        CONTROLLER();
        void run();
        void OdometryCallback(const nav_msgs::Odometry odometry_msg);
        void ctrl_loop();
        bool get_allocation_matrix(Eigen::MatrixXd & allocation_M, int motor_size );
        void request_new_plan();

    private:
        ros::NodeHandle _nh;
        ros::Subscriber _odom_sub;
        ros::Subscriber _model_state_sub;
        ros::Publisher _motor_velocity_reference_pub;
        ros::Publisher _cmd_vel_motor_pub;
        nav_msgs::Odometry _odom;
        bool _first_odom;
        motion_planner::generate_plan _plan;
        bool _new_plan;


};





CONTROLLER::CONTROLLER(): _first_odom(false), _new_plan(false) {
    _odom_sub = _nh.subscribe("/iris_smc/odometry", 0, &CONTROLLER::OdometryCallback, this);
    _cmd_vel_motor_pub = _nh.advertise<std_msgs::Float32MultiArray>("/iris_smc/cmd/motor_vel", 0);
}

void CONTROLLER::request_new_plan() {


    float x, y, z, yaw;
    ros::ServiceClient client = _nh.serviceClient<motion_planner::generate_plan>("generate_tarjectory");

    while(ros::ok()) {
        cout << "Insert new coordinates x (front), y (left), z (upword), yaw (c-clowise)" <<endl;
        scanf("%f %f %f %f", &x, &y, &z, &yaw);

        cout << "Request new plan for: [" << x << ", " << y << ", " << z << " - " << yaw << "]" << endl;

        _plan.request.p_i.position.x = _odom.pose.pose.position.x;
        _plan.request.p_i.position.y = _odom.pose.pose.position.y;
        _plan.request.p_i.position.z = _odom.pose.pose.position.z;
        Vector3d eu = R2XYZ( QuatToMat ( Vector4d( _odom.pose.pose.orientation.w,  _odom.pose.pose.orientation.x, _odom.pose.pose.orientation.y, _odom.pose.pose.orientation.z ) ) );
        
        tf::Quaternion q;
        q.setRPY(0, 0, eu(2));
        q = q.normalize();

        _plan.request.p_i.orientation.w = q.w();
        _plan.request.p_i.orientation.x = q.x();
        _plan.request.p_i.orientation.y = q.y();
        _plan.request.p_i.orientation.z = q.z();


        _plan.request.p_f.position.x = x;
        _plan.request.p_f.position.y = y;
        _plan.request.p_f.position.z = z;


        Vector4d dq = rot2quat ( XYZ2R( Vector3d( 0, 0, yaw) ) );


        _plan.request.p_f.orientation.w = dq[0];
        _plan.request.p_f.orientation.x = dq[1];
        _plan.request.p_f.orientation.y = dq[2];
        _plan.request.p_f.orientation.z = dq[3];


        if( client.call( _plan ) ) {
          _new_plan = true;
        }        
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
    allocation_M(1, i) = -cos( rotor_angle ) * arm_length * force_k;
    allocation_M(2, i) = -direction * force_k * moment_k;
    allocation_M(3, i) = force_k;

    i++;
    rotor_angle = 2.565;
    arm_length = 0.238;
    direction = 1.0;
    allocation_M(0, i) =  sin( rotor_angle ) * arm_length * force_k;
    allocation_M(1, i) = -cos( rotor_angle ) * arm_length * force_k;
    allocation_M(2, i) = -direction * force_k * moment_k;
    allocation_M(3, i) = force_k;


    i++;
    rotor_angle = 0.533708;
    arm_length = 0.255;
    direction = -1.0;
    allocation_M(0, i) =  sin( rotor_angle ) * arm_length * force_k;
    allocation_M(1, i) = -cos( rotor_angle ) * arm_length * force_k;
    allocation_M(2, i) = -direction * force_k * moment_k;
    allocation_M(3, i) = force_k;


    i++; 
    rotor_angle = -2.565;
    arm_length = 0.238;
    direction = -1.0;
    allocation_M(0, i) =  sin( rotor_angle ) * arm_length * force_k;
    allocation_M(1, i) = -cos( rotor_angle ) * arm_length * force_k;
    allocation_M(2, i) = -direction * force_k * moment_k;
    allocation_M(3, i) = force_k;
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

  des_p << 0.0, 0.0, 0.0;
  des_dp << 0.0, 0.0, 0.0;
  des_ddp << 0.0, 0.0, 0.0;
  

  Eigen::Vector3d mes_p;
  Eigen::Quaterniond mes_q;
  Eigen::Vector3d mes_dp;    
  Eigen::Vector3d mes_w;


  Eigen::Matrix4Xd allocation_matrix;
  Eigen::Vector3d position_gain;
  Eigen::Vector3d velocity_gain;
  Eigen::Vector3d attitude_gain;
  Eigen::Vector3d angular_rate_gain;

  Eigen::Matrix3d inertia;
  Eigen::MatrixXd allocation_M;
  Eigen::MatrixXd wd2rpm;


  inertia = Eigen::Matrix3d( Eigen::Vector3d( 0.0347563, 0.0458929, 0.0977 ).asDiagonal() );

  attitude_gain << 2, 3, 0.15;
  angular_rate_gain << 0.4, 0.52, 0.18;



  Eigen::Vector3d normalized_attitude_gain;
  Eigen::Vector3d normalized_angular_rate_gain;
                                
  normalized_attitude_gain = attitude_gain.transpose() * inertia.inverse();
  normalized_angular_rate_gain = angular_rate_gain.transpose() * inertia.inverse();

                                
  position_gain << 6, 6, 6;
  velocity_gain << 4.7, 4.7, 4.7;
  attitude_gain << 2, 3, 0.15;
  angular_rate_gain << 0.4, 0.52, 0.18;
  
  double mass = 1.57;
  double gravity = 9.81;


  while( !_first_odom ) usleep(0.1*1e6);
  boost::thread input_t( &CONTROLLER::request_new_plan, this);




  if( !get_allocation_matrix( allocation_M, _motor_num ) ) exit(0);
  else cout << "Matrix: " << allocation_M << endl;
  //exit(0);

  Vector3d eu = R2XYZ( QuatToMat ( Vector4d( _odom.pose.pose.orientation.w,  _odom.pose.pose.orientation.x, _odom.pose.pose.orientation.y, _odom.pose.pose.orientation.z ) ) );
  double des_yaw = eu(2);

  wd2rpm.resize( _motor_num, 4 );
  //Malloc dagger
  Eigen::Matrix4d I;
  I.setZero();
  I.block<3, 3>(0, 0) = inertia;
  I(3, 3) = 1;

  wd2rpm = allocation_M.transpose() * (allocation_M*allocation_M.transpose()).inverse()*I;    
  cout << "wd2rpm: " << endl << wd2rpm << endl;

  LEE_CONTROLLER lc;

  std_msgs::Float32MultiArray motor_vel;
  motor_vel.data.resize(4);


  int plan_index = 0;
   
    while( ros::ok() ) {

      

    Eigen::VectorXd ref_rotor_velocities;
    //lee_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

    /*
    for(int i=0; i<plan.response.traj.points[0].positions.size(); i++ ) {

        des_p << plan.response.traj.points[0].positions[i],  plan.response.traj.points[1].positions[i], plan.response.traj.points[2].positions[i];
        des_yaw = plan.response.traj.points[3].positions[i];
    */

    if( _new_plan ) {
        _new_plan = false;
        plan_index = 0;
    }

    if( _plan.response.traj.points.size() > 0.0 ) {
        if( plan_index < _plan.response.traj.points[0].positions.size()-1 ) {
            plan_index++;
            des_p << _plan.response.traj.points[0].positions[plan_index], _plan.response.traj.points[1].positions[plan_index], _plan.response.traj.points[2].positions[plan_index];
            des_dp << _plan.response.traj.points[0].velocities[plan_index], _plan.response.traj.points[1].velocities[plan_index], _plan.response.traj.points[2].velocities[plan_index];
            des_ddp << _plan.response.traj.points[0].accelerations[plan_index], _plan.response.traj.points[1].accelerations[plan_index], _plan.response.traj.points[2].accelerations[plan_index];
         
            des_yaw = _plan.response.traj.points[3].positions[plan_index];
        }

    }



    mes_p << _odom.pose.pose.position.x, _odom.pose.pose.position.y, _odom.pose.pose.position.z;
    mes_dp << _odom.twist.twist.linear.x, _odom.twist.twist.linear.y, _odom.twist.twist.linear.z; 
    mes_q = Eigen::Quaterniond( _odom.pose.pose.orientation.w, _odom.pose.pose.orientation.x, _odom.pose.pose.orientation.y, _odom.pose.pose.orientation.z );
    mes_w << _odom.twist.twist.angular.x, _odom.twist.twist.angular.y, _odom.twist.twist.angular.z;  

    Eigen::Matrix3d R = mes_q.toRotationMatrix();
    mes_w = R.transpose()*mes_w;

    lc.controller( _motor_num, mes_p, des_p, mes_q, mes_dp, des_dp, des_ddp, des_yaw, mes_w,
                                            position_gain, velocity_gain, normalized_attitude_gain, normalized_angular_rate_gain, wd2rpm, mass, gravity, &ref_rotor_velocities);
    

    

    motor_vel.data[0] = ref_rotor_velocities[0];
    motor_vel.data[1] = ref_rotor_velocities[1];
    motor_vel.data[2] = ref_rotor_velocities[2];
    motor_vel.data[3] = ref_rotor_velocities[3];
    _cmd_vel_motor_pub.publish( motor_vel );

    r.sleep();
  // }

  }


}

void CONTROLLER::OdometryCallback(const nav_msgs::Odometry odometry_msg) {
    _odom = odometry_msg;
    _first_odom = true;

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
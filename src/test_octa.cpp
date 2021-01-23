#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "boost/thread.hpp"
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Dense>
#include "mav_msgs/Actuators.h"
#include <tf/tf.h>
#include "utils.h"
#include "std_msgs/Bool.h"
#include "gazebo_msgs/ModelState.h"
#include "geometry_msgs/Pose.h"
#include "std_srvs/Empty.h"
#include <iostream>
#include <fstream>
#include "std_msgs/Float64.h"
#include "geometry_msgs/Wrench.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Vector3.h"


using namespace std;
using namespace Eigen;

#define Z_INIT -0.2


Vector3d Vee(Matrix3d S) {
  Vector3d v;
  v << S(2,1),S(0,2),S(1,0);
  return v;
}

Matrix3d Skew(Vector3d v) {
  Matrix3d S;
  S << 0, -v(2), v(1),
       v(2), 0, -v(0),
       -v(1), v(0), 0;

  return S;
}

class QUAD_CTRL {

    public:
        QUAD_CTRL();    
        void run();
        void ctrl_loop();
        void odom_cb( nav_msgs::OdometryConstPtr );
        void ffilter();
        void insert_dest(); //Temp
        void extWesti();
        void sys_reset( std_msgs::Bool );
        void system_reset();
        void cmd_publisher();
        void goal_cb( geometry_msgs::Pose );
        void wlogs();
        void cv_vel_cb( std_msgs::Float64 cv );
        void fault_cb( std_msgs::Float32MultiArray fs);

    private:

        void updateError();
        void correctW(Vector4d & w);

        ros::NodeHandle _nh;
        ros::Subscriber _odom_sub;
        ros::Subscriber _system_reset_req;
        ros::Subscriber _goal_sub;
        ros::Subscriber _cv_sub;
        ros::Subscriber _fault_pub;
        ros::Publisher _model_state_pub;
        ros::Publisher _cmd_vel_pub;
        ros::Publisher _controller_active;
        ros::Publisher _land_state_pub;
        ros::Publisher _NED_odom_pub;
        ros::Publisher _est_wrench_pub;
        ros::Publisher _e_p_pub;
        ros::Publisher _e_r_pub;
        
        int _rate;
        double _freq;
        Vector3d _P;
        Vector3d _P_dot;
        Vector3d _Eta;
        Vector3d _Eta_dot;
        Vector3d _etaDot_des, _etadd_des;
        Vector3d _wbb;
        Matrix3d _Rb, _Rb_des;
        Matrix3d _RNed;
        Vector3d _Ep;
        Vector3d _Ev;
        Vector3d _Er, _Ew;

        double _yaw_cmd;
        Vector3d _cmd_p;
        Vector3d _cmd_dp;
        Vector3d _cmd_ddp;
        Vector3d _ref_p;
        Vector3d _ref_dp;
        Vector3d _ref_ddp;
        double _ref_yaw;
        
        Matrix3d _Q, _Qdot;
        Matrix4d _G;
        double _c_T;
        double _c_a;
        double _l;
        double _m;
        Matrix3d _I_b;
        
      
        bool _odomOk;
        Vector3d _P_des, _Pd_des, _Pdd_des;
        Vector3d _Pcompl_des, _Pdcompl_des, _Pddcompl_des;
        Vector3d _wbb_des, _wbbd_des;
        Matrix3d _Kp, _Kv, _Kr, _Kw;
        double _uT;
        Vector3d _tau_b;
        bool _isSettingTraj;
        tf::Quaternion _q_des;
        mav_msgs::Actuators _comm;    
        double _ref_dyaw;
        double _ref_ddyaw;
        VectorXd _Fe, _Fe_integral, _Fe_integral_out;
        bool _sys_res;
        double _rp_th;
        ros::ServiceClient _pause_phy;
        ros::ServiceClient _unpause_phy;
        bool _restarting;

        Vector4d _omega_motor;
        bool _landed;
        double _l_h; //Landing altitude     
        double _ref_vel_max;
        Vector4d _faults;

        //System parameters
        string _motor_speed_topic;
        string _odometry_topic;
        string _model_name;
};



QUAD_CTRL::QUAD_CTRL() { 


    if( !_nh.getParam("motor_speed_topic", _motor_speed_topic)) {
      _motor_speed_topic = "/ndt2/gazebo/command/motor_speed";
    }
    if( !_nh.getParam("odometry_topic", _odometry_topic)) {
      _odometry_topic = "/hummingbird/ground_truth/odometry";
    }
    if( !_nh.getParam("model_name", _model_name)) {
      _model_name = "/ndt2/gazebo/command/motor_speed";
    }

    _odom_sub = _nh.subscribe("/hummingbird/ground_truth/odometry", 1, &QUAD_CTRL::odom_cb, this);
    _system_reset_req = _nh.subscribe("/lee/sys_reset", 1, &QUAD_CTRL::sys_reset, this);
    _cmd_vel_pub = _nh.advertise< mav_msgs::Actuators>("/ndt2/gazebo/command/motor_speed", 1);
    _controller_active = _nh.advertise< std_msgs::Bool >("/lee/controller_active", 1);
    _model_state_pub = _nh.advertise< gazebo_msgs::ModelState >("/gazebo/set_model_state", 1);
    _NED_odom_pub = _nh.advertise< geometry_msgs::Pose > ("/lee/pose/ned", 1);
    _land_state_pub = _nh.advertise< std_msgs::Bool > ("/lee/landed", 1);
    _goal_sub = _nh.subscribe< geometry_msgs::Pose > ("/lee/goal", 1, &QUAD_CTRL::goal_cb, this );
    _cv_sub = _nh.subscribe ("/lee/cruise_velocity", 1, &QUAD_CTRL::cv_vel_cb, this );
    _est_wrench_pub = _nh.advertise< geometry_msgs::Wrench > ("/lee/ext_wrench", 1);
    _fault_pub = _nh.subscribe("/lee/faults", 1, &QUAD_CTRL::fault_cb, this);
    _e_p_pub = _nh.advertise<geometry_msgs::Vector3> ("/lee/ep", 1);
    _e_r_pub = _nh.advertise<geometry_msgs::Vector3> ("/lee/er", 1);

    ros::Publisher _e_p_pub;
    ros::Publisher _e_r_pub;
       
    _odomOk = false;

    _P.resize(3);
    _Eta.resize(3);


    if( !_nh.getParam("rate", _rate)) {
      _rate = 200;
    }
    _freq = 1.0/float(_rate);


    if( !_nh.getParam("l", _l)) {
      _l = 0.17;
    }
    if( !_nh.getParam("c_T", _c_T)) {
      _c_T = 8.54802e-06;
    }
    if( !_nh.getParam("c_a", _c_a)) {
      _c_a = -1.36780194e-7; //Motor k * Moment k
    }
    if( !_nh.getParam("m", _m)) {
      _m = 1.5;
    }


    //Inertia matrix
    _I_b << 0.777, 0, 0,
           0, 0.777, 0,
           0, 0, 1.112;

    double mRobot = 0.97;//Kg

    //_I_b+=_I_b;
    //_m += mRobot;

    _G(0,0) = _c_T;    _G(0,1) = _c_T;    _G(0,2) = _c_T; _G(0,3) = _c_T;
    _G(1,0) = 0;       _G(1,1) = _l*_c_T; _G(1,2) = 0;    _G(1,3) = -_l*_c_T;
    _G(2,0) = -_l*_c_T; _G(2,1) = 0;       _G(2,2) = _l*_c_T; _G(2,3) = 0;
    _G(3,0) = -_c_a;    _G(3,1) = _c_a;    _G(3,2) = -_c_a; _G(3,3) = _c_a;

  
    ROS_WARN("Starting GEOMETRIC controller!");

    double _ll_kp_x;
    double _ll_kp_y;
    double _ll_kp_z;

    double _ll_kv_x;
    double _ll_kv_y;
    double _ll_kv_z;

    double _ll_kr_x;
    double _ll_kr_y;
    double _ll_kr_z;

    double _ll_kw_x;
    double _ll_kw_y;
    double _ll_kw_z;


    if( !_nh.getParam("rp_th", _rp_th)) {
      _rp_th = 0.25;
    }


    if( !_nh.getParam("ll_kp_x", _ll_kp_x)) {
      _ll_kp_x = 5.0;
    }
    if( !_nh.getParam("ll_kp_y", _ll_kp_y)) {
      _ll_kp_y = 5.0;
    }
    if( !_nh.getParam("ll_kp_y", _ll_kp_z)) {
      _ll_kp_z = 5.0;
    }

    if( !_nh.getParam("ll_kv_x", _ll_kv_x)) {
      _ll_kv_x = 5.0;
    }
    if( !_nh.getParam("ll_kv_y", _ll_kv_y)) {
      _ll_kv_y = 5.0;
    }
    if( !_nh.getParam("ll_kv_z", _ll_kv_z)) {
      _ll_kv_z = 5.0;
    }

    if( !_nh.getParam("ll_kr_x", _ll_kr_x)) {
      _ll_kr_x = 5.0;
    }
    if( !_nh.getParam("ll_kr_y", _ll_kr_y)) {
      _ll_kr_y = 5.0;
    }
    if( !_nh.getParam("ll_kr_z", _ll_kr_z)) {
      _ll_kr_z = 5.0;
    }

    if( !_nh.getParam("ll_kw_x", _ll_kw_x)) {
      _ll_kw_x = 5.0;
    }
    if( !_nh.getParam("ll_kw_y", _ll_kw_y)) {
      _ll_kw_y = 5.0;
    }
    if( !_nh.getParam("ll_kw_z", _ll_kw_z)) {
      _ll_kw_z = 5.0;
    }

    _Kp = Vector3d( _ll_kp_x, _ll_kp_y, _ll_kp_z ).asDiagonal();
    _Kv = Vector3d( _ll_kv_x, _ll_kv_y, _ll_kv_z ).asDiagonal();
    _Kr = Vector3d( _ll_kr_x, _ll_kr_y, _ll_kr_z ).asDiagonal();
    _Kw = Vector3d( _ll_kw_x, _ll_kw_y, _ll_kw_z ).asDiagonal();

    _Kp = 5.0*Vector3d(10,10,80).asDiagonal();
    _Kv = _Kp/1.5;
    _Kr = 4.0*Vector3d(30,30,30).asDiagonal();
    _Kw = _Kr/4;


    _Fe.resize(6);
    _Fe = Eigen::VectorXd::Zero(6);
    _Fe_integral.resize(6);
    _Fe_integral = Eigen::VectorXd::Zero(6);
    _Fe_integral_out.resize(6);
    _Fe_integral_out = Eigen::VectorXd::Zero(6);
  

   


    _Qdot.resize(3,3);
    _uT = 0;
    _tau_b = Vector3d::Zero();
   
    _cmd_p << 0.0, 0.0, 0.0;
    _cmd_dp << 0.0, 0.0, 0.0;
    _cmd_ddp << 0.0, 0.0, 0.0;
    _ref_yaw = 0.0;
    _omega_motor << 0.0, 0.0, 0.0, 0.0;
    
    _pause_phy = _nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    _unpause_phy = _nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

    _restarting = false;
    _l_h = 0.25;
    _landed = false;
    _faults << 1.0, 1.0, 1.0, 1.0;
}

void QUAD_CTRL::fault_cb( std_msgs::Float32MultiArray fs) {
  _faults << (1.0-fs.data[0]), (1.0-fs.data[1]), (1.0-fs.data[2]), (1.0-fs.data[3]);
}


void QUAD_CTRL::extWesti() {
  
  geometry_msgs::Wrench est_w;

  double zita_ = 1.0;
  double omega_lin = 5;
  double omega_tor = 5;

  double omega_z = omega_tor;
  
  MatrixXd zita(6,6), omega(6,6);
  MatrixXd K1(6,6), K2(6,6);
  

  zita = MatrixXd::Identity(6,6);
  omega = MatrixXd::Identity(6,6);
  zita.diagonal() << zita_, zita_, zita_, zita_, zita_, zita_;
  omega.diagonal() << omega_lin, omega_lin, omega_z, omega_tor, omega_tor, omega_z;
  K1 = 2*zita*omega;
  K2 = omega*omega*K1.inverse();

  Vector3d e3(0,0,1);
 
  MatrixXd M_xi(6,6);
  M_xi << _m*MatrixXd::Identity(3,3) , MatrixXd::Zero(3,3),
  MatrixXd::Zero(3,3) , _I_b;

  VectorXd alpha(6);
  alpha.head(3)=_Rb.transpose()*_P_dot;
  alpha.tail(3)=_wbb;

  VectorXd internal(6);
  internal.head(3) = -_uT*e3 + _m*9.81*_Rb.transpose()*e3;
  internal.tail(3) = _tau_b - Skew(_wbb)*_I_b*_wbb;



  _Fe_integral += ( internal + _Fe )*_freq;
  if( !isnan(_Fe_integral.norm()) ) {
    _Fe_integral_out += ( -_Fe + K2*( M_xi*alpha - _Fe_integral ) )*_freq;
    _Fe = K1*_Fe_integral_out;
  }

  

  est_w.force.x = _Fe(0);
  est_w.force.y = _Fe(1);
  est_w.force.z = _Fe(2);
  est_w.torque.x = _Fe(3);
  est_w.torque.y = _Fe(4);
  est_w.torque.z = _Fe(5);
  _est_wrench_pub.publish(est_w);
  
}

//Data log for debug:
//  P, RefP. CmdP - yaw, refYaw, cmdYaw, ExtForce
void QUAD_CTRL::wlogs( ) {
  ros::Rate r(100);
 
  ofstream log_file;
  log_file.open ("/tmp/lee_logs.txt");
  
  while(ros::ok()) {


    if( !_landed ) {

      log_file << _P(0) << ", " << _P(1) << ", " << _P(2) << ", ";
      log_file << _ref_p(0) << ", " << _ref_p(1) << ", " << _ref_p(2) << ", ";
      log_file << _cmd_p(0) << ", " << _cmd_p(1) << ", " << _cmd_p(2) << ", ";

      log_file << _Eta(2) << ", " << _ref_yaw << ", " << _yaw_cmd << ", ";
      log_file << _Fe(0) << ", " << _Fe(1) << ", " << _Fe(2) << ", ";
      log_file << _Fe(3) << ", " << _Fe(4) << ", " << _Fe(5) << endl;
    }
    r.sleep();
  }

  log_file.close();

}
void QUAD_CTRL::sys_reset( std_msgs::Bool d) {
  _sys_res = d.data;
}
void QUAD_CTRL::ffilter(){
  
  //Params
  double ref_jerk_max;
  double ref_acc_max;
  double ref_omega;
  double ref_zita;

  double ref_o_jerk_max;
  double ref_o_acc_max;
  double ref_o_vel_max;
  

  if( !_nh.getParam("ref_jerk_max", ref_jerk_max)) {
      ref_jerk_max = 0.35;
  }
  if( !_nh.getParam("ref_acc_max", ref_acc_max)) {
      ref_acc_max = 0.75;
  }
  if( !_nh.getParam("ref_vel_max", _ref_vel_max)) {
      _ref_vel_max = 1.5;
  }
  if( !_nh.getParam("ref_omega", ref_omega)) {
      ref_omega = 1.0;
  }
  if( !_nh.getParam("ref_zita", ref_zita)) {
      ref_zita = 0.5;
  }
  
  if( !_nh.getParam("ref_o_jerk_max", ref_o_jerk_max)) {
      ref_o_jerk_max = 0.35;
  }
  if( !_nh.getParam("ref_o_acc_max", ref_o_acc_max)) {
      ref_o_acc_max = 0.75;
  }
  if( !_nh.getParam("ref_o_vel_max", ref_o_vel_max)) {
      ref_o_vel_max = 1.5;
  }


  while( !_odomOk ) usleep(0.1*1e6);

  ros::Rate r(_rate);
  double ref_T = 1.0/100.0;
    
  _cmd_p = _P;
  _ref_p = _P;
  
  Vector3d ddp;
  ddp << 0.0, 0.0, 0.0;
  Vector3d dp;  
  dp << 0.0, 0.0, 0.0;
  _ref_dp << 0.0, 0.0, 0.0;  
  _ref_ddp << 0.0, 0.0, 0.0;

  _ref_yaw = _Eta(2);
  _yaw_cmd = _Eta(2);

  _ref_dyaw = 0;
  _ref_ddyaw = 0;
  double ddyaw = 0.0;
  double dyaw = 0.0;


  Vector3d ep;
  ep << 0.0, 0.0, 0.0; 
  Vector3d jerk;
  jerk << 0.0, 0.0, 0.0;
          
  
  while( ros::ok() ) {


    if( _restarting) {

      while( !_odomOk ) usleep(0.1*1e6);
      _cmd_p = _P;
      _ref_p = _P;
      ddp << 0.0, 0.0, 0.0;
      dp << 0.0, 0.0, 0.0;
      _ref_dp << 0.0, 0.0, 0.0;  
      _ref_ddp << 0.0, 0.0, 0.0;
      _ref_yaw = _Eta(2);
      _yaw_cmd = _Eta(2);
      _ref_dyaw = 0;
      _ref_ddyaw = 0;
      ddyaw = 0.0;
      dyaw = 0.0;
      ep << 0.0, 0.0, 0.0; 
      jerk << 0.0, 0.0, 0.0;
    }
    else {
      ep = _cmd_p - _ref_p;


      double eyaw = _yaw_cmd - _ref_yaw;

      if(fabs(eyaw) > M_PI)
        eyaw = eyaw - 2*M_PI* ((eyaw>0)?1:-1);

      for(int i=0; i<3; i++ ) {
        ddp(i) = ref_omega*ref_omega * ep(i) - 2.0 * ref_zita*ref_omega*_ref_dp(i);

        jerk(i) = (ddp(i) - _ref_ddp(i))/ref_T;
        if( fabs( jerk(i) > ref_jerk_max) ) {
          if( jerk(i) > 0.0 ) jerk(i) = ref_jerk_max;
          else jerk(i) = -ref_jerk_max;
        } 

        ddp(i) = _ref_ddp(i) + jerk(i)*ref_T;
        if( fabs( ddp(i)) > ref_acc_max   ) {
          if( ddp(i) > 0.0 )
            _ref_ddp(i) = ref_acc_max;
          else 
            _ref_ddp(i) = -ref_acc_max;
        }
        else {
          _ref_ddp(i) = ddp(i);
        }

        dp(i) = _ref_dp(i) + _ref_ddp(i) * ref_T;
        if( fabs( dp(i) ) > _ref_vel_max )  {
          if( dp(i) > 0.0 ) _ref_dp(i) = _ref_vel_max;
          else _ref_dp(i) = -_ref_vel_max;
        }
        else 
          _ref_dp(i) = dp(i);

        _ref_p(i) += _ref_dp(i)*ref_T;

      }


      double ddyaw = ref_omega*ref_omega * eyaw - 2.0 * ref_zita*ref_omega*_ref_dyaw;
      double o_jerk = (ddyaw - _ref_ddyaw)/ref_T;
      if ( fabs ( o_jerk ) > ref_o_jerk_max ) {
        if( o_jerk > 0.0 ) o_jerk = ref_o_jerk_max;
        else o_jerk = -ref_o_jerk_max;
      }

      ddyaw = _ref_ddyaw + o_jerk*ref_T;
      if( fabs( ddyaw ) > ref_o_acc_max ) {
        if ( ddyaw > 0.0 ) _ref_ddyaw = ref_o_acc_max;
        else if( ddyaw < 0.0 ) _ref_ddyaw = -ref_o_acc_max;
      }
      else 
        _ref_ddyaw = ddyaw;

      dyaw = _ref_dyaw + _ref_ddyaw*ref_T;
      if( fabs( dyaw ) > ref_o_vel_max ) {
        if( dyaw > 0.0 ) dyaw = ref_o_vel_max;
        else dyaw = -ref_o_vel_max;
      }
      else 
        _ref_dyaw = dyaw;

      _ref_yaw += _ref_dyaw*ref_T;

      r.sleep();
    }
  }
}

void QUAD_CTRL::correctW(Vector4d & w2) {
  if(w2(0)<0 && w2(2)<0) {
    w2(0)=0;
    w2(2)=0;
  } 
  else {
    if (w2(0) < 0) {
      w2(2) += w2(0);
      w2(0) = 0;
    }
    if (w2(2) < 0) {
      w2(0) += w2(2);
      w2(2) = 0;
    }
  }

  if(w2(1)<0 && w2(3)<0) {
    w2(1)=0;
    w2(3)=0;
  } 
  else {
    if (w2(1) < 0) {
      w2(3) += w2(1);
      w2(1) = 0;
    }
    if (w2(3) < 0) {
      w2(1) += w2(3);
      w2(3) = 0;
    }
  }

  for (int i=0; i<4; i++) {
    if(w2(i)<0) w2(i)=0;
  }
}

void QUAD_CTRL::odom_cb( nav_msgs::OdometryConstPtr odom ) {

    geometry_msgs::Pose odom_pose;
    tf::Matrix3x3 RNed;
    RNed.setEulerYPR(M_PI/2,0,M_PI);

    tf::Vector3 p;
    tf::Vector3 pDot;
    tf::Vector3 wbb;
    tf::Vector3 wb;
    tf::Vector3 etaDot;

    tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z,  odom->pose.pose.orientation.w);
    tf::Matrix3x3 Rb(q);
    tf::Matrix3x3 RbNed = RNed*Rb*(RNed.transpose());

    RbNed.getRPY(_Eta(0), _Eta(1), _Eta(2)); //phi theta psi

    tf::matrixTFToEigen (RbNed, _Rb);
    tf::matrixTFToEigen (RNed, _RNed);

    p[0] = odom->pose.pose.position.x;
    p[1] = odom->pose.pose.position.y;
    p[2] = odom->pose.pose.position.z;

    p = RNed*p;
    _P(0) = p[0];
    _P(1) = p[1];
    _P(2) = p[2];

    _Q(0,0) = 1; _Q(0,1) = 0;             _Q(0,2) = -sin(_Eta(1));
    _Q(1,0) = 0; _Q(1,1) = cos(_Eta(0));  _Q(1,2) = cos(_Eta(1))*sin(_Eta(0));
    _Q(2,0) = 0; _Q(2,1) = -sin(_Eta(0)); _Q(2,2) = cos(_Eta(1))*cos(_Eta(0));

    pDot[0] = odom->twist.twist.linear.x;
    pDot[1] = odom->twist.twist.linear.y;
    pDot[2] = odom->twist.twist.linear.z;

    pDot = RNed*Rb*pDot;

    _P_dot(0) = pDot[0];
    _P_dot(1) = pDot[1];
    _P_dot(2) = pDot[2];

    wbb[0] = odom->twist.twist.angular.y;
    wbb[1] = odom->twist.twist.angular.x;
    wbb[2] = -odom->twist.twist.angular.z;

    _wbb(0) = wbb[0];
    _wbb(1) = wbb[1];
    _wbb(2) = wbb[2];

    _Eta_dot = _Q.inverse() * _wbb;

    _Qdot(0,0) = 0; _Qdot(0,1) = 0;                         _Qdot(0,2) = -cos(_Eta(1))*_Eta_dot(1);
    _Qdot(1,0) = 0; _Qdot(1,1) = -sin(_Eta(0))*_Eta_dot(0); _Qdot(1,2) = -sin(_Eta(1))*sin(_Eta(0))*_Eta_dot(1) + cos(_Eta(1))*cos(_Eta(0))*_Eta_dot(0);
    _Qdot(2,0) = 0; _Qdot(2,1) = -cos(_Eta(0))*_Eta_dot(0); _Qdot(2,2) = -sin(_Eta(1))*cos(_Eta(0))*_Eta_dot(1) + -cos(_Eta(1))*sin(_Eta(0))*_Eta_dot(0);

    Eigen::Matrix3d RNed_eigen;
    for(int i=0; i<3; i++ ) {
      for( int j=0; j<3; j++ ) {
        RNed_eigen(i,j) = RbNed[i][j];
      }
    }

    odom_pose.position.x = _P(0);
    odom_pose.position.y = _P(1);
    odom_pose.position.z = _P(2);

    Vector4d q_eigen = utilities::rot2quat( RNed_eigen );
    
    odom_pose.orientation.w = q_eigen(0);
    odom_pose.orientation.x = q_eigen(1);
    odom_pose.orientation.y = q_eigen(2);
    odom_pose.orientation.z = q_eigen(3);

    _NED_odom_pub.publish( odom_pose ); 
    _odomOk = true;
}

void QUAD_CTRL::cv_vel_cb( std_msgs::Float64 cv ) {
  _ref_vel_max = cv.data;
}

void QUAD_CTRL::system_reset() {

  _odomOk = false;
  gazebo_msgs::ModelState s;
  s.model_name = _model_name;
  s.pose.position.x = 0.0;
  s.pose.position.y = 0.0;
  s.pose.position.z = 0.2;
  s.pose.orientation.w = 1.0;
  s.pose.orientation.x = 0.0;
  s.pose.orientation.y = 0.0;
  s.pose.orientation.z = 0.0;
  s.twist.linear.x = 0.0;
  s.twist.linear.y = 0.0;
  s.twist.linear.z = 0.0;
  s.twist.angular.x = 0.0;
  s.twist.angular.y = 0.0;
  s.twist.angular.z = 0.0;
  
  for( int i=0; i<100; i++ ) {
    _model_state_pub.publish( s );
    usleep(0.01*1e6);
  }
  
  _cmd_p << 0.0, 0.0, 0.0;
  _cmd_dp << 0.0, 0.0, 0.0;
  _cmd_ddp << 0.0, 0.0, 0.0;
  _ref_yaw = 0.0;

  _Fe = Eigen::VectorXd::Zero(6);
  _Fe_integral = Eigen::VectorXd::Zero(6);
  _Fe_integral_out = Eigen::VectorXd::Zero(6);
}

void QUAD_CTRL::cmd_publisher() {

  _comm.angular_velocities.resize(8);

  ros::Rate r(_rate);
  while( ros::ok() ) {
    _comm.header.stamp = ros::Time::now();

    //cout << "_faults: " << _faults.transpose() << endl;
    //cout << "Omega motor " << _omega_motor.transpose() << endl;

    for(int i=0; i<8; i++ ) {
      for(int j=0; j<8; j++) {
        if ( j != i ) {
          _comm.angular_velocities[j] = 0.0  ;
        }
      
      }
      cout << "Moving motor: " << i << endl;
      _comm.angular_velocities[i] = 100.0; //_faults(0) * _omega_motor(0);
      _cmd_vel_pub.publish( _comm );
      sleep(1);
    }
    //_comm.angular_velocities[1] = 0.0; //_faults(1) * _omega_motor(1);
    //_comm.angular_velocities[2] = 0.0; //_faults(2) * _omega_motor(2);
    //_comm.angular_velocities[3] = 0.0; //_faults(3) * _omega_motor(3);
    

    //cout << "_comm.angular_velocities: " << _comm.angular_velocities[0] << ", " << _comm.angular_velocities[1] << ", " << _comm.angular_velocities[2] << ", " << _comm.angular_velocities[3] << endl;
    r.sleep();
  }
}

void QUAD_CTRL::ctrl_loop() {

  ros::Rate r(_rate);
  _omega_motor << 0.0, 0.0, 0.0, 0.0;
  Vector4d w2, controlInput;

  Vector3d zb_des;
  Vector3d yb_des;
  Vector3d xb_des;
  Matrix3d Rb_des;
  Matrix3d Q_des;
  Matrix3d Qd_des;
  Vector3d wbb_des;
  Vector3d etaDot_des;
  Vector3d eta_des;
  Vector3d wbbd_des;
  Vector3d etadd_des;
  Vector3d e3(0,0,1);

  /* Red: 0 poi gli 1-2-3 in senso antiorario
     0 frontale asse x - 1 frontale asse y
     NED: 1 frontale asse x - 0 frontale asse y
          w1 = 1, w4 = 2, w3 = 3, w2 = 0*/

  ROS_INFO("Control Active");
  while(!_odomOk) {
    usleep(0.1*1e6);
  }
  ROS_INFO("Odometry ready");

  std_msgs::Bool c_active;
  _sys_res = false;

  geometry_msgs::Vector3 ep_data;
  geometry_msgs::Vector3 er_data;
  
  while( ros::ok() ) {
    
    if( _sys_res == true ) {

      //for(int i=0; i<10;i++ )  {
        c_active.data = false;
        _controller_active.publish( c_active );
      //  usleep(0.1*1e6);
      //}
      ROS_INFO("System reset");
      _restarting = true;
      _faults << 1.0, 1.0, 1.0, 1.0;
      _omega_motor << 0.0, 0.0, 0.0, 0.0;
      system_reset();
      _sys_res = false;

      while( !_odomOk ) usleep( 0.1*1e6 );
      ROS_INFO("System reset done");

      _restarting = false;
      sleep(1);
    }
    else {
      _P_des = _ref_p;

      if(  fabs(_P_des(2)) < _l_h && (fabs( _cmd_p(2) ) < 0.4) || _cmd_p(2) > 0.0 ) {
        _omega_motor << 0.0, 0.0, 0.0, 0.0;
        _landed = true;
      }
      else {      
        _landed = false;
  
        _Pd_des =  _ref_dp;
        _Pdd_des = _ref_ddp;
        _Ep = _P - _P_des;
      
        ep_data.x = _Ep(0);
        ep_data.y = _Ep(1);
        ep_data.z = _Ep(2);

        _Ev = _P_dot - _Pd_des;

        zb_des = _Kp*_Ep + _Kv*_Ev + _m*9.81*e3 - _m*_Pdd_des;
    
        _uT = zb_des.transpose() * _Rb * e3;
        zb_des = zb_des/zb_des.norm();

        Vector4d qdes = utilities::rot2quat( utilities::XYZ2R( Vector3d(0, 0, _ref_yaw ) ) );
        //Vector4d qdes = utilities::rot2quat( utilities::XYZ2R( Vector3d(0, 0, _yaw_cmd ) ) );
        tf::Quaternion q_des(qdes[1], qdes[2], qdes[3], qdes[0]); 
        _q_des = q_des;
        wbb_des << 0.0, 0.0, _ref_dyaw; 
        wbbd_des << 0.0, 0.0, _ref_ddyaw; 
        tf::Matrix3x3 Rb_des_tf(_q_des);
        tf::matrixTFToEigen(Rb_des_tf,Rb_des);

        Rb_des = _RNed*Rb_des*(_RNed.transpose()); //Rb NED transform
        
        xb_des = Rb_des.col(0);
        yb_des = zb_des.cross(xb_des);
        yb_des = yb_des / yb_des.norm();
        xb_des = yb_des.cross(zb_des);
        Rb_des << xb_des(0), yb_des(0), zb_des(0),
                xb_des(1), yb_des(1), zb_des(1),
                xb_des(2), yb_des(2), zb_des(2);
        _Rb_des = Rb_des;

        Vector3d appo(wbb_des(1),wbb_des(0),-wbb_des(2));
        wbb_des = appo;
        _wbb_des = wbb_des;

        Vector3d appo1(wbbd_des(1),wbbd_des(0),-wbbd_des(2));
        wbbd_des = appo1;
        _wbbd_des = wbbd_des;

        
        Vector3d eu = utilities::R2XYZ(_Rb_des);
        if( fabs(eu(0)) > _rp_th ) {
          if( eu(0)  > 0.0 ) eu(0) = _rp_th;
          else eu(0) = -_rp_th;
        } 
        if( fabs(eu(1)) > _rp_th ) {
          if( eu(1)  > 0.0 ) eu(1) = _rp_th;
          else eu(1) = -_rp_th;
        } 
        
        _Rb_des = utilities::XYZ2R( eu );
        
        //cout << utilities::R2XYZ(_Rb_des).transpose() << endl;

        _Er = 0.5*Vee(_Rb_des.transpose()*_Rb - _Rb.transpose()*_Rb_des);


        er_data.x = _Er(0);
        er_data.y = _Er(1);
        er_data.z = _Er(2);


        _e_p_pub.publish( ep_data );
        _e_r_pub.publish( er_data );

        _Ew = _wbb - _Rb.transpose()*_Rb_des*_wbb_des;
        _tau_b = -_Kr*_Er - _Kw*_Ew + Skew(_wbb)*_I_b*_wbb - _I_b*( Skew(_wbb)*_Rb.transpose()*_Rb_des*_wbb_des - _Rb.transpose()*_Rb_des*_wbbd_des );
        //---

        controlInput(0) = _uT      ;
        controlInput(1) = _tau_b(0);
        controlInput(2) = _tau_b(1);
        controlInput(3) = _tau_b(2);
        w2 = _G.inverse() * controlInput;

        extWesti();

        _comm.header.stamp = ros::Time::now();

        correctW(w2);

        if (w2(0)>=0 && w2(1)>=0 && w2(2)>=0 && w2(3)>=0) {
          _omega_motor << sqrt(w2(3)), sqrt(w2(2)), sqrt(w2(1)), sqrt(w2(0));
        }
        else {
          ROS_WARN("w problem");
          cout<<w2<<endl;
        }
      }
      c_active.data = true;
      _controller_active.publish( c_active );
      std_msgs::Bool b;
      b.data = _landed;
      _land_state_pub.publish( b );
    }
    r.sleep();
  }
}

void QUAD_CTRL::goal_cb( geometry_msgs::Pose p ) {

  _cmd_p << p.position.x, p.position.y, p.position.z;
  Vector3d eu = utilities::quatToRpy( Vector4d( p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z ) );
  _yaw_cmd = eu(2); 

}

void QUAD_CTRL::insert_dest( ) {

  while(ros::ok() ) {
    cout << "Insert x, y, z" << endl;
    float x,y,z,yaw;
    scanf( "%f%f%f%f", &x, &y, &z, &yaw );

    Vector3d v;
    v << x, y, z;
    v = _RNed*v;
    _cmd_p = v;
    _yaw_cmd = yaw;

  }
}

void QUAD_CTRL::run() {
    //temp
    //boost::thread insert_dest_t ( &QUAD_CTRL::insert_dest, this);
    //boost::thread wlogs_t ( &QUAD_CTRL::wlogs, this);

    boost::thread cmd_publisher_t( &QUAD_CTRL::cmd_publisher, this);
    //boost::thread ffilter_t(&QUAD_CTRL::ffilter, this);
    //boost::thread ctrl_loop_t ( &QUAD_CTRL::ctrl_loop, this);
    ros::spin();
}

int main( int argc, char** argv) {
  ros::init(argc, argv, "SO3_UAV_controller" );
  QUAD_CTRL c;
  c.run();
  return 0;
}

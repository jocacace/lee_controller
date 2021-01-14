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
#include "std_srvs/Empty.h"

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
        QUAD_CTRL(double freq);    
        void run();
        void ctrl_loop();
        void odom_cb( nav_msgs::OdometryConstPtr );
        void ffilter();
        void insert_dest(); //Temp
        void extWesti();
        void sys_reset( std_msgs::Bool );
        void system_reset();
    private:

        void updateError();
        void correctW(Vector4d & w);

        ros::NodeHandle _nh;
        ros::Subscriber _odom_sub;
        ros::Subscriber _system_reset_req;
        ros::Publisher _model_state_pub;
        ros::Publisher _cmd_vel_pub;
        ros::Publisher _controller_active;
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

        ros::ServiceClient _pause_phy;
        ros::ServiceClient _unpause_phy;
        bool _restarting;
        
};


void QUAD_CTRL::extWesti() {
  
  
  double zita_=1.0, omega_lin=10, omega_tor=10, omega_z=omega_tor;
  MatrixXd zita(6,6), omega(6,6);
  MatrixXd K1(6,6),K2(6,6);

  zita.diagonal() << zita_,zita_,zita_,zita_,zita_,zita_;
  omega.diagonal() << omega_lin,omega_lin,omega_z,omega_tor,omega_tor,omega_z;

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

  _Fe_integral += ( internal + _Fe )*(1.0/_freq);
  _Fe_integral_out += ( -_Fe + K2*( M_xi*alpha - _Fe_integral ) )*(1.0/_freq);
  _Fe = K1*_Fe_integral_out;

  //cout << "_Fe: " << _Fe.transpose() << endl;
/*
  //Vector3d _Fe_b = _Rb.transpose()*_Fe.head(3);

  geometry_msgs::WrenchStamped est_w;
  est_w.header.stamp=ros::Time::now();
  est_w.wrench.force.x = _Fe(0);
  est_w.wrench.force.y = _Fe(1);
  est_w.wrench.force.z = _Fe(2);
  est_w.wrench.torque.x = _Fe(3);
  est_w.wrench.torque.y = _Fe(4);
  est_w.wrench.torque.z = _Fe(5);
  _est_wrench_pub.publish(est_w);
  */
}

void QUAD_CTRL::sys_reset( std_msgs::Bool d) {
  _sys_res = d.data;
}

void QUAD_CTRL::ffilter(){
  
  //Params
  double ref_jerk_max;
  double ref_acc_max;
  double ref_vel_max;
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
  if( !_nh.getParam("ref_vel_max", ref_vel_max)) {
      ref_vel_max = 1.5;
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

  ros::Rate r(100);
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
      
      cout << "ep " << ep.transpose() << endl;
      
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
        if( fabs( dp(i) ) > ref_vel_max )  {
          if( dp(i) > 0.0 ) _ref_dp(i) = ref_vel_max;
          else _ref_dp(i) = -ref_vel_max;
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

QUAD_CTRL::QUAD_CTRL(double freq) { 
    _odom_sub = _nh.subscribe("/hummingbird/ground_truth/odometry", 1, &QUAD_CTRL::odom_cb, this);
    _system_reset_req = _nh.subscribe("/lee/sys_reset", 1, &QUAD_CTRL::sys_reset, this);
    _cmd_vel_pub = _nh.advertise< mav_msgs::Actuators>("/hummingbird/command/motor_speed", 1);
    _controller_active = _nh.advertise< std_msgs::Bool >("/lee/controller_active", 1);
    _model_state_pub = _nh.advertise< gazebo_msgs::ModelState >("/gazebo/set_model_state", 1);

    _odomOk = false;

    _freq = freq;

    _P.resize(3);
    _Eta.resize(3);

    _l = 0.17; //meters
    _c_T = 8.54802e-06;
    _c_a = -0.000001;
    _m = 7.68 + 0.05;

    //Inertia matrix
    _I_b << 0.777, 0, 0,
           0, 0.777, 0,
           0, 0, 1.112;

    double mRobot = 0.97;//Kg

    Matrix3d IRobot;
    IRobot<< 0.34, 0,0,
             0, 0.37, 0,
             0, 0, 0.2;

    //_I_b+=_I_b;
    _m += mRobot;

    _G(0,0) = _c_T;    _G(0,1) = _c_T;    _G(0,2) = _c_T; _G(0,3) = _c_T;
    _G(1,0) = 0;       _G(1,1) = _l*_c_T; _G(1,2) = 0;    _G(1,3) = -_l*_c_T;
    _G(2,0) = -_l*_c_T; _G(2,1) = 0;       _G(2,2) = _l*_c_T; _G(2,3) = 0;
    _G(3,0) = -_c_a;    _G(3,1) = _c_a;    _G(3,2) = -_c_a; _G(3,3) = _c_a;

  
    ROS_WARN("Starting GEOMETRIC controller!");
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

    _pause_phy = _nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    _unpause_phy = _nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

    _restarting = false;
}

void QUAD_CTRL::odom_cb( nav_msgs::OdometryConstPtr odom ) {

    tf::Matrix3x3 RNed;
    RNed.setEulerYPR(M_PI/2,0,M_PI);
    //RNed = RNed.transpose();
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
    //cout<<odom->twist.twist.linear.x;
    pDot = RNed*Rb*pDot;
    //cout<<pDot[0];
    //pDot = RNed*Rb*pDot*RNed.transpose();

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

    _odomOk = true;
}


void QUAD_CTRL::system_reset() {

  _odomOk = false;
  gazebo_msgs::ModelState s;
  s.model_name = "hummingbird";
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

  _model_state_pub.publish( s );
   
  //stop simulation
  std_srvs::Empty epause;
  _pause_phy.call( epause );
  cout << "pause called!" << endl;
  
  ros::Rate r(5);
  for( int i=0; i<100; i++ ) {
    _model_state_pub.publish( s );
    usleep(0.01*1e6);
  }
  
  _cmd_p << 0.0, 0.0, 0.0;
  _cmd_dp << 0.0, 0.0, 0.0;
  _cmd_ddp << 0.0, 0.0, 0.0;
  _ref_yaw = 0.0;

  _unpause_phy.call(epause);
  cout << "unpause called!" << endl;

}

void QUAD_CTRL::ctrl_loop() {

  ros::Rate r(_freq);

  _comm.angular_velocities.resize(4);
  _comm.angular_velocities[0] = 0;
  _comm.angular_velocities[1] = 0;
  _comm.angular_velocities[2] = 0;
  _comm.angular_velocities[3] = 0;
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
  
  while( ros::ok() ) {
    
    if( _sys_res == true ) {

      ROS_INFO("System reset");
      _restarting = true;
      _comm.angular_velocities[0] = 0.0;
      _comm.angular_velocities[1] = 0.0;
      _comm.angular_velocities[2] = 0.0;
      _comm.angular_velocities[3] = 0.0;
      _cmd_vel_pub.publish (_comm);

      system_reset();
      _sys_res = false;

      while( !_odomOk ) usleep( 0.1*1e6 );
      ROS_INFO("System reset done");
      _restarting = false;
      
    }
    else {

      _P_des = _ref_p;
      _Pd_des =  _ref_dp;
      _Pdd_des = _ref_ddp;

      _Ep = _P - _P_des;


      //cout << "_Ep: " << _Ep.transpose() << endl;

      _Ev = _P_dot - _Pd_des;
      zb_des = _Kp*_Ep + _Kv*_Ev + _m*9.81*e3 - _m*_Pdd_des;
  
      _uT = zb_des.transpose() * _Rb * e3;
      zb_des = zb_des/zb_des.norm();

      //---TODO: add orientation
      Vector4d qdes = utilities::rot2quat( utilities::XYZ2R( Vector3d(0, 0, _ref_yaw ) ) );

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

      _Er = 0.5*Vee(_Rb_des.transpose()*_Rb - _Rb.transpose()*_Rb_des);
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
        _comm.angular_velocities[0] = sqrt(w2(3));
        _comm.angular_velocities[1] = sqrt(w2(2));
        _comm.angular_velocities[2] = sqrt(w2(1));
        _comm.angular_velocities[3] = sqrt(w2(0));
      }
      else {
        ROS_WARN("w problem");
        cout<<w2<<endl;
      }
      
      // cout<<"gain:"<<((_m*9.8)/(w2.sum())*1e6) << endl;
      _cmd_vel_pub.publish (_comm);

      c_active.data = true;
      _controller_active.publish( c_active );
    }
    r.sleep();
  }
}

void QUAD_CTRL::insert_dest( ) {

  while(ros::ok() ) {
    cout << "Insert x, y, z" << endl;
    float x,y,z,yaw;
    scanf( "%f%f%f%f", &x, &y, &z, &yaw );

    Vector3d v;
    v << x, y, z;
    _cmd_p = _RNed*v;
    _yaw_cmd = yaw;

  }
}

void QUAD_CTRL::run() {
    //temp
    boost::thread insert_dest_t ( &QUAD_CTRL::insert_dest, this);

    boost::thread ffilter_t(&QUAD_CTRL::ffilter, this);
    boost::thread ctrl_loop_t ( &QUAD_CTRL::ctrl_loop, this);
    ros::spin();
}

int main( int argc, char** argv) {
  ros::init(argc, argv, "SO3_UAV_controller" );
  QUAD_CTRL c(200);
  c.run();
  return 0;
}

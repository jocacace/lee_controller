#include "ros/ros.h"
#include "controller/lee_controller.h"
#include "extstim/extstim.h"
#include "nav_msgs/Odometry.h"
#include "boost/thread.hpp"
#include <Eigen/Eigen>
#include "gazebo_msgs/ModelStates.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32MultiArray.h"
#include "motion_planner/generate_plan.h"
#include "tf/tf.h"
#include "utils.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>      // std::fstream
#include "geometry_msgs/Wrench.h"

using namespace std;
using namespace Eigen;


class CONTROLLER {

    public:
        CONTROLLER();
        void run();
        void ctrl_loop();
        void request_new_plan();
        bool get_allocation_matrix(Eigen::MatrixXd & allocation_M, int motor_size );
        void OdometryCallback(const nav_msgs::Odometry odometry_msg);
        void fault_cb( std_msgs::Float32MultiArray fs);
        void sys_reset( std_msgs::Bool );
        void system_reset();
        void ffilter();
        void cmd_publisher();
        void goal_cb( geometry_msgs::Pose p );
        void test_motors(int motors);
        void write_logs();

    private:

        //---Parameters
        string _model_name;
        int _ctrl_rate;
        int _motor_num;
        Eigen::Matrix3d _inertia;
        Eigen::Vector3d _position_gain;
        Eigen::Vector3d _velocity_gain;
        Eigen::Vector3d _attitude_gain;
        Eigen::Vector3d _angular_rate_gain;
        Eigen::VectorXd _omega_motor;
        double _mass;
        double _gravity;
        vector<double> _rotor_angles;
        vector<double> _arm_length;
        double _motor_force_k;
        double _motor_moment_k;
        vector<int> _motor_rotation_direction;
        //---

        Eigen::Vector3d _perror;
        Eigen::Vector3d _verror;

        ros::NodeHandle _nh;
        ros::Subscriber _odom_sub;
        ros::Subscriber _system_reset_req;
        ros::Subscriber _goal_sub;
        ros::Subscriber _fault_pub;
        ros::Publisher _e_r_pub;
        ros::Publisher _land_state_pub;
        ros::Publisher _cmd_vel_motor_pub;
        ros::Publisher _model_state_pub;
        ros::Publisher _controller_active;
        ros::Publisher _ext_w_pub;

        nav_msgs::Odometry _odom;
        motion_planner::generate_plan _plan;

        bool _first_odom;
        bool _new_plan;

        bool _sys_res;
        double _spawning_z;
        Vector3d _ref_p;
        Vector3d _ref_dp;
        Vector3d _ref_ddp;
        double _ref_yaw;
        double _ref_vel_max;
        int _rate;
        bool _restarting;
        Vector3d _cmd_p;
        Vector3d _cmd_dp;
        Vector3d _cmd_ddp;
        
        Vector3d _Eta;
        Vector3d _Eta_dot;
        double _yaw_cmd;
        double _ref_dyaw;
        double _ref_ddyaw;
        double _l_h; //Landing altitude     
        bool _landed;
        bool _test_motors;
        Eigen::Vector3d _mes_p;
        Eigen::Vector3d _mes_dp;
        bool _save_logs;

        double _ext_zita;
        double _ext_omega_lin;
        double _ext_omega_z;
        double _ext_omega_tor;

        //Vector4d _faults;
        VectorXd _faults;
};



CONTROLLER::CONTROLLER(): _first_odom(false), _new_plan(false) {


    //Load params
    if( !_nh.getParam("model_name", _model_name) ) {
        _model_name =  "iris_smc";
    }
    if( !_nh.getParam("control_rate", _ctrl_rate) ) {
        _ctrl_rate =  100;
    }
    if( !_nh.getParam("motor_num", _motor_num) ) {
        _motor_num =  4;
    }
    if( !_nh.getParam("spawning_z", _spawning_z )) {
        _spawning_z = 0.2;
    }
    if( !_nh.getParam("test_motors", _test_motors)) {
      _test_motors = false;
    }
    if( !_nh.getParam("save_logs", _save_logs) ) {
      _save_logs = false;
    }
    if( !_nh.getParam("ext_zita", _ext_zita)) {
      _ext_zita = 1.0;  
    }
    if( !_nh.getParam("ext_omega_lin", _ext_omega_lin)) {
      _ext_omega_lin = 5.0;  
    }
    if( !_nh.getParam("ext_omega_tor",_ext_omega_tor)) {
      _ext_omega_tor = 5.0;  
    }
    if( !_nh.getParam("ext_omega_z", _ext_omega_z)) {
      _ext_omega_z = 5.0;  
    }

    vector<double> inertia;
    if( !_nh.getParam("inertia", inertia) ) {
        inertia.resize(3);
        inertia[0] = 1.0;
        inertia[1] = 1.0;
        inertia[2] = 1.0;
    }
    _inertia = Eigen::Matrix3d( Eigen::Vector3d( inertia[0], inertia[1], inertia[2] ).asDiagonal() );

    vector<double> kp;
    if( !_nh.getParam("kp", kp) ) {
        kp.resize(3);
        kp[0] = 1.0;
        kp[1] = 1.0;
        kp[2] = 1.0;
    }
    _position_gain << kp[0], kp[1], kp[2];
    
    vector<double> kd;
    if( !_nh.getParam("kd", kd) ) {
        kd.resize(3);
        kd[0] = 1.0;
        kd[1] = 1.0;
        kd[2] = 1.0;
    }
    _velocity_gain << kd[0], kd[1], kd[2];

    vector<double> attitude_gain;
    if( !_nh.getParam("attitude_gain", attitude_gain) ) {
        attitude_gain.resize(3);
        attitude_gain[0] = 1.0;
        attitude_gain[1] = 1.0;
        attitude_gain[2] = 1.0;
    }
    _attitude_gain << attitude_gain[0], attitude_gain[1], attitude_gain[2];
    
    vector<double> angular_rate_gain;
    if( !_nh.getParam("angular_rate_gain", angular_rate_gain) ) {
        angular_rate_gain.resize(3);
        angular_rate_gain[0] = 1.0;
        angular_rate_gain[1] = 1.0;
        angular_rate_gain[2] = 1.0;
    }
    _angular_rate_gain << angular_rate_gain[0], angular_rate_gain[1], angular_rate_gain[2];
    
    if( !_nh.getParam("mass", _mass) ) {
        _mass =  1.5;
    }
    if( !_nh.getParam("gravity", _gravity) ) {
        _gravity =  9.81;
    }

    if( !_nh.getParam( "rotor_angle", _rotor_angles ) ) {
        _rotor_angles.resize( 4 );
        _rotor_angles[0] = -0.5337;
        _rotor_angles[1] = 2.565;
        _rotor_angles[2] = 0.5337;
        _rotor_angles[3] = -2.565;
    }

    if( !_nh.getParam( "arm_length", _arm_length ) ) {
        _arm_length.resize( 4 );
        _arm_length[0] = 0.255;
        _arm_length[1] = 0.238;
        _arm_length[2] = 0.255;
        _arm_length[3] = 0.238;
    }


    if( !_nh.getParam( "motor_rotation_direction", _motor_rotation_direction ) ) {
        _motor_rotation_direction.resize( 4 );
        _motor_rotation_direction[0] = 1;
        _motor_rotation_direction[1] = 1;
        _motor_rotation_direction[2] = -1;
        _motor_rotation_direction[3] = -1;
    }

    if( !_nh.getParam("motor_force_k", _motor_force_k) ) {
        _motor_force_k =  8.54858e-06;
    }
    if( !_nh.getParam("motor_moment_k", _motor_moment_k) ) {
        _motor_moment_k = 1.6e-2;
    }
    if( !_nh.getParam("rate", _rate)) {
         _rate = 200;
    }

    _odom_sub = _nh.subscribe(_model_name + "/odometry", 0, &CONTROLLER::OdometryCallback, this);
    _cmd_vel_motor_pub = _nh.advertise<std_msgs::Float32MultiArray>( _model_name + "/cmd/motor_vel", 0);
    _system_reset_req = _nh.subscribe("/lee/sys_reset", 1, &CONTROLLER::sys_reset, this);
    _model_state_pub = _nh.advertise< gazebo_msgs::ModelState >("/gazebo/set_model_state", 1);
    _controller_active = _nh.advertise< std_msgs::Bool >("/lee/controller_active", 1);
    _goal_sub = _nh.subscribe< geometry_msgs::Pose > ("/lee/goal", 1, &CONTROLLER::goal_cb, this );
    _land_state_pub = _nh.advertise< std_msgs::Bool > ("/lee/landed", 1);
    _ext_w_pub = _nh.advertise<geometry_msgs::Wrench>("/lee/ext_wrench", 1);
    _fault_pub = _nh.subscribe("/lee/faults", 1, &CONTROLLER::fault_cb, this);
    _e_r_pub = _nh.advertise<geometry_msgs::Vector3>("/lee/er", 1);
    _sys_res = false;
    _restarting = false;

    _cmd_p << 0.0, 0.0, 0.0;
    _cmd_dp << 0.0, 0.0, 0.0;
    _cmd_ddp << 0.0, 0.0, 0.0;
    _ref_yaw = 0.0;
    _l_h = 0.25;
    _landed = true;
    _Eta.resize(3);

    _faults.resize(_motor_num);
    _omega_motor.resize(_motor_num);
    for( int i=0; i<_motor_num; i++ ) {
      _omega_motor[i] = 0.0;
      _faults[i] = 1.0;
    }
}

void CONTROLLER::sys_reset( std_msgs::Bool d) {
  _sys_res = d.data;
}

void CONTROLLER::fault_cb( std_msgs::Float32MultiArray fs) {
  _faults << (1.0-fs.data[0]), (1.0-fs.data[1]), (1.0-fs.data[2]), (1.0-fs.data[3]);
}

void CONTROLLER::request_new_plan() {

    float x, y, z, yaw;
    ros::ServiceClient client = _nh.serviceClient<motion_planner::generate_plan>("generate_tarjectory");

    while(ros::ok()) {

        cout << "Insert new coordinates x (front), y (left), z (upword), yaw (c-clowise)" <<endl;
        scanf("%f %f %f %f", &x, &y, &z, &yaw);
        cout << "Request new plan for: [" << x << ", " << y << ", " << z << " - " << yaw << "]" << endl;

        //---flu -> NED
        _cmd_p << x, -y, -z;
        _yaw_cmd = -yaw;
        //---
    }
}

void CONTROLLER::write_logs() {

  ros::Rate r(100);

  std::fstream log_file;
  log_file.open ("/tmp/lee_logs.txt");

  if (log_file.is_open())
  {
    cout << "lorem ipsum" << endl;
  }
  else
  {
    std::cout << "Error opening file" << endl;
    exit(0);
  }

  while( ros::ok() ) {
    log_file << _mes_p[0] << ", " << _mes_p[1] << ", " << _mes_p[2] << ", " << _ref_p[0] << ", " << _ref_p[1] << ", " << _ref_p[2] << ", " << 
              _mes_dp[0] << ", " << _mes_dp[1] << ", " << _mes_dp[2] << ", " << _ref_dp[0] << ", " << _ref_dp[1] << ", " << _ref_dp[2] << ", " << 
              _ref_ddp[0] << ", " << _ref_ddp[1] << ", " << _ref_ddp[2] << ","  << 
              _perror[0] << ", " << _perror[1] << ", " << _perror[2] << ", " <<
                _verror[0] << ", " << _verror[1] << ", " << _verror[2] << endl;

    
    r.sleep();
  }

  log_file.close();
}

bool generate_allocation_matrix(Eigen::MatrixXd & allocation_M, 
                                    int motor_size,
                                    vector<double> rotor_angle,
                                    vector<double> arm_length, 
                                    double force_k,
                                    double moment_k,
                                    vector<int> direction ) {

    allocation_M.resize(4, motor_size );

    for(int i=0; i<motor_size; i++ ) {
        allocation_M(0, i) = sin( rotor_angle[i] ) * arm_length[i] * force_k;
        allocation_M(1, i) = cos( rotor_angle[i] ) * arm_length[i] * force_k;
        allocation_M(2, i) = direction[i] * force_k * moment_k;
        allocation_M(3, i) = -force_k;
    }

    Eigen::FullPivLU<Eigen::Matrix4Xd> lu( allocation_M);
    if ( lu.rank() < 4 ) {
        ROS_ERROR("The allocation matrix rank is lower than 4. This matrix specifies a not fully controllable system, check your configuration");
        return false;
    }
    return true;
}


void CONTROLLER::system_reset() {

  _first_odom = false;
  gazebo_msgs::ModelState s;
  s.model_name = _model_name;
  s.pose.position.x = 0.0;
  s.pose.position.y = 0.0;
  s.pose.position.z = _spawning_z;
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
  for( int i=0; i<_motor_num; i++ ) {
    //_omega_motor[i] = 0.0;
    _faults[i] = 1.0;
  }
}


void CONTROLLER::ffilter(){
  
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


  while( !_first_odom ) usleep(0.1*1e6);

  ros::Rate r(_rate);
  double ref_T = 1.0/(double)_rate;
    
  _cmd_p << _odom.pose.pose.position.x, _odom.pose.pose.position.y, _odom.pose.pose.position.z;
  _ref_p = _cmd_p;
  
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

      while( !_first_odom ) usleep(0.1*1e6);
      _cmd_p << _odom.pose.pose.position.x, _odom.pose.pose.position.y, _odom.pose.pose.position.z;
      _ref_p = _cmd_p;
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
        
      //cout << "_cmd_p: " << _cmd_p.transpose() << endl;
      //cout << "_ref_f: " << _ref_p.transpose() << endl;

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

void CONTROLLER::cmd_publisher() {

    ros::Rate r(_rate);
    std_msgs::Float32MultiArray motor_vel;
    motor_vel.data.resize( _motor_num );
    
    while( ros::ok() ) {
        for( int i=0; i<_motor_num; i++ ) {
          motor_vel.data[i] = _faults(i) * _omega_motor(i);
        }
        /*
        motor_vel.data[0] = _faults(0) * _omega_motor(0);
        motor_vel.data[1] = _faults(1) * _omega_motor(1);
        motor_vel.data[2] = _faults(2) * _omega_motor(2);
        motor_vel.data[3] = _faults(3) * _omega_motor(3);        
        */
        _cmd_vel_motor_pub.publish( motor_vel );

        r.sleep();
    }
}

void CONTROLLER::goal_cb( geometry_msgs::Pose p ) {

  _cmd_p << p.position.x, p.position.y, p.position.z;
  Vector3d eu = utilities::quatToRpy( Vector4d( p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z ) );
  _yaw_cmd = eu(2); 

}

void CONTROLLER::ctrl_loop() {


    ros::Rate r(_ctrl_rate);

    //---Input
    Eigen::Vector3d des_p;              
    Eigen::Vector3d des_dp; 
    Eigen::Vector3d des_ddp; 
    des_dp << 0.0, 0.0, 0.0;
    des_ddp << 0.0, 0.0, 0.0;

    Eigen::Quaterniond mes_q;
    Eigen::Vector3d mes_dp;    
    Eigen::Vector3d mes_w;

    //---

    Eigen::MatrixXd allocation_M;
    Eigen::MatrixXd wd2rpm;
    std_msgs::Bool c_active;
    c_active.data = false;

    while( !_first_odom ) usleep(0.1*1e6);
    _mes_p << _odom.pose.pose.position.x, _odom.pose.pose.position.y, _odom.pose.pose.position.z;
    _mes_dp << _odom.twist.twist.linear.x, _odom.twist.twist.linear.y, _odom.twist.twist.linear.z;

    if(!generate_allocation_matrix( allocation_M, _motor_num, _rotor_angles, _arm_length, _motor_force_k, _motor_moment_k, _motor_rotation_direction ) ) {     
        cout << "Wrong allocation matrix" << endl;
        exit(0);
    }

    boost::thread input_t( &CONTROLLER::request_new_plan, this);
    boost::thread ffilter_t(&CONTROLLER::ffilter, this);
    boost::thread cmd_publisher_t(&CONTROLLER::cmd_publisher, this);

    wd2rpm.resize( _motor_num, 4 );
    Eigen::Matrix4d I;
    I.setZero();
    I.block<3, 3>(0, 0) = _inertia;
    I(3, 3) = 1;
    
    LEE_CONTROLLER lc;
    lc.set_uav_dynamics( _motor_num, _mass, _gravity, I);
    lc.set_controller_gains( _position_gain, _velocity_gain, _attitude_gain, _angular_rate_gain );
    lc.set_allocation_matrix( allocation_M );

    EXTSTIM ext;
    Eigen::Matrix<double,6,6> K1;
    Eigen::Matrix<double,6,6> K2;
    K1.diagonal() << 2.0, 2.0, 10.0, 2.0, 2.0, 2.0;
    K2.diagonal() << 0.5, 0.5, 2.5, 0.5, 0.5, 0.5;

    ext.set_dyn_param( _mass, _gravity, I.block<3,3>(0,0));
    ext.set_gains( _ext_zita, _ext_omega_lin, _ext_omega_z, _ext_omega_tor );

    //double _zita; // = 1.0;
    //double _omega_lin; // = 5;
    //double _omega_tor; // = 5;
    //double _omega_z;            
        
    int plan_index = 0;
    Eigen::VectorXd ref_rotor_velocities;
    Eigen::Vector4d ft;
    Eigen::Vector3d perror;
    Eigen::Vector3d verror;

    std::fstream log_file;
    if( _save_logs ) {
        log_file.open ("/tmp/lee_logs.txt", std::fstream::in | std::fstream::out | std::fstream::out);

      if (log_file.is_open()) {
        cout << "lorem ipsum" << endl;
      }
      else {
        std::cout << "Error opening file" << endl;
        exit(0);
      }
    }

    Vector3d ext_f;
    Vector3d ext_t;
    Vector3d att_err;
    geometry_msgs::Vector3 att_err_data;
    geometry_msgs::Wrench extW;

    while( ros::ok() ) {

        //Measured
        _mes_p << _odom.pose.pose.position.x, _odom.pose.pose.position.y, _odom.pose.pose.position.z;
        _mes_dp << _odom.twist.twist.linear.x, _odom.twist.twist.linear.y, _odom.twist.twist.linear.z; 
        mes_q = Eigen::Quaterniond( _odom.pose.pose.orientation.w, _odom.pose.pose.orientation.x, _odom.pose.pose.orientation.y, _odom.pose.pose.orientation.z );
        mes_w << _odom.twist.twist.angular.x, _odom.twist.twist.angular.y, _odom.twist.twist.angular.z;  

        if( _sys_res == true ) {
            c_active.data = false;
            _controller_active.publish( c_active );
            ROS_INFO("System reset");
            _restarting = true;
            for( int i=0; i<_motor_num; i++ ) 
              _omega_motor[i] = 0.0;
            system_reset();
            _sys_res = false;
            ext.reset();
        }
        else {
          _restarting = false;
          c_active.data = true;

          if( ( fabs(_ref_p(2)) < _l_h) && ( (fabs( _cmd_p(2)  < 0.4) ) || _cmd_p(2) > 0.0 ) ) {
              for( int i=0; i<_motor_num; i++ ) 
                _omega_motor[i] = 0.0;
            
              _landed = true;
              ext.reset();
          }
          else {

              Eigen::Matrix3d R = mes_q.toRotationMatrix();
              mes_w = R.transpose()*mes_w;

              lc.controller(_mes_p, _ref_p, mes_q, _mes_dp, _ref_dp, _ref_ddp, _ref_yaw, mes_w, &ref_rotor_velocities, &ft, &_perror, &_verror, &att_err);   
              ext.estimation(R, mes_w, _mes_dp, ft[3], Vector3d( ft[0], ft[1], ft[2] ), double( _ctrl_rate ), ext_f, ext_t );        
            
              extW.force.x = ext_f[0];
              extW.force.y = ext_f[1];
              extW.force.z = ext_f[2];

              extW.torque.x = ext_t[0];
              extW.torque.y = ext_t[1];
              extW.torque.z = ext_t[2];

              //cout << "att_err: " << att_err.transpose() << endl;
              att_err_data.x = att_err[0];
              att_err_data.y = att_err[1];
              att_err_data.z = att_err[2];

              for(int i=0; i<_motor_num; i++ ) {
                  _omega_motor[i] = ref_rotor_velocities[i]; 
                  cout << i << ": " << _omega_motor[i] << endl;
              }
              cout << endl;
              
              _landed = false;

              _ext_w_pub.publish( extW );
              _e_r_pub.publish( att_err_data );
          }

          if( _save_logs ) {
            log_file << _mes_p[0] << ", " << _mes_p[1] << ", " << _mes_p[2] << ", " <<   //1:3
                        _ref_p[0] << ", " << _ref_p[1] << ", " << _ref_p[2] << ", " <<   //4:6
                        _mes_dp[0] << ", " << _mes_dp[1] << ", " << _mes_dp[2] << ", " << //7:9
                        _ref_dp[0] << ", " << _ref_dp[1] << ", " << _ref_dp[2] << ", " <<  //10:12
                        _ref_ddp[0] << ", " << _ref_ddp[1] << ", " << _ref_ddp[2] << ","  <<  //13:15
                        _perror[0] << ", " << _perror[1] << ", " << _perror[2] << ", " << //16:18
                        _verror[0] << ", " << _verror[1] << ", " << _verror[2] << ", " << //19:21
                        _cmd_p[0] << ", " << _cmd_p[1] << ", " << _cmd_p[2] << endl;      //22:24
          }

          _controller_active.publish( c_active );

          std_msgs::Bool b;
          b.data = _landed;
          _land_state_pub.publish( b );   
        }
        r.sleep();
    }
}

void CONTROLLER::OdometryCallback(const nav_msgs::Odometry odometry_msg) {
    _odom = odometry_msg;
    _Eta = utilities::R2XYZ( utilities::QuatToMat ( Vector4d( _odom.pose.pose.orientation.w,  _odom.pose.pose.orientation.x, _odom.pose.pose.orientation.y, _odom.pose.pose.orientation.z ) ) );
    _first_odom = true;
}


void CONTROLLER::test_motors(int num_motor) {
  std_msgs::Float32MultiArray motor_vel;
  motor_vel.data.resize( num_motor );
  ros::Rate r(10);
  while(ros::ok()) {
    for(int i=0;i<num_motor;i++) {
        cout << "Test motor: " << i <<endl;
        for(int k=0;k<num_motor;k++) {
          if ( k!= i)
            motor_vel.data[k] = 0;
        }
        for(int j=0; j<10; j++) {
          motor_vel.data[i] = 300;
          _cmd_vel_motor_pub.publish( motor_vel );
          r.sleep();
        }
    }
    r.sleep();
  }
}

void CONTROLLER::run() {
  if ( _test_motors )
      boost::thread test_motors_t( &CONTROLLER::test_motors, this, 6 );
  else
    boost::thread ctrl_loop_t( &CONTROLLER::ctrl_loop, this );
    
  //boost::thread write_logs_t(&CONTROLLER::write_logs, this);
  ros::spin();
}

int main( int argc, char** argv ) {

    ros::init(argc, argv, "lee_controller");
    CONTROLLER c;
    c.run();

    return 0;
}
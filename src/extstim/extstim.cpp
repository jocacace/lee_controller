#include "extstim.h"

EXTSTIM::EXTSTIM() {
    
    _Fe.resize(6);
    _Fe = Eigen::VectorXd::Zero(6);
    _Fe_integral.resize(6);
    _Fe_integral = Eigen::VectorXd::Zero(6);
    _Fe_integral_out.resize(6);
    _Fe_integral_out = Eigen::VectorXd::Zero(6);
}


void EXTSTIM::set_dyn_param( double mass_, double gravity_, Eigen::Matrix3d Ib_ ) {
    _mass = mass_;
    _Ib = Ib_;
}

void EXTSTIM::set_gains( double zita_, double omega_lin_, double omega_z_, double omega_tor_ ) {
    _zita = zita_;
    _omega_lin = omega_lin_;
    _omega_z = omega_z_;
    _omega_tor = omega_tor_;
}

        
void EXTSTIM::reset( ) {
    //TODO:reset
}

/*
void EXTSTIM::Odomcallback(const nav_msgs::Odometry odometry_){
    _eta = utilities::R2XYZ( utilities::QuatToMat ( Vector4d( odometry_.pose.pose.orientation.w,  odometry_.pose.pose.orientation.x, odometry_.pose.pose.orientation.y, odometry_.pose.pose.orientation.z ) ) );
    _eta_dot << odometry_.twist.twist.angular.x, odometry_.twist.twist.angular.y, odometry_.twist.twist.angular.z;
    _vel << odometry_.twist.twist.linear.x, odometry_.twist.twist.linear.y, odometry_.twist.twist.linear.z;
    _first_odom = true;
}
*/

/*
void EXTSTIM::ForceThrustcallback(const std_msgs::Float32MultiArray force_thrust){
    _force = force_thrust.data[3];
    _tau_b << force_thrust.data[0],force_thrust.data[1],force_thrust.data[2];
}
*/


void EXTSTIM::estimation(Eigen::Matrix3d Rb, Eigen::Vector3d eta_dot, Eigen::Vector3d vel, double force, Eigen::Vector3d tau_b, double Tc, Vector3d & extF, Vector3d & extT ) {
    
    //geometry_msgs::Wrench est_w;

    //double zita_ = 1.0;
    //double omega_lin = 5;
    //double omega_tor = 5;
    //double omega_z = omega_tor;

    MatrixXd zita(6,6), omega(6,6);
    MatrixXd K1(6,6), K2(6,6);
    zita = MatrixXd::Identity(6,6);
    omega = MatrixXd::Identity(6,6);
    
    zita.diagonal() << _zita, _zita, _zita, _zita, _zita, _zita;
    omega.diagonal() << _omega_lin, _omega_lin, _omega_z, _omega_tor, _omega_tor, _omega_z;
    
    K1 = 2*zita*omega;
    K2 = omega*omega*K1.inverse();

    Vector3d e3(0,0,1);

    MatrixXd M_xi(6,6);
    M_xi << _mass*MatrixXd::Identity(3,3) , MatrixXd::Zero(3,3),
    MatrixXd::Zero(3,3) , _Ib;

    VectorXd alpha(6);

    alpha.head(3) = Rb.transpose()*vel;
    alpha.tail(3) = eta_dot;

    VectorXd internal(6);
    internal.head(3) = force*e3 + _mass*9.81*Rb.transpose()*e3;
    internal.tail(3) = tau_b - utilities::skew(eta_dot)*_Ib*eta_dot;


    _Fe_integral += ( internal + _Fe )*(1.0/Tc);
    if( !isnan(_Fe_integral.norm()) ) {
        _Fe_integral_out += ( -_Fe + K2*( M_xi*alpha - _Fe_integral ) )*(1.0/Tc);
        _Fe = K1*_Fe_integral_out;
    }

    extF << _Fe[0], _Fe[1],  _Fe[2]; 
    extT << _Fe[3], _Fe[4],  _Fe[5]; 


    /*
    est_w.force.x = _Fe(0);
    est_w.force.y = _Fe(1);
    est_w.force.z = _Fe(2);
    est_w.torque.x = _Fe(3);
    est_w.torque.y = _Fe(4);
    est_w.torque.z = _Fe(5);
    _est_wrench_pub.publish(est_w);
    */
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    /*
    Matrix3d Jacobian;
    Matrix3d Jacobian_dot;
    Eigen::Matrix<double,6,1> momentum;
    Jacobian(0,0) = 1; 
    Jacobian(0,1) = 0;
    Jacobian(1,0) = 0;
    Jacobian(2,0) = 0;
    Jacobian(0,2) = -sin(eta(2));
    Jacobian(1,1) =  cos(eta(1));
    Jacobian(1,2) =  cos(eta(2))*sin(eta(1));
    Jacobian(2,1) = -sin(_eta(1));
    Jacobian(2,2) =  cos(_eta(2))*cos(_eta(1));

    Jacobian_dot(0,0) = 0;
    Jacobian_dot(0,1) = 0;
    Jacobian_dot(1,0) = 0;
    Jacobian_dot(2,0) = 0;
    Jacobian_dot(0,2) = -cos(_eta(2))*_eta_dot(2);
    Jacobian_dot(1,1) = -sin(_eta(1))*_eta_dot(1);
    Jacobian_dot(1,2) = (-sin(_eta(2))*sin(_eta(1))*_eta_dot(2))+(cos(_eta(1))*cos(_eta(2))*_eta_dot(1));
    Jacobian_dot(2,1) = -cos(_eta(1))*_eta_dot(1);
    Jacobian_dot(2,2) = (-sin(_eta(2))*cos(_eta(1))*_eta_dot(2))-(sin(_eta(1))*cos(_eta(2))*_eta_dot(1));

    Vector3d omega;
    omega = Jacobian*eta_dot;

    Matrix3d S_c;
    Matrix3d C;

    S_c(0,0) = 0;
    S_c(0,1) = -omega(2);
    S_c(0,2) = omega(1);
    S_c(1,0) = omega(2);
    S_c(1,1) = 0;
    S_c(1,2) = -omega(0);
    S_c(2,0) = -omega(1);
    S_c(2,1) = omega(0);
    S_c(2,2) = 0;

   
    C = (Jacobian.transpose()*S_c*_Ib*Jacobian) + (Jacobian.transpose()*_Ib*Jacobian_dot);
    Vector3d ez(Eigen::Vector3d::UnitZ());
    Vector3d Rb_col3;

    Rb_col3 = utilities::XYZ2R(eta)*ez ;
    /*
    Vector3d force_temp;
    force_temp = (-force*Rb_col3) + (_mass*_gravity*ez);
    
    Vector3d torque_temp;
    torque_temp = (Jacobian.transpose()*tau_b) + (C.transpose()*eta_dot);
    
    Eigen::Matrix<double,6,1> momentum_dot;
    momentum_dot << force_temp, torque_temp;
 
    Matrix3d M;
    M = Jacobian.transpose()*_Ib*Jacobian;
    Matrix<double,6,6> momentum_temp;

    momentum_temp << _mass*Matrix3d::Identity(), Matrix3d::Zero(), Matrix3d::Zero(), M;
    Matrix<double,6,1> force_torque;
       
    force_torque << vel, eta_dot;
    momentum = momentum_temp * force_torque;

    _stima_dot = _stima + momentum_dot;
    _stima_onetemp = _stima_onetemp + (_stima_dot*(1.0/Tc));
    _stima_twotemp = _K2*( momentum- _stima_onetemp);
    _stima_threetemp = _stima_twotemp - _stima;
    _stima_fourtemp = _stima_fourtemp + (_stima_threetemp*(1.0/Tc));

   _stima = _K1*_stima_fourtemp;
    cout << "stime: " << _stima.transpose() << endl;
   
    //for(int i=0;i<6;i++){
    //    stima_msg.data[i]=_stima(i);
    //    cout<<"stima("<<i<<"): "<<_stima(i)<<endl;
    //}
   
   //else {
   //     _stima<<0,0,0,0,0,0;
   //     for(int i=0;i<6;i++){
   //     stima_msg.data[i]=_stima(i);
   //     cout<<"stima("<<i<<"): "<<_stima(i)<<endl;
   // }
   //}
      
     */
    //_momentum=momentum_temp*force_torque;


}

/*
void EXTSTIM::momentum(){
  

    Matrix3d M;
    M=_Jacobian.transpose()*_Ib*_Jacobian;

    Matrix<double,6,6> momentum_temp;
    momentum_temp<<_mass*Matrix3d::Identity(),Matrix3d::Zero(),
                    Matrix3d::Zero(),M;
    
    Matrix<double,6,1> force_torque;
    force_torque<<_vel,_eta_dot;
    
    _momentum=momentum_temp*force_torque;
  // cout<<"Momentum="<<_momentum<<endl; 
}
*/
/*
void EXTSTIM::stim(){


  Matrix<double,6,1> stima_dot;
  Matrix<double,6,1> stima_onetemp;
  Matrix<double,6,1> stima_twotemp;
  Matrix<double,6,1> stima_threetemp;
  Matrix<double,6,1> stima_fourtemp;
  stima_dot=Matrix<double,6,1>::Zero();
  stima_onetemp=Matrix<double,6,1>::Zero();
  stima_twotemp=Matrix<double,6,1>::Zero();
  stima_threetemp=Matrix<double,6,1>::Zero();
  stima_fourtemp=Matrix<double,6,1>::Zero();

  ros::Rate rate(100);
  std_msgs::Float32MultiArray stima_msg;
  //std_msgs::Bool run_msg;
  stima_msg.data.resize(6);
 
  while(ros::ok()){






   momentum_dot();
   momentum();
   
   if(_first_odom==1){
   stima_dot=_stima+_momentum_dot;
   //cout<<"stima_dot="<<stima_dot<<endl;
   stima_onetemp=stima_onetemp+(stima_dot*(1.0/100.0));
   //cout<<"stima_onetemp="<<stima_onetemp<<endl;
   stima_twotemp=_K2*(_momentum-stima_onetemp);
   //cout<<"stima_twotemp="<<stima_twotemp<<endl;
   stima_threetemp=stima_twotemp-_stima;
   //cout<<"stima_threetemp="<<stima_threetemp<<endl;
    stima_fourtemp=stima_fourtemp+(stima_threetemp*(1.0/100.0));
   _stima=_K1*stima_fourtemp;
   //cout<<"stima="<<_stima<<endl; 
   
   
    for(int i=0;i<6;i++){
        stima_msg.data[i]=_stima(i);
        cout<<"stima("<<i<<"): "<<_stima(i)<<endl;
    }
   }
   else {
        _stima<<0,0,0,0,0,0;
        for(int i=0;i<6;i++){
        stima_msg.data[i]=_stima(i);
        cout<<"stima("<<i<<"): "<<_stima(i)<<endl;
    }
   }
      
        

   rate.sleep();
   ros::spinOnce();
  }
      
    
}

*/
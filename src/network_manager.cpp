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
#include "std_msgs/Float32MultiArray.h"
#include "gazebo_msgs/ModelState.h"
#include "std_srvs/Empty.h"

class NET_MANAGER{
    public:
    void run();
    void insert_point();
    void net_loop();
    void odom_cb( nav_msgs::OdometryConstPtr );

    private:
    ros::NodeHandle _nh;
    ros::Subscriber _odom_sub;
    ros::Publisher _point_pub;
    Vector3d _P_des;
    Vector3d _P;
    Vector3d _P_dot;
    Vector3d _Eta;
    Vector3d _Eta_dot;
    Vector3d _etaDot_des, _etadd_des;
    Vector3d _wbb;
    Matrix3d _Rb, _Rb_des;
    Matrix3d _RNed;
    float _yaw_des;

}
 NET_MANAGER::net_manager(){
     _odom_sub = _nh.subscribe("/hummingbird/ground_truth/odometry", 1, &QUAD_CTRL::odom_cb, this);
     _point_pub = _nh.advertise< std_msgs::Float32MultiArray>("/hummingbird/point", 0);

 }
 void NET_MANAGER::odom_cb( nav_msgs::OdometryConstPtr odom ) {

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

}
void NET_MANAGER::insert_point(){
    while(ros::ok() ) {
    cout << "Insert x, y, z e yaw" << endl;
    float x,y,z,yaw;
    scanf( "%f%f%f%f", &x, &y, &z, &yaw );

    Vector3d v;
    v << x, y, z;
    _P_des = _RNed*v;
    _yaw_des = yaw;

  }
}
void NET_MANAGER::net_loop(){
   Vector3d err_p;
   float err_yaw;
   ros::Rate r();
    while(ros::ok()){
        err_p= _P_des-_P;
        err_yaw= _yaw_des-_Eta(2);
        if (err_p[0]==0 && err_p[1]==0 && err_p[2]==0 && err_yaw==0){
            insert_point();
        }
        rate.sleep();
        ros::spinOnce();
    }
    

}
void NET_MANAGER::run(){
    main();
    ros::spin();
}
int main(int argc,char** argv){
    ros::init(argc,argv,"net_man");
    NET_MANAGER nm;
    nm.run();
    return 0;
}

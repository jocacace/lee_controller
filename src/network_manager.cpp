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
#include "geometry_msgs/Pose.h"
#include <random>
#include "std_msgs/Float64.h"
#include "planner_spline.h"
#include "std_msgs/Float32MultiArray.h"

class NET_MANAGER {
    public:
        NET_MANAGER();
        void run();
        void insert_point();
        void net_loop();
        void odom_cb( geometry_msgs::Pose odom );
        Vector4d get_point_k();
        void land_state_cb( std_msgs::Bool b );
        //geometry_msgs::Pose takeoff();
        void takeoff();
        void move_to( Vector4d wp, double cv  );
        void ctrl_active( std_msgs::Bool b );
        void gen_fault();
        void test_fault();

    private:
        ros::NodeHandle _nh;
        ros::Subscriber _odom_sub;
        ros::Subscriber _land_state_sub;
        ros::Subscriber _controller_active_sub;
        ros::Publisher _point_pub;
        ros::Publisher _reset_pub;
        ros::Publisher _cv_pub;
        ros::Publisher _fault_pub;
        Vector3d _P;
        Vector3d _Eta;
        Matrix3d _RNed;
        float _yaw_des;

        bool _point_from_kb;
        bool _first_odom;
        bool _landed_state;
        bool _ctrl_active;
        double _take_off_altitude;
        bool _traj_done;
        int _gen_th;
        CARTESIAN_PLANNER *_cp;
        std_msgs::Float32MultiArray _faults;
};

NET_MANAGER::NET_MANAGER() {
    _odom_sub = _nh.subscribe("/lee/pose/ned", 1, &NET_MANAGER::odom_cb, this);
    _land_state_sub = _nh.subscribe("/lee/landed", 1, &NET_MANAGER::land_state_cb, this);
    _point_pub = _nh.advertise< geometry_msgs::Pose >("/lee/goal", 1);
    _reset_pub = _nh.advertise< std_msgs::Bool > ("/lee/sys_reset", 1);
    _controller_active_sub = _nh.subscribe< std_msgs::Bool >("/lee/controller_active", 1, &NET_MANAGER::ctrl_active, this);
    _cv_pub = _nh.advertise< std_msgs::Float64> ("/lee/cruise_velocity", 1);
    _fault_pub = _nh.advertise< std_msgs::Float32MultiArray > ("/lee/faults", 1);

    _faults.data.resize(4); //QuadCopter

    if( !_nh.getParam("point_from_kb", _point_from_kb)) {
        _point_from_kb = true;
    }

    if( !_nh.getParam("take_off_altitude", _take_off_altitude)) {
        _take_off_altitude = -0.6; 
    }

    if( !_nh.getParam("gen_th", _gen_th )) {
        _gen_th = 85;
    }
    
    _landed_state = false;
    _first_odom = false;
    _traj_done = false;

    _cp = new CARTESIAN_PLANNER(100);
}

void NET_MANAGER::land_state_cb( std_msgs::Bool d ) {
    _landed_state = d.data;
}

void NET_MANAGER::ctrl_active( std_msgs::Bool b ) {
    _ctrl_active = b.data;
}


void NET_MANAGER::odom_cb( geometry_msgs::Pose odom ) {
    _P << odom.position.x, odom.position.y, odom.position.z;
    Vector4d q; 
    q << odom.orientation.w, odom.orientation.x, odom.orientation.y, odom.orientation.z;
    _Eta = utilities::R2XYZ( utilities::QuatToMat( q ) );

    _first_odom = true;
}

Vector4d NET_MANAGER::get_point_k( ) {
    cout << "Insert x, y, z e yaw in ENU fixed frame" << endl;
    float x,y,z,yaw;
    scanf( "%f%f%f%f", &x, &y, &z, &yaw );

    Vector3d v;
    v << x, y, z;
    
    v = _RNed*v; 
    Vector4d p;
    p(0) = v(0);
    p(1) = v(1);
    p(2) = v(2); 
    p(4) = yaw;

    return p;
}

void NET_MANAGER::takeoff() {


    geometry_msgs::Pose p;
    p.position.x = _P(0);
    p.position.y = _P(1);
    p.position.z = _P(2) + _take_off_altitude; 


    Vector4d q = utilities::RpyToQuat( Vector3d(0, 0, _Eta(2) ) );

    p.orientation.w = q(0);
    p.orientation.x = q(1);
    p.orientation.y = q(2);
    p.orientation.z = q(3);

    _point_pub.publish( p );
    ros::Rate r(10);
    bool reached = false;
    while( !reached ) { 
        Vector3d eu = utilities::quatToRpy( Vector4d( p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z ) );
        reached = ( (_P - Vector3d( p.position.x, p.position.y, p.position.z ) ).norm()  < 0.05 ) && ( fabs(_Eta(2) - eu(2)) < 0.05 ) ; 
        r.sleep();
    }
}

void NET_MANAGER::gen_fault() {

    _traj_done = false;

    const int gen_fault_num_range_from  = 0;
    const int gen_fault_num_range_to    = 500;
    std::random_device                  gen_fault_rand_dev;
    std::mt19937                        gen_fault_generator(gen_fault_rand_dev());
    std::uniform_int_distribution<int>  gen_fault_distr(gen_fault_num_range_from, gen_fault_num_range_to);


    const float perc_damage_from  = 0.1; // 10%
    const float perc_damage_to    = 0.9;  //90%  
    std::random_device                  perc_rand_dev;
    std::mt19937                        perc_generator(perc_rand_dev());
    std::uniform_real_distribution<float>  perc_distr(perc_damage_from, perc_damage_to);
 
    bool gen = false;

    sleep(1);
    while( !gen && !_traj_done ) {

        int gdata = gen_fault_distr( gen_fault_generator );
        //cout << "gdata: " << gdata << " : " << _gen_th << endl;
        gen = ( gdata > _gen_th ); 
        //cout << "gen: " << gen << endl;
        sleep(1);
    }

    if ( !_traj_done ) {
        ROS_WARN("GEN Fault!");
        ROS_INFO("Fault generation");
    }
    else {

        //Only on motor 0 right now
        float perc_damage = perc_distr( perc_generator );
        
        std_msgs::Float32MultiArray d;
        d.data.resize(4);
        d.data[0] = 0.0;
        d.data[1] = 0.0;
        d.data[2] = 0.0;
        d.data[3] = 0.0;

        d.data[0] = perc_damage;    
        _fault_pub.publish( d );
        
        return;
    }
}


void NET_MANAGER::move_to( Vector4d wp, double cv ) {

    cout << "New wp: " << wp.transpose() << " - CV: " << cv << endl;

    std::vector<geometry_msgs::PoseStamped> poses;
    std::vector<double> times;
    
    geometry_msgs::PoseStamped p0;
    double t0;

    //--Start point
    p0.pose.position.x = _P(0);
    p0.pose.position.y = _P(1);
    p0.pose.position.z = _P(2);

    Vector4d q = utilities::RpyToQuat( Vector3d(0, 0, _Eta(2) ) );

    p0.pose.orientation.w = q(0);
    p0.pose.orientation.x = q(1);
    p0.pose.orientation.y = q(2);
    p0.pose.orientation.z = q(3);

    t0 = 0.0;
    poses.push_back( p0 );
    times.push_back( t0 );
    //---
    
    //---End point
    p0.pose.position.x = wp(0);
    p0.pose.position.y = wp(1);
    p0.pose.position.z = wp(2);

    q = utilities::RpyToQuat( Vector3d(0, 0, wp(3) ) );

    p0.pose.orientation.w = q(0);
    p0.pose.orientation.x = q(1);
    p0.pose.orientation.y = q(2);
    p0.pose.orientation.z = q(3);
    
    double s = ( _P - Vector3d( wp(0), wp(1), wp(2)) ).norm();
    
    //t0 = s / cv;
    t0 = s / cv;
    poses.push_back( p0 );
    times.push_back( t0 );
    //---

    _cp->set_waypoints( poses, times );
    _cp->compute();

    geometry_msgs::PoseStamped x;
    geometry_msgs::TwistStamped xd;
    geometry_msgs::AccelStamped xdd;

    ros::Rate r(100);
    while( _cp->getNext(x, xd, xdd) ) {
        //cout << "P: " << x << endl; 
        _point_pub.publish( x.pose );
        _fault_pub.publish( _faults );
        r.sleep();
    }



}

void NET_MANAGER::net_loop() {

    while( !_first_odom ) usleep(0.1*1e6);

    geometry_msgs::Pose dp;

    ros::Rate r(10);

    /*---Random data for an epoch
        *  
        *  number of waypoints: 1 ~ 4
        *  positions: 
        *          x: -10 ~ 10
        *          y: -10 ~ 10
        *          z: -0.5 ~ -7.5
        *          yaw: 0 ~ 2pi
        *
        *  cruse velocity for each wp: 0.2 ~ 1.0
        *  Time for fault: 1.0s ~ 5.0s
        *
    */

    const int wp_num_range_from  = 1;
    const int wp_num_range_to    = 4;
    std::random_device                  wp_rand_dev;
    std::mt19937                        wp_generator(wp_rand_dev());
    std::uniform_int_distribution<int>  wp_distr(wp_num_range_from, wp_num_range_to);
    //---
    const float xy_range_from  = -10.0;
    const float xy_range_to    =  10.0;
    std::random_device                  xy_rand_dev;
    std::mt19937                        xy_generator(xy_rand_dev());
    std::uniform_real_distribution<float>  xy_distr(xy_range_from, xy_range_to);
    //---
    const float z_range_from  = -0.5;
    const float z_range_to    = -12.0;
    std::random_device                  z_rand_dev;
    std::mt19937                        z_generator(z_rand_dev());
    std::uniform_real_distribution<float>  z_distr(z_range_from, z_range_to);
    //---
    const float yaw_range_from  = 0.0;
    const float yaw_range_to    = 2*M_PI;
    std::random_device                  yaw_rand_dev;
    std::mt19937                        yaw_generator(yaw_rand_dev());
    std::uniform_real_distribution<float>  yaw_distr(yaw_range_from, yaw_range_to);
    //---   
    const float cv_range_from  = 0.2;
    const float cv_range_to    = 1.0;
    std::random_device                  cv_rand_dev;
    std::mt19937                        cv_generator(cv_rand_dev());
    std::uniform_real_distribution<float>  cv_distr(cv_range_from, cv_range_to);
    //---
  
    //Session 0: no fault
    //Session 1: fault
    int session = 1; 
    vector < Vector4d > wps;
    vector < double > cvs;

    while(ros::ok()) {

        while (!_ctrl_active ) usleep(0.1*1e6);
        _ctrl_active = false;

        wps.clear();
        int n_wp = wp_distr(wp_generator);
        wps.resize ( n_wp );
        cvs.resize ( n_wp );
        cout << "Path: " << n_wp << " segments" << endl;

        for(int i=0; i<n_wp; i++ ) {
            float x = xy_distr(xy_generator);
            float y = xy_distr(xy_generator);
            float z = z_distr(z_generator);
            float yaw = 0.0; //yaw_distr ( yaw_generator );
            float cv = cv_distr( cv_generator );
            wps[i] << x, y, z, yaw;
            cvs[i] = cv;
        }

        if( session == 0 ) {
            ROS_INFO("Start no-fault session");
            if( _landed_state ) {
                takeoff();
            }
            for(int i=0; i<wps.size(); i++ ) {
                move_to( wps[i], cvs[i] );
            }
        }
        
        else {

            ROS_INFO("Start fault session");
            if( _landed_state ) {
                takeoff();
            }
            for(int i=0; i<wps.size(); i++ ) {
                boost::thread gen_fault_t( &NET_MANAGER::gen_fault, this );
                move_to( wps[i], cvs[i] );
                _traj_done = true;
                usleep(0.1*1e6);
            }
        }

        _landed_state = true;
        std_msgs::Bool reset;
        reset.data = true;
        _reset_pub.publish( reset );
        sleep(1);
    }
}


void NET_MANAGER::test_fault() {

    std_msgs::Float32MultiArray d;
    d.data.resize(4);
    d.data[0] = 0.0;
    d.data[1] = 0.0;
    d.data[2] = 0.0;
    d.data[3] = 0.0;

    ros::Rate r(10);

    float f1;
    int m;
    cout << "Insert motor fault and fault value" << endl;
    scanf("%d%f", &m, &f1);

    d.data[m] = f1;    
    if( f1 > 1.0 ) f1 = 1.0;
    while(ros::ok() ) {
        _fault_pub.publish( d );
        r.sleep();
    }
}

void NET_MANAGER::run(){
    //boost::thread test_faults_t( &NET_MANAGER::test_fault, this);
    boost::thread net_loop_t ( &NET_MANAGER::net_loop, this );
    ros::spin();
}

int main(int argc,char** argv){
    ros::init(argc, argv, "net_man");
    
    NET_MANAGER nm;
    nm.run();

    return 0;
}

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

    private:
        ros::NodeHandle _nh;
        ros::Subscriber _odom_sub;
        ros::Subscriber _land_state_sub;
        ros::Subscriber _controller_active_sub;
        ros::Publisher _point_pub;
        ros::Publisher _reset_pub;
        Vector3d _P;
        Vector3d _Eta;
        Matrix3d _RNed;
        float _yaw_des;

        bool _point_from_kb;
        bool _first_odom;
        bool _landed_state;
        bool _ctrl_active;
        double _take_off_altitude;

};

NET_MANAGER::NET_MANAGER() {
    _odom_sub = _nh.subscribe("/lee/pose/ned", 1, &NET_MANAGER::odom_cb, this);
    _land_state_sub = _nh.subscribe("/lee/landed", 1, &NET_MANAGER::land_state_cb, this);
    _point_pub = _nh.advertise< geometry_msgs::Pose >("/lee/goal", 1);
    _reset_pub = _nh.advertise< std_msgs::Bool > ("/lee/sys_reset", 1);
    _controller_active_sub = _nh.subscribe< std_msgs::Bool >("/lee/controller_active", 1, &NET_MANAGER::ctrl_active, this);



    if( !_nh.getParam("point_from_kb", _point_from_kb)) {
        _point_from_kb = true;
    }

    if( !_nh.getParam("take_off_altitude", _take_off_altitude)) {
        _take_off_altitude = -0.6; 
    }

    
    _landed_state = false;
    _first_odom = false;
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


    //return p;
}


void NET_MANAGER::move_to( Vector4d wp, double cv ) {
    
    geometry_msgs::Pose p;
    p.position.x = wp(0);
    p.position.y = wp(1);
    p.position.z = wp(2); 


    Vector4d q = utilities::RpyToQuat( Vector3d(0, 0, wp(3) ) );

    p.orientation.w = q(0);
    p.orientation.x = q(1);
    p.orientation.y = q(2);
    p.orientation.z = q(3);


    _point_pub.publish( p );
    ros::Rate r(10);
    bool reached = false;
    while( !reached ) { 
        Vector3d eu = utilities::quatToRpy( Vector4d( p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z ) );
        reached = ( (_P - Vector3d( p.position.x, p.position.y, p.position.z ) ).norm()  < 0.05 ) && ( fabs(-_Eta(2) - eu(2)) < 0.05 ) ; 

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
    //--- Danage time: a probabilistic value: 0 -> 100. If > 60 and starting time > 7 sec -> damage it
    //const float tff_range_from  = 2.0;
    //const float tff_range_to    = 15.0;
    //std::random_device                  tff_rand_dev;
    //std::mt19937                        tff_generator(tff_rand_dev());
    //std::uniform_real_distribution<float>  tff_distr(tff_range_from, tff_range_to);
    //---
    const float perc_damage_from  = 0.05; // 5%
    const float perc_damage_to    = 0.9;  //90%  
    std::random_device                  perc_rand_dev;
    std::mt19937                        perc_generator(perc_rand_dev());
    std::uniform_real_distribution<float>  perc_distr(perc_damage_from, perc_damage_to);
 

    //Session 0: no fault
    //Session 1: fault
    int session = 1; 
    vector < Vector4d > wps;
    vector < double > cvs;
    float perc_damage;

    while(ros::ok()) {

        while (!_ctrl_active ) usleep(0.1*1e6);
        _ctrl_active = false;

        wps.clear();
        int n_wp = wp_distr(wp_generator);
        n_wp = 1;
        wps.resize ( n_wp );
        cvs.resize ( n_wp );

        cout << "wp size: "  << wps.size() << endl;

        for(int i=0; i<n_wp; i++ ) {
            float x = 0.0; //xy_distr(xy_generator);
            float y = 0.0; //xy_distr(xy_generator);
            float z = _P(2) + _take_off_altitude;  //0.0; //z_distr(z_generator);
            float yaw = yaw_distr ( yaw_generator );
            float cv = cv_distr( cv_generator );
            wps[i] << x, y, z, yaw;
            cvs[i] = cv;
        }

        if( session == 0 ) {

            ROS_INFO("Start no-fault session");

        }
        else {

            ROS_INFO("Start fault session");
            perc_damage = perc_distr( perc_generator );
            if( _landed_state ) {
                takeoff();
            }
            for(int i=0; i<wps.size(); i++ ) {
                cout << "Wp: " << wps[i].transpose() << endl;
                move_to( wps[i], 0 );
            }
        }

        _landed_state = true;
        std_msgs::Bool reset;
        reset.data = true;
        _reset_pub.publish( reset );
        sleep(1);
    }
}


void NET_MANAGER::run(){
    boost::thread net_loop_t ( &NET_MANAGER::net_loop, this );
    ros::spin();
}

int main(int argc,char** argv){
    ros::init(argc, argv, "net_man");
    
    NET_MANAGER nm;
    nm.run();

    return 0;
}

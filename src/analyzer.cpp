//This code is to evaluate the NNFault detector

#include "ros/ros.h"
#include "lee_controller/fault.h"
#include "std_msgs/Float32MultiArray.h"
#include "boost/thread.hpp"

using namespace std;

class NNanalyzer {

    public:
        NNanalyzer();
        void run();
        void fault_cb( lee_controller::fault );
        void damage_cb( std_msgs::Float32MultiArray );
        void analyze();
        void sys_reset( std_msgs::Bool b );
    private:

        ros::NodeHandle _nh;
        ros::Subscriber _fault_detector_sub;
        ros::Subscriber _damage_sub;
        ros::Subscriber _system_reset_req;

        bool _real_motor_fault_happened;
        bool _detected_motor_fault_happened;
        int _real_motor_fault;
        int _detected_motor_fault;

        bool _new_session;

};

NNanalyzer::NNanalyzer() {
    _fault_detector_sub = _nh.subscribe("/motor_fault", 1, &NNanalyzer::fault_cb, this);
    _damage_sub = _nh.subscribe("/lee/faults", 1, &NNanalyzer::damage_cb, this );
    _system_reset_req = _nh.subscribe("/lee/sys_reset", 1, &NNanalyzer::sys_reset, this);


    _new_session = false;

    _real_motor_fault_happened = false;
    _detected_motor_fault_happened = false;

}

void NNanalyzer::sys_reset( std_msgs::Bool res ) {


    _new_session = res.data;

}

void NNanalyzer::damage_cb( std_msgs::Float32MultiArray f ) {
    
    
    _real_motor_fault = -1;
    
    bool fault = false;
    for(int i=0; i<f.data.size(); i++ ) {
        if ( f.data[i] > 0.0 ) {
             _real_motor_fault_happened = true;
             //cout << "_real_motor_fault_happened: " << _real_motor_fault_happened << endl;
            _real_motor_fault = i;
            fault = true;
        }
    }

    if( !fault ) _real_motor_fault_happened = false;

}


void NNanalyzer::fault_cb( lee_controller::fault f) {

    if( f.fault.data == true ) {
        _detected_motor_fault_happened = true;
        _detected_motor_fault = f.m.data;
        //cout << "_detected_motor_fault_happened: " << _detected_motor_fault_happened << endl; 
    }
    //else _detected_motor_fault_happened = false;

}


void NNanalyzer::analyze() {

    ros::Rate r(10);
    while(ros::ok()) {

        if( _real_motor_fault_happened ) {
            if( _detected_motor_fault_happened ) {
                cout << "Fault accadato e detectato" << endl;
                while( !_new_session ) {
                    usleep(0.1*1e6);
                    _detected_motor_fault_happened = false;
                    _real_motor_fault_happened = false;
                }
                _new_session = false;
            }
        }
        r.sleep();
    }
}

void NNanalyzer::run() {
    boost::thread analyze_t( &NNanalyzer::analyze, this );
    ros::spin();
}




int main( int argc, char** argv ) {

    ros::init(argc, argv, "nn_analyzer");
    
    NNanalyzer nanal;
    nanal.run();

    return 0;
}


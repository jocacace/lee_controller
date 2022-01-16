#!/usr/bin/env python3

import functools
import numpy as np
import tensorflow as tf
import pandas as pd
from tensorflow import keras
from tensorflow.keras import layers
import rospy
import rospkg
import time

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import numpy as np
import tensorflow as tf
from lee_controller.msg import fault
from geometry_msgs.msg import Wrench

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('lee_controller')

class detector():
    def __init__(self):
        rospy.init_node('fault_detector_nn')
        print ("Detector class")
        rospy.Subscriber("/lee/ext_wrench", Wrench, self.ext_wrench_cb)
        self.fault_pub = rospy.Publisher('/motor_fault', fault, queue_size=1)
        self.ext_wrench = Wrench()
        self.ext_wrench_ready = False
        self.fault_data = fault()
        self.window = 10

    def ext_wrench_cb(self, msg):
        self.ext_wrench = msg 
        self.ext_wrench_ready = True

    def run(self):
        rate = rospy.Rate(100) # 10hz

        model = tf.keras.models.load_model( pkg_path + "/NN/TS/model/netQuadcHumm.model") #Rete neurale rottura motori

        m1 = 0
        m2 = 0
        m3 = 0
        m4 = 0

        fault_itr_m0 = 0
        fault_itr_m1 = 0
        fault_itr_m2 = 0
        fault_itr_m3 = 0

        new_fault = False

        while not rospy.is_shutdown():
            
            if( self.ext_wrench_ready == True):
                input_value_raw = np.array([ self.ext_wrench.force.x, self.ext_wrench.force.y, self.ext_wrench.force.z, 
                self.ext_wrench.torque.x, self.ext_wrench.torque.y, self.ext_wrench.torque.z   ])

                input_value = input_value_raw.reshape((1,1,6))
                prediction = model.predict(input_value)
               
                if( prediction[0][0][0] < 0.8 ):
                    m1 = 0
                else:
                    m1 = 1

                if( prediction[0][0][1] < 0.8 ):
                    m2 = 0
                else:
                    m2 = 1

                if( prediction[0][0][2] < 0.8 ):
                    m3 = 0
                else:
                    m3 = 1

                if( prediction[0][0][3] < 0.8 ):
                    m4 = 0
                else:
                    m4 = 1

                print ( prediction[0][0][0] , " ", prediction[0][0][1] , " ", prediction[0][0][2] , " ", prediction[0][0][3] )
                
                if m1 > 0:
                    fault_itr_m0 = fault_itr_m0+1
                else:
                    fault_itr_m0 = 0

                if m2 > 0:
                    fault_itr_m1 = fault_itr_m1+1
                else:
                    fault_itr_m1 = 0

                if m3 > 0:
                    fault_itr_m2 = fault_itr_m2+1
                else:
                    fault_itr_m2 = 0

                if m4 > 0:
                    fault_itr_m3 = fault_itr_m3+1
                else:
                    fault_itr_m3 = 0

                if fault_itr_m0 > self.window or fault_itr_m1 > self.window or fault_itr_m2 > self.window or fault_itr_m3 > self.window:
                    self.fault_data.fault.data = True                    
                else: 
                    self.fault_data.fault.data = False                    
                    self.fault_data.m.data = -1
                    
                if self.fault_data.fault.data == True:
                    if fault_itr_m0 > self.window:
                        self.fault_data.m.data = 0
                    if fault_itr_m1 > self.window:
                        self.fault_data.m.data = 1
                    if fault_itr_m2 > self.window:
                        self.fault_data.m.data = 2
                    if fault_itr_m3 > self.window:
                        self.fault_data.m.data = 3

                self.fault_pub.publish( self.fault_data )

            rate.sleep()

if __name__ == '__main__':
    try:
        d = detector()
        d.run()
    except rospy.ROSInterruptException:
        pass


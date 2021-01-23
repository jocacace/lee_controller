#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import numpy as np
import tensorflow as tf

from geometry_msgs.msg import Wrench

class detector():
    def __init__(self):
        rospy.init_node('fault_detector_nn')
        print ("Detector class")
        rospy.Subscriber("/lee/ext_wrench", Wrench, self.ext_wrench_cb)
        self.ext_wrench = Wrench()
        self.ext_wrench_ready = False

    def ext_wrench_cb(self, msg):
        self.ext_wrench = msg 
        self.ext_wrench_ready = True

    def run(self):
        rate = rospy.Rate(100) # 10hz

        model = tf.keras.models.load_model("/home/jcacace/Neural_net_v5.model") #Rete neurale rottura motori

        while not rospy.is_shutdown():
            
            if( self.ext_wrench_ready == True) :
                input_value_raw = np.array([ self.ext_wrench.force.x, self.ext_wrench.force.y, self.ext_wrench.force.z, 
                self.ext_wrench.torque.x, self.ext_wrench.torque.y, self.ext_wrench.torque.z   ])

                input_value = input_value_raw.reshape((1,1,6))
                prediction = model.predict(input_value)
                print("prediction: ", prediction)

            rate.sleep()

if __name__ == '__main__':
    try:
        d = detector()
        d.run()
    except rospy.ROSInterruptException:
        pass

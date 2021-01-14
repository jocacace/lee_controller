#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import numpy as np
import tensorflow as tf




def callback(stima):
    global value_0
    global value_1
    global value_2
    global value_3
    global value_4
    global value_5
    value_0=stima.data[0]
    value_1=stima.data[1]
    value_2=stima.data[2]
    value_3=stima.data[3]
    value_4=stima.data[4]
    value_5=stima.data[5]
    return value_0,value_1,value_2,value_3,value_4,value_5
   

def talker():
    rospy.init_node('talker', anonymous=True)
    rospy.Subscriber("/stima", Float32MultiArray, callback)
    
    pub = rospy.Publisher('/neural_net', Float32, queue_size=1)
    
    #model=tf.keras.models.load_model("Neural_net_v4.model") #Rete neurale 
    model=tf.keras.models.load_model("Neural_net_v5.model") #Rete neurale rottura motori
    rate = rospy.Rate(100) # 100hz
    while not rospy.is_shutdown():
        input_value_temp=np.array([value_0,value_1,value_2,value_3,value_4,value_5])
        input_value=input_value_temp.reshape((1,1,6))
        prediction=model.predict(input_value)
        if prediction>=1.1 and prediction<2.5:
            prediction=2
        elif prediction>=2.5:
            prediction=4
        else:
            prediction=0    
        print(prediction)
        pub.publish(prediction)
        rate.sleep()
        
        

if __name__ == '__main__':
    try:
        talker()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

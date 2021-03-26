#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import MinMaxScaler
import time
import math

# For LSTM model
from keras.models import Sequential
from keras.layers import Dense
from keras.layers import LSTM
from keras.layers import Dropout

#from keras import InputLayer
from keras.callbacks import EarlyStopping
from keras.models import load_model

import functools
import numpy as np
import tensorflow as tf
import pandas as pd
from tensorflow import keras
from tensorflow.keras import layers
import rospy
import rospkg
import time
import random

DEFINE_BLOCKS = False
MOTORS = 4

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('lee_controller')
ts_path = pkg_path + "/NN/TS/humm_val.txt"
validation_path = pkg_path + "/NN/TS/humm_val.txt"

count = 0
ts_file = open(ts_path)
ds_file = open(validation_path)

ts_Lines = ts_file.readlines() 
ds_Lines = ds_file.readlines()

ts_traj = 0
for l in ts_Lines: 
    if('====' in l): 
        file = open("/tmp/traj_" + str(ts_traj) + ".txt", "w") 
        ts_traj = ts_traj + 1    
    else:   
        file.write(l)      

print("Created: " + str(ts_traj) + " files from training set")


ds_traj = 0
for l in ds_Lines: 
    if('====' in l): 
        file = open("/tmp/test_" + str(ds_traj) + ".txt", "w") 
        ds_traj = ds_traj + 1    
    else:   
        file.write(l)      

print("Created: " + str(ds_traj) + " files from test set")

hid_layer_dim = 250
window = 1

Xtot = []
Ytot = []

##Param
Th = 0.6

if DEFINE_BLOCKS == False:
    X = []
    Y = []
    for i in range(0,ts_traj):
    #for i in range(0,1):
        file = '/tmp/traj_' + str(i) + ".txt"
        train = pd.read_csv(file, sep=',')

        Xt = train[['Fx','Fy','Fz','Mx','My','Mz']].values

        #print(Xt)
        if MOTORS == 1:
            Yt = train[['m1']].values
        else:
            Yt = train[['m1', 'm2', 'm3', 'm4']].values
        
        for i in range(window,len(Xt)):
            X.append(Xt[i-window:i,:])
            Y.append(Yt[i])

        #X, Y = np.array(X), np.array(Y)
        #Xtot.append(X)
        #Ytot.append(Y)
    print(len(X))
else:
    print("Else")

    
#model = tf.keras.models.load_model("/home/jcacace/Neural_net_v5.model") #Rete neurale rottura motori
#model = tf.keras.models.load_model(pkg_path + "/NN/TS/model/netQuadcross.model")
model = tf.keras.models.load_model(pkg_path + "/NN/TS/model/netQuadplus.model")

tot = 0
wrong = 0
m1 = 0
m2 = 0
m3 = 0
m4 = 0

for i in range(0, len(X) ):
    input_value = X[i][0].reshape((1,1,6))
    prediction = model.predict( input_value )
    
    if( prediction[0][0][0] < Th ):
        m1 = 0
    else:
        m1 = 1

    if( prediction[0][0][1] < Th ):
        m2 = 0
    else:
        m2 = 1

    if( prediction[0][0][2] < Th ):
        m3 = 0
    else:
        m3 = 1

    if( prediction[0][0][3] < Th ):
        m4 = 0
    else:
        m4 = 1

    tot = tot + 1
    check = ( m1 -  Y[i][0]) + (m2 -  Y[i][1]) + (m3 -  Y[i][2]) + (m4 -  Y[i][3])
    
    if( math.fabs( check ) > 0 ):
        print("prediction: ", prediction, " real: ", Y[i][0], " ", Y[i][1], " ", Y[i][2], " ", Y[i][3])
        wrong = wrong +1
    

    '''
    if( prediction[0][0] < 0.8 ):
        m1 = 0
    else:
        m1 = 1

    if( prediction[0][1] < 0.8 ):
        m2 = 0
    else:
        m2 = 1

    if( prediction[0][2] < 0.8 ):
        m3 = 0
    else:
        m3 = 1

    if( prediction[0][3] < 0.8 ):
        m4 = 0
    else:
        m4 = 1
    '''

    tot = tot + 1
    check = ( m1 -  Y[i][0]) + (m2 -  Y[i][1]) + (m3 -  Y[i][2]) + (m4 -  Y[i][3])
    
    if( math.fabs( check ) > 0 ):
        print("prediction: ", prediction, " real: ", Y[i][0], " ", Y[i][1], " ", Y[i][2], " ", Y[i][3])
        wrong = wrong +1
print ("Tot: ", tot, " Wrong: ", wrong )

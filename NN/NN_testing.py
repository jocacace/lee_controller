#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import MinMaxScaler
import time
from keras.utils.vis_utils import plot_model
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

MOTORS = 4

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('lee_controller')
ts_path = pkg_path + "/NN/TS/humm_ts.txt"
validation_path = pkg_path + "/NN/TS/humm_test.txt"

#ts_file: training set
#ds_file: dataset for testing

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

model = keras.Sequential()
model = tf.keras.models.load_model( pkg_path + "/NN/model/net1.model") #Rete neurale rottura motori
model.summary()

plot_model(model, to_file=pkg_path + "/NN/model_plot.png", show_shapes=True, show_layer_names=True)

Xtot = []
Ytot = []
for i in range(0,ts_traj):
#for i in range(0,1):
    file = '/tmp/traj_' + str(i) + ".txt"
    train = pd.read_csv(file, sep=',')

    Xt = train[['Fx','Fy','Fz','Mx','My','Mz']].values
    #Yt = train[['m1', 'm2', 'm3', 'm4']].values
    Yt = train[['m1']].values

    X = []
    Y = []
    for i in range(window,len(Xt)):
        X.append(Xt[i-window:i,:])
        Y.append(Yt[i])

    X, Y = np.array(X), np.array(Y)
    Xtot.append(X)
    Ytot.append(Y)

Xdstot = []
Ydstot = []

for i in range(0,ds_traj):
    file = '/tmp/test_' + str(i) + ".txt"
    test = pd.read_csv(file, sep=',')

    Xdt = test[['Fx','Fy','Fz','Mx','My','Mz']].values
    #Ydt = test[['m1', 'm2', 'm3', 'm4']].values
    Ydt = test[['m1']].values

    X = []
    Y = []
    for i in range(window,len(Xdt)):
        X.append(Xdt[i-window:i,:])
        Y.append(Ydt[i])

    X, Y = np.array(X), np.array(Y)
    Xdstot.append(X)
    Ydstot.append(Y)

#for i in range(0, len(Xdstot)):
#    for j in range(0, len(Xdstot[i])):
#        X = []
#        for k in range (0, window):
#            X.append( Xdstot[i][j][k] )
#
#        X = np.array(X)
#        print (model.predict( X ))
#        time.sleep(1)
#        print ("DONE")

print(len(Xdstot[0][0]))
for i in range(0, len(Xdstot)):
    for j in range(0, len(Xdstot[i])):
        #for k in range(0, len(Xdstot[i][j])):
        model.fit( np.array(Xdstot[i][j]) )

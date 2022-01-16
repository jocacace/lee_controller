#!/usr/bin/env python3


import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import MinMaxScaler
import time
from pandas import Series

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

DEFINE_BLOCKS = True
MOTORS = 4

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('lee_controller')
ts_path = pkg_path + "/NN/TS/humm_ts_2.txt"
validation_path = pkg_path + "/NN/TS/humm_ts_2.txt"

count = 0
ts_file = open(ts_path)
ds_file = open(validation_path)

ts_Lines = ts_file.readlines() 
ds_Lines = ds_file.readlines()

#Generating temps file from the Trainingset
ts_traj = 0
for l in ts_Lines: 
    if('====' in l):
        #time.sleep(3) 
        file = open("/tmp/traj_" + str(ts_traj) + ".txt", "w") 
        ts_traj = ts_traj + 1    
    else:   
        file.write(l)      

#Generating temps file from the Testset
ds_traj = 0
for l in ds_Lines: 
    if('====' in l): 
        file = open("/tmp/test_" + str(ds_traj) + ".txt", "w") 
        ds_traj = ds_traj + 1    
    else:   
        file.write(l)      


print("Created: " + str(ts_traj) + " files from training set")
print("Created: " + str(ds_traj) + " files from test set")



hid_layer_dim = 250
window = 1

Xtot = []
Ytot = []

#Create one big trajectory considering training set raws
for i in range(0,ts_traj):
    file = '/tmp/traj_' + str(i) + ".txt"
    train = pd.read_csv(file, sep=',')

    Xt = train[['Fx','Fy','Fz','Mx','My','Mz']].values
    Yt = train[['m1', 'm2', 'm3', 'm4']].values
    
    #print("LEEEN: ", len(Xt))
    X = []
    Y = []
    for i in range(window,len(Xt)):
        X.append( [ Xt[i][0], Xt[i][1], Xt[i][2], Xt[i][3], Xt[i][4], Xt[i][5]  ] )
        Y.append( [ Yt[i][0], Yt[i][1], Yt[i][2], Yt[i][3] ] )
        #X.append(Xt[i-window:i+1,:])
        #Y.append(Yt[i])

    X, Y = np.array(X), np.array(Y)
    
    Xtot.append(X)
    Ytot.append(Y)

Xdstot = []
Ydstot = []

#Create one big trajectory considering training set raws
for i in range(0, ds_traj):
    file = '/tmp/test_' + str(i) + ".txt"
    test = pd.read_csv(file, sep=',')

    Xdst = test[['Fx','Fy','Fz','Mx','My','Mz']].values
    Ydst = test[['m1', 'm2', 'm3', 'm4']].values
    
    #print("LEEEN: ", len(Xt))
    X = []
    Y = []
    for i in range(window,len(Xdst)):
        X.append( [ Xdst[i][0], Xdst[i][1], Xdst[i][2], Xdst[i][3], Xdst[i][4], Xdst[i][5]  ] )
        Y.append( [ Ydst[i][0], Ydst[i][1], Ydst[i][2], Ydst[i][3] ] )
        #X.append(Xt[i-window:i+1,:])
        #Y.append(Yt[i])

    X, Y = np.array(X), np.array(Y)
    
    Xdstot.append(X)
    Ydstot.append(Y)


model = keras.Sequential()


#LSTM
##model.add(LSTM(50, batch_input_shape=(1, None, 6), return_sequences=True , stateful=True, recurrent_dropout=0.1))
##model.add(LSTM(50, recurrent_dropout=0.1, stateful=True))
##model.add(Dense(4, activation='softmax'))

##Feedforward
model.add(Dense(50,  kernel_initializer="uniform", activation="relu", input_shape=(1, 6)))
model.add(Dense(50, activation="relu", kernel_initializer="uniform"))
model.add(Dense(4, activation='softmax'))

model.compile(loss='mean_squared_error', optimizer='adam',metrics=['categorical_accuracy'])


for i in range(0, len(Xtot)): 

    #Take a random sample from the validation set
    val_index = random.randint(0,len(Xdstot)-1)
    
    #Convert Training data for the fit function
    xData = np.array(Xtot[i])
    num_rows_ts = len(Xtot[i])
    xData = Xtot[i].reshape((num_rows_ts,1,6))
    yData = Ytot[i].reshape((num_rows_ts,1,MOTORS))

    #Convert Validation data for the fit function
    xdsData = np.array(Xdstot[val_index])
    num_rows_ds = len(Xtot[i])
    #xdsData = Xdstot[i].reshape((num_rows_ds,1,6))
    #ydsData = Ydstot[i].reshape((num_rows_ds,1,MOTORS))

    #Fit
    history = model.fit( xData, yData, epochs=100) #, validation_data=(xdsData, ydsData))

model.save(pkg_path + "/NN/TS/model/netQuadcHumm.model")

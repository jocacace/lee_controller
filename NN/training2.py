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
ts_path = pkg_path + "/NN/TS/humm_ts.txt"
validation_path = pkg_path + "/NN/TS/test.txt"


count = 0
ts_file = open(ts_path)
ds_file = open(validation_path)

ts_Lines = ts_file.readlines() 
ds_Lines = ds_file.readlines()

ts_traj = 0
for l in ts_Lines: 
    if('====' in l):
        #time.sleep(3) 
        file = open("/tmp/traj_" + str(ts_traj) + ".txt", "w") 
        ts_traj = ts_traj + 1    
    else:   
        file.write(l)      
        #print(l)

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

x_data = []
y_data = []

if DEFINE_BLOCKS == False:
    X = []
    Y = []
    for i in range(0,ts_traj):
    #for i in range(0,1):
        file = '/tmp/traj_' + str(i) + ".txt"
        train = pd.read_csv(file, sep=',')

        Xt = train[['Fx','Fy','Fz','Mx','My','Mz']].values

        #print(Xt)
        #Yt = train[['m1', 'm2', 'm3', 'm4']].values
        Yt = train[['m1']].values

        for i in range(window,len(Xt)):
            X.append(Xt[i-window:i,:])
            Y.append(Yt[i])

        #X, Y = np.array(X), np.array(Y)
        #Xtot.append(X)
        #Ytot.append(Y)
    print(len(X))


    X = np.array(X)
    Y = np.array(Y)

    Xtot = X
    Ytot = Y
else: 
    if MOTORS == 1:
        for i in range(0,ts_traj):
            file = '/tmp/traj_' + str(i) + ".txt"
            train = pd.read_csv(file, sep=',')

            Xt = train[['Fx','Fy','Fz','Mx','My','Mz']].values
            #Yt = train[['m1', 'm2', 'm3', 'm4']].values
            Yt = train[['m1']].values

            X = []
            Y = []
            #for i in range(window,len(Xt)):
            #    X.append(Xt[i-window:i,:])
            #    Y.append(Yt[i])
            for i in range(0,len(Xt)):
                X.append(Xt[i])
                Y.append(Yt[i])

            #print ("Lunghezza: ", len(X))
            X, Y = np.array(X), np.array(Y)
            Xtot.append(X)
            Ytot.append(Y)
    else:
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
            #for i in range(0,len(Xt)):
            #    X.append(Xt[i])
            #    Y.append(Yt[i])

            #print ("Lunghezza: ", len(X))
            #series = Series(X[0])
            X, Y = np.array(X), np.array(Y)

           
            Xtot.append(X)
            Ytot.append(Y)

#print( X.shape )
#num_rows_ts, num_cols_ts = X.shape
#X = X.reshape((num_rows_ts,1,num_cols_ts))


model = keras.Sequential()
#model.add(layers.LSTM(200, input_shape=(1, 6 ) ) )
#if MOTORS == 1:
#    model.add(layers.Dense(1))
#else:
#    model.add(layers.Dense(4))

hid_layer_dim = 50
#model.add(Dense(units=50, return_sequences=True, input_shape=(1, 6)))
model.add(Dense(768, input_dim=3072, kernel_initializer="uniform", activation="relu", input_shape=(1, 6)))
#model.add(Dropout(0.2))
#model.add(LSTM(units=50, return_sequences=True))
#model.add(Dropout(0.2))
#model.add(LSTM(units=50))
#model.add(Dropout(0.2))
model.add(Dense(384, activation="relu", kernel_initializer="uniform"))
model.add(Dense(4, activation='softmax'))
#model.compile(optimizer = 'adam', loss = 'categorical_crossentropy',metrics = ['categorical_accuracy'])
model.compile(
    optimizer='adam', 
    loss='mean_absolute_error', 
    metrics=['mean_absolute_error'])

model.compile(loss='mean_squared_error', optimizer='adam',metrics=['categorical_accuracy'])

if MOTORS == 1:
    for i in range(0, len(Xtot)):
        history = model.fit(Xtot[i][0].reshape((window,1,6)), Ytot[i][0].reshape((window,1,1)), epochs=10) #, validation_data=(test_X, test_Y))
else:
    for i in range(0, len(Xtot)): 

        #print( Xtot[i] )
        xData = np.array(Xtot[i])
        num_rows_ts = len(Xtot[i])
        xData = Xtot[i].reshape((num_rows_ts,1,6))
        yData = Ytot[i].reshape((num_rows_ts,1,4))
        #print( xData )
        #print( yData )
        history = model.fit( xData, yData, epochs=100) #, validation_data=(test_X, test_Y))

        #train_X = Xtot[i].reshape((num_rows_ts,1,6))
        #print (Xtot[i])
        #print(num_rows_ts)
        #print(Xtot[i][1].reshape((window,1,6)))
        #print(Ytot[i][1].reshape((window,1,4)))
        #history = model.fit(Xtot[i][0].reshape((window,1,6)), Ytot[i][0].reshape((window,1,4)), epochs=100) #, validation_data=(test_X, test_Y))

model.save('/home/jcacace/Neural_net_v5.model')

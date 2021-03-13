#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import MinMaxScaler
import time

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

#model = keras.Sequential()
#model.add(LSTM(units=50, return_sequences=True, input_shape=(window, 6)))
##model.add(Dropout(0.2))
#model.add(LSTM(units=50, return_sequences=True))
##model.add(Dropout(0.2))
#model.add(LSTM(units=50))
##model.add(Dropout(0.2))
#model.add(Dense(units=4))
##model.compile(optimizer = 'adam', loss = 'categorical_crossentropy',metrics = ['categorical_accuracy'])
#model.compile(
#    optimizer='adam', 
#    loss='mean_absolute_error', 
#    metrics=['mean_absolute_error'])
# Allow for early exit 
#es = EarlyStopping(monitor='loss',mode='min',verbose=1,patience=10)
model = keras.Sequential()
model.add(layers.LSTM(200, input_shape=(1, 6 ) ) )
model.add(layers.Dense(1))
model.compile(loss='mean_squared_error', optimizer='adam',metrics=['categorical_accuracy'])

#model.add(LSTM(units=hid_layer_dim, batch_input_shape=(window, 1, 6), return_sequences=True, recurrent_dropout=0.1 ) ) 
#model.add(LSTM(units=250, recurrent_dropout=0.1) )
#model.add(Dense(4, activation='softmax'))
#model.compile(loss='categorical_crossentropy', optimizer='adam', metrics =['categorical_accuracy'])
#model.summary()

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



# Fit (and time) LSTM model
#t0 = time.time()

#for i in range(0, len(Xtot)):
i=0
ts_data = random.randint(0, len(Ydstot)-1  )
history = model.fit(Xtot[i], Ytot[i], epochs = 10, batch_size = window, verbose=1)
#history = model.fit(Xtot[i], Ytot[i], epochs = 80, batch_size = 250, callbacks=[es], verbose=1, validation_data=(Xdstot[ ts_data ], Ydstot[ ts_data]))
#t1 = time.time()
#print('Runtime: %.2f s' %(t1-t0))



#for i in range(0, len(Xtot)):
#    ts_data = random.randint(0, len(Ydstot)-1  )
#    history = model.fit(Xtot[i], Ytot[i], epochs=100, verbose=1, validation_data=(Xdstot[ ts_data ], Ydstot[ ts_data]))

model.save( pkg_path + '/NN/model/net1.model')
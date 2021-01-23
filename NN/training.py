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

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('lee_controller')
ts_path = pkg_path + "/NN/TS/ts.txt"
validation_path = pkg_path + "/NN/TS/train.txt"

count = 0
ts_file = open(ts_path)
validation_file = open(validation_path)

ts_Lines = ts_file.readlines() 
first = True
ts_batch = []

n_traj = 0
for l in ts_Lines: 
    if('====' in l and not first): 
        n_traj = n_traj + 1
    elif ( first ):
        first = False
    else:   
        ts_batch.append(l)

print("In the trainign set have been found: ", len(ts_batch ), " samples in ", n_traj, " trajectories")

x_data = []
y_data = []

for b in range(0, len(ts_batch)):
    
    s = ts_batch[b]
    x = s.split(", ")
    x_data.append( [float(x[0]), float(x[1]), float(x[2]), float(x[3]), float(x[4]), float(x[5])] ) 
    y_data.append( [int(x[6])] ) 
   
arr_x = np.array(x_data)
arr_y = np.array(y_data)

num_rows, num_cols = arr_x.shape
train_X = arr_x.reshape((num_rows,1,num_cols))
train_Y = arr_y.reshape((num_rows,1,1))


print("Len train_X: ", len(train_X))



val_batch = []
val_Lines = validation_file.readlines() 
n_traj = 0
first = True
for l in val_Lines: 
    if('====' in l and not first): 
        n_traj = n_traj + 1
    elif ( first ):
        first = False
    else:   
        val_batch.append(l)

print("In the trainign set have been found: ", len(val_batch ), " samples in ", n_traj, " trajectories")

x_data = []
y_data = []

for b in range(0, len(val_batch)):
    
    s = ts_batch[b]
    x = s.split(", ")
    x_data.append( [float(x[0]), float(x[1]), float(x[2]), float(x[3]), float(x[4]), float(x[5])] ) 
    y_data.append( [int(x[6])] ) 
   
arr_x = np.array(x_data)
arr_y = np.array(y_data)

num_rows, num_cols = arr_x.shape
test_X = arr_x.reshape((num_rows,1,num_cols))
test_Y = arr_y.reshape((num_rows,1,1))


print("Len train_X: ", len(test_X))

model = keras.Sequential()
model.add(layers.LSTM(200, input_shape=(1, num_cols)))
model.add(layers.Dense(1))
model.compile(loss='mae', optimizer='adam',metrics=['accuracy'])
history = model.fit(train_X, train_Y, epochs=10, validation_data=(test_X, test_Y))

model.save('/home/jcacace/Neural_net_v5.model')

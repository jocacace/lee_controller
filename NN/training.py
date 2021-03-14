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

MOTORS = 1

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('lee_controller')
ts_path = pkg_path + "/NN/TS/ts.txt"
validation_path = pkg_path + "/NN/TS/test.txt"

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
    if MOTORS == 1:
        y_data.append( int(x[6]) ) 
    else:
        y_data.append( [int(x[6]), int(x[7]), int(x[8]), int(x[9])] ) 
  
   
arr_x = np.array(x_data)
arr_y = np.array(y_data)

num_rows_ts, num_cols_ts = arr_x.shape
train_X = arr_x.reshape((num_rows_ts,1,num_cols_ts))

if MOTORS == 1:
    train_Y = arr_y.reshape((num_rows_ts,1,1))
else:
    train_Y = arr_y.reshape((num_rows_ts,1,4))


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

print("len(val_batch): " , len(val_batch))
for b in range(0, len(val_batch)):
    
    s = val_batch[b]
    x = s.split(", ")
    x_data.append( [float(x[0]), float(x[1]), float(x[2]), float(x[3]), float(x[4]), float(x[5])] ) 
    if MOTORS == 1:
        y_data.append( int(x[6])  ) 
    else:
        y_data.append( [int(x[6]), int(x[7]), int(x[8]), int(x[9])] ) 
   
arr_x = np.array(x_data)
arr_y = np.array(y_data)

num_rows_test, num_cols_test = arr_x.shape
test_X = arr_x.reshape((num_rows_test,1,num_cols_test))
#print( num_rows_ts, " " , num_cols_ts)

if MOTORS == 1:
    test_Y = arr_y.reshape((num_rows_test,1,1))
else:
    test_Y = arr_y.reshape((num_rows_test,1,4))

#test_Y = arr_y.reshape((num_rows,1,4))

'''
print("Len train_X: ", len(test_X))
print("num cols: ", num_cols)
model = keras.Sequential()
model.add(layers.LSTM(200, input_shape=(1, num_cols)))
model.add(layers.Dense(4))
#loss='categorical_crossentropy', optimizer='adam', metrics =['categorical_accuracy']
model.compile(loss='mse', optimizer='adam',metrics=['accuracy'])
history = model.fit(train_X, train_Y, epochs=10, validation_data=(test_X, test_Y))
'''

'''
model = keras.Sequential()
# Add an Embedding layer expecting input vocab of size 1000, and
# output embedding dimension of size 64.
model.add(layers.Embedding(input_dim=1000, output_dim=64))

# Add a LSTM layer with 128 internal units.
model.add(layers.LSTM(128))

# Add a Dense layer with 10 units.
model.add(layers.Dense(10))

model.summary()
'''
'''

model = keras.Sequential()
model.add(layers.LSTM(128, batch_input_shape=( 1, 1, num_cols_ts), return_sequences=True , stateful=True, recurrent_dropout=0.1))
model.add(layers.LSTM(128, recurrent_dropout=0.1, stateful=True))
model.add(layers.Dense(1, activation='softmax'))
model.summary()
model.compile(loss='mean_squared_error', optimizer='adam', metrics =['categorical_accuracy'])
history = model.fit(train_X, train_Y, epochs=10, validation_data=(test_X, test_Y))
model.save('/home/jcacace/Neural_net_v5.model')
'''

model = keras.Sequential()
model.add(layers.LSTM(200, input_shape=(1, num_cols_ts ) ) )
#model.add(layers.LSTM(128, batch_input_shape=(1, 1, num_cols_ts ),  return_sequences=True , stateful=True, recurrent_dropout=0.1)) 
#model.add(layers.LSTM(128, recurrent_dropout=0.1, stateful=True))
if MOTORS == 1:
    model.add(layers.Dense(1))
else:
    model.add(layers.Dense(4))

print(len(train_X) )
#model.compile(loss='mean_squared_error', optimizer='adam',metrics=['categorical_accuracy'])
#history = model.fit(train_X, train_Y, epochs=10, validation_data=(test_X, test_Y))
#model.save('/home/jcacace/Neural_net_v5.model')

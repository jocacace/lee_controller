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



Xtot = []
Ytot = []
ts_data_file = open("/tmp/TS_data.txt", "w") 
ts_target_file = open("/tmp/TS_target.txt", "w") 

X = []
Y = []
for i in range(0,ts_traj):
    file = '/tmp/traj_' + str(i) + ".txt"
    train = pd.read_csv(file, sep=',')

    Xt = train[['Fx','Fy','Fz','Mx','My','Mz']].values
    Yt = train[['m1', 'm2', 'm3', 'm4']].values
    
    ts_data_file.write(str(Xt))
    ts_data_file.write("\n")
    ts_target_file.write(str(Yt))
    ts_target_file.write("\n")
    print(Xt)


Xtot = X
Ytot = Y
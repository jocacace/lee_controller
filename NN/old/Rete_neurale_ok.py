#!/usr/bin/env python3

import functools
import numpy as np
import tensorflow as tf
import pandas as pd
from tensorflow import keras
from tensorflow.keras import layers

########################VERSIONE ROTTURA DEFINITIVA###############
import functools
import numpy as np
import tensorflow as tf
import pandas as pd
from tensorflow import keras
from tensorflow.keras import layers

#DATI NO FAULT
stima1_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/No_fault/stima1.csv')
stima2_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/No_fault/stima2.csv')
stima3_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/No_fault/stima3.csv')
stima4_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/No_fault/stima4.csv')
stima5_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/No_fault/stima5.csv')
stima6_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/No_fault/stima6.csv')
stima7_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/No_fault/stima7.csv')
stima8_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/No_fault/stima8.csv')
stima9_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/No_fault/stima9.csv')
stima10_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/No_fault/stima10.csv')
#DATI FAULT MOTORE 1
fault_stima1_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault_low/fault_low_stima1.csv')
fault_stima2_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault_low/fault_low_stima2.csv')
fault_stima3_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault_low/fault_low_stima3.csv')
fault_stima4_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault_low/fault_low_stima4.csv')
fault_stima5_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault_low/fault_low_stima5.csv')
fault_stima6_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault_low/fault_low_stima6.csv')
fault_stima7_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault_low/fault_low_stima7.csv')
fault_stima8_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault_low/fault_low_stima8.csv')
fault_stima9_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault_low/fault_low_stima9.csv')
fault_stima10_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault_low/fault_low_stima10.csv')
#DATI FAULT MOTORE 2
fault2_stima1_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault2/fault2_stima1.csv')
fault2_stima2_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault2/fault2_stima2.csv')
fault2_stima3_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault2/fault2_stima3.csv')
fault2_stima4_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault2/fault2_stima4.csv')
fault2_stima5_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault2/fault2_stima5.csv')
fault2_stima6_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault2/fault2_stima6.csv')
fault2_stima7_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault2/fault2_stima7.csv')
fault2_stima8_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault2/fault2_stima8.csv')
fault2_stima9_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault2/fault2_stima9.csv')
fault2_stima10_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault2/fault2_stima10.csv')
#DATI FAULT MOTORE 3
fault3_stima1_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault3/fault3_stima1.csv')
fault3_stima2_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault3/fault3_stima2.csv')
fault3_stima3_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault3/fault3_stima3.csv')
fault3_stima4_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault3/fault3_stima4.csv')
fault3_stima5_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault3/fault3_stima5.csv')
fault3_stima6_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault3/fault3_stima6.csv')
fault3_stima7_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault3/fault3_stima7.csv')
fault3_stima8_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault3/fault3_stima8.csv')
fault3_stima9_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault3/fault3_stima9.csv')
fault3_stima10_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault3/fault3_stima10.csv')
#DATI FAULT MOTORE 4
fault4_stima1_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault4/fault4_stima1.csv')
fault4_stima2_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault4/fault4_stima2.csv')
fault4_stima3_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault4/fault4_stima3.csv')
fault4_stima4_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault4/fault4_stima4.csv')
fault4_stima5_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault4/fault4_stima5.csv')
fault4_stima6_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault4/fault4_stima6.csv')
fault4_stima7_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault4/fault4_stima7.csv')
fault4_stima8_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault4/fault4_stima8.csv')
fault4_stima9_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault4/fault4_stima9.csv')
fault4_stima10_file = open('/home/jcacace/dev/ros_ws/src/AERIAL/lee_controller/NN/Fault4/fault4_stima10.csv')

##NO FAULT
stima1 = pd.read_csv(stima1_file)
stima1 = stima1.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
target1 = stima1.pop('fault')
[row1,col1]=stima1.values.shape
x1_data = stima1.values[0:row1]
y1_data = target1.values[0:row1]

stima2 = pd.read_csv(stima2_file)
stima2 = stima2.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
target2 = stima2.pop('fault')
[row2,col2]=stima2.values.shape
x2_data = stima2.values[0:row2]
y2_data = target2.values[0:row2]

stima3 = pd.read_csv(stima3_file)
stima3 = stima3.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
target3 = stima3.pop('fault')
[row3,col3]=stima3.values.shape
x3_data = stima3.values[0:row3]
y3_data = target3.values[0:row3]

stima4 = pd.read_csv(stima4_file)
stima4 = stima4.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
target4 = stima4.pop('fault')
[row4,col4]=stima4.values.shape
x4_data = stima4.values[0:row4]
y4_data = target4.values[0:row4]

stima5 = pd.read_csv(stima5_file)
stima5 = stima5.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
target5 = stima5.pop('fault')
[row5,col5]=stima5.values.shape
x5_data = stima5.values[0:row5]
y5_data = target5.values[0:row5]

stima6 = pd.read_csv(stima6_file)
stima6 = stima6.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
target6 = stima6.pop('fault')
[row6,col6]=stima6.values.shape
x6_data = stima6.values[0:row6]
y6_data = target6.values[0:row6]

stima7 = pd.read_csv(stima7_file)
stima7 = stima7.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
target7 = stima7.pop('fault')
[row7,col7]=stima7.values.shape
x7_data = stima7.values[0:row7]
y7_data = target7.values[0:row7]

stima8 = pd.read_csv(stima8_file)
stima8 = stima8.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
target8 = stima8.pop('fault')
[row8,col8]=stima8.values.shape
x8_data = stima8.values[0:row8]
y8_data = target8.values[0:row8]

stima9 = pd.read_csv(stima9_file)
stima9 = stima9.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
target9 = stima9.pop('fault')
[row9,col9]=stima9.values.shape
x9_data = stima9.values[0:row9]
y9_data = target9.values[0:row9]

stima10 = pd.read_csv(stima10_file)
stima10 = stima10.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
target10 = stima10.pop('fault')
[row10,col10]=stima10.values.shape
x10_data = stima10.values[0:row10]
y10_data = target10.values[0:row10]

###FAULT MOTORE 1
fault_stima1 = pd.read_csv(fault_stima1_file)
fault_stima1 = fault_stima1.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault_target1 = fault_stima1.pop('fault')
[f_row1,f_col1]=fault_stima1.values.shape
x1_fault_data = fault_stima1.values[0:f_row1]
y1_fault_data = fault_target1.values[0:f_row1]

fault_stima2 = pd.read_csv(fault_stima2_file)
fault_stima2 = fault_stima2.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault_target2 = fault_stima2.pop('fault')
[f_row2,f_col2]=fault_stima2.values.shape
x2_fault_data = fault_stima2.values[0:f_row2]
y2_fault_data = fault_target2.values[0:f_row2]

fault_stima3 = pd.read_csv(fault_stima3_file)
fault_stima3 = fault_stima3.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault_target3 = fault_stima3.pop('fault')
[f_row3,f_col3]=fault_stima3.values.shape
x3_fault_data = fault_stima3.values[0:f_row3]
y3_fault_data = fault_target3.values[0:f_row3]

fault_stima4 = pd.read_csv(fault_stima4_file)
fault_stima4 = fault_stima4.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault_target4 = fault_stima4.pop('fault')
[f_row4,f_col1]=fault_stima4.values.shape
x4_fault_data = fault_stima4.values[0:f_row4]
y4_fault_data = fault_target4.values[0:f_row4]

fault_stima5 = pd.read_csv(fault_stima5_file)
fault_stima5 = fault_stima5.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault_target5 = fault_stima5.pop('fault')
[f_row5,f_col5]=fault_stima5.values.shape
x5_fault_data = fault_stima5.values[0:f_row5]
y5_fault_data = fault_target5.values[0:f_row5]

fault_stima6 = pd.read_csv(fault_stima6_file)
fault_stima6 = fault_stima6.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault_target6 = fault_stima6.pop('fault')
[f_row6,f_col6]=fault_stima6.values.shape
x6_fault_data = fault_stima6.values[0:f_row6]
y6_fault_data = fault_target6.values[0:f_row6]

fault_stima7 = pd.read_csv(fault_stima7_file)
fault_stima7 = fault_stima7.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault_target7 = fault_stima7.pop('fault')
[f_row7,f_col7]=fault_stima7.values.shape
x7_fault_data = fault_stima7.values[0:f_row7]
y7_fault_data = fault_target7.values[0:f_row7]

fault_stima8 = pd.read_csv(fault_stima8_file)
fault_stima8 = fault_stima8.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault_target8 = fault_stima8.pop('fault')
[f_row8,f_col8]=fault_stima8.values.shape
x8_fault_data = fault_stima8.values[0:f_row8]
y8_fault_data = fault_target8.values[0:f_row8]

fault_stima9 = pd.read_csv(fault_stima9_file)
fault_stima9 = fault_stima9.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault_target9 = fault_stima9.pop('fault')
[f_row9,f_col9]=fault_stima9.values.shape
x9_fault_data = fault_stima9.values[0:f_row9]
y9_fault_data = fault_target9.values[0:f_row9]

fault_stima10 = pd.read_csv(fault_stima10_file)
fault_stima10 = fault_stima10.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault_target10 = fault_stima10.pop('fault')
[f_row10,f_col10]=fault_stima10.values.shape
x10_fault_data = fault_stima10.values[0:f_row10]
y10_fault_data = fault_target10.values[0:f_row10]

###FAULT MOTORE 2
fault2_stima1 = pd.read_csv(fault2_stima1_file)
fault2_stima1 = fault2_stima1.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault2_target1 = fault2_stima1.pop('fault')
[f2_row1,f2_col1]=fault2_stima1.values.shape
x1_fault2_data = fault2_stima1.values[0:f2_row1]
y1_fault2_data = fault2_target1.values[0:f2_row1]

fault2_stima2 = pd.read_csv(fault2_stima2_file)
fault2_stima2 = fault2_stima2.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault2_target2 = fault2_stima2.pop('fault')
[f2_row2,f2_col2]=fault2_stima2.values.shape
x2_fault2_data = fault2_stima2.values[0:f2_row2]
y2_fault2_data = fault2_target2.values[0:f2_row2]

fault2_stima3 = pd.read_csv(fault2_stima3_file)
fault2_stima3 = fault2_stima3.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault2_target3 = fault2_stima3.pop('fault')
[f2_row3,f2_col3]=fault2_stima3.values.shape
x3_fault2_data = fault2_stima3.values[0:f2_row3]
y3_fault2_data = fault2_target3.values[0:f2_row3]

fault2_stima4 = pd.read_csv(fault2_stima4_file)
fault2_stima4 = fault2_stima4.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault2_target4 = fault2_stima4.pop('fault')
[f2_row4,f2_col1]=fault2_stima4.values.shape
x4_fault2_data = fault2_stima4.values[0:f2_row4]
y4_fault2_data = fault2_target4.values[0:f2_row4]

fault2_stima5 = pd.read_csv(fault2_stima5_file)
fault2_stima5 = fault2_stima5.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault2_target5 = fault2_stima5.pop('fault')
[f2_row5,f2_col5]=fault2_stima5.values.shape
x5_fault2_data = fault2_stima5.values[0:f2_row5]
y5_fault2_data = fault2_target5.values[0:f2_row5]

fault2_stima6 = pd.read_csv(fault2_stima6_file)
fault2_stima6 = fault2_stima6.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault2_target6 = fault2_stima6.pop('fault')
[f2_row6,f2_col6]=fault2_stima6.values.shape
x6_fault2_data = fault2_stima6.values[0:f2_row6]
y6_fault2_data = fault2_target6.values[0:f2_row6]

fault2_stima7 = pd.read_csv(fault2_stima7_file)
fault2_stima7 = fault2_stima7.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault2_target7 = fault2_stima7.pop('fault')
[f2_row7,f2_col7]=fault2_stima7.values.shape
x7_fault2_data = fault2_stima7.values[0:f2_row7]
y7_fault2_data = fault2_target7.values[0:f2_row7]

fault2_stima8 = pd.read_csv(fault2_stima8_file)
fault2_stima8 = fault2_stima8.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault2_target8 = fault2_stima8.pop('fault')
[f2_row8,f2_col8]=fault2_stima8.values.shape
x8_fault2_data = fault2_stima8.values[0:f2_row8]
y8_fault2_data = fault2_target8.values[0:f2_row8]

fault2_stima9 = pd.read_csv(fault2_stima9_file)
fault2_stima9 = fault2_stima9.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault2_target9 = fault2_stima9.pop('fault')
[f2_row9,f2_col9]=fault2_stima9.values.shape
x9_fault2_data = fault2_stima9.values[0:f2_row9]
y9_fault2_data = fault2_target9.values[0:f2_row9]

fault2_stima10 = pd.read_csv(fault2_stima10_file)
fault2_stima10 = fault2_stima10.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault2_target10 = fault2_stima10.pop('fault')
[f2_row10,f2_col10]=fault2_stima10.values.shape
x10_fault2_data = fault2_stima10.values[0:f2_row10]
y10_fault2_data = fault2_target10.values[0:f2_row10]

###FAULT MOTORE 3
fault3_stima1 = pd.read_csv(fault3_stima1_file)
fault3_stima1 = fault3_stima1.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault3_target1 = fault3_stima1.pop('fault')
[f3_row1,f3_col1]=fault3_stima1.values.shape
x1_fault3_data = fault3_stima1.values[0:f3_row1]
y1_fault3_data = fault3_target1.values[0:f3_row1]

fault3_stima2 = pd.read_csv(fault3_stima2_file)
fault3_stima2 = fault3_stima2.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault3_target2 = fault3_stima2.pop('fault')
[f3_row2,f3_col2]=fault3_stima2.values.shape
x2_fault3_data = fault3_stima2.values[0:f3_row2]
y2_fault3_data = fault3_target2.values[0:f3_row2]

fault3_stima3 = pd.read_csv(fault3_stima3_file)
fault3_stima3 = fault3_stima3.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault3_target3 = fault3_stima3.pop('fault')
[f3_row3,f3_col3]=fault3_stima3.values.shape
x3_fault3_data = fault3_stima3.values[0:f3_row3]
y3_fault3_data = fault3_target3.values[0:f3_row3]

fault3_stima4 = pd.read_csv(fault3_stima4_file)
fault3_stima4 = fault3_stima4.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault3_target4 = fault3_stima4.pop('fault')
[f3_row4,f3_col1]=fault3_stima4.values.shape
x4_fault3_data = fault3_stima4.values[0:f3_row4]
y4_fault3_data = fault3_target4.values[0:f3_row4]

fault3_stima5 = pd.read_csv(fault3_stima5_file)
fault3_stima5 = fault3_stima5.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault3_target5 = fault3_stima5.pop('fault')
[f3_row5,f3_col5]=fault3_stima5.values.shape
x5_fault3_data = fault3_stima5.values[0:f3_row5]
y5_fault3_data = fault3_target5.values[0:f3_row5]

fault3_stima6 = pd.read_csv(fault3_stima6_file)
fault3_stima6 = fault3_stima6.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault3_target6 = fault3_stima6.pop('fault')
[f3_row6,f3_col6]=fault3_stima6.values.shape
x6_fault3_data = fault3_stima6.values[0:f3_row6]
y6_fault3_data = fault3_target6.values[0:f3_row6]

fault3_stima7 = pd.read_csv(fault3_stima7_file)
fault3_stima7 = fault3_stima7.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault3_target7 = fault3_stima7.pop('fault')
[f3_row7,f3_col7]=fault3_stima7.values.shape
x7_fault3_data = fault3_stima7.values[0:f3_row7]
y7_fault3_data = fault3_target7.values[0:f3_row7]

fault3_stima8 = pd.read_csv(fault3_stima8_file)
fault3_stima8 = fault3_stima8.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault3_target8 = fault3_stima8.pop('fault')
[f3_row8,f3_col8]=fault3_stima8.values.shape
x8_fault3_data = fault3_stima8.values[0:f3_row8]
y8_fault3_data = fault3_target8.values[0:f3_row8]

fault3_stima9 = pd.read_csv(fault3_stima9_file)
fault3_stima9 = fault3_stima9.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault3_target9 = fault3_stima9.pop('fault')
[f3_row9,f3_col9]=fault3_stima9.values.shape
x9_fault3_data = fault3_stima9.values[0:f3_row9]
y9_fault3_data = fault3_target9.values[0:f3_row9]

fault3_stima10 = pd.read_csv(fault3_stima10_file)
fault3_stima10 = fault3_stima10.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault3_target10 = fault3_stima10.pop('fault')
[f3_row10,f3_col10]=fault3_stima10.values.shape
x10_fault3_data = fault3_stima10.values[0:f3_row10]
y10_fault3_data = fault3_target10.values[0:f3_row10]

######FAULT MOTORE 4
fault4_stima1 = pd.read_csv(fault4_stima1_file)
fault4_stima1 = fault4_stima1.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault4_target1 = fault4_stima1.pop('fault')
[f4_row1,f4_col1]=fault4_stima1.values.shape
x1_fault4_data = fault4_stima1.values[0:f4_row1]
y1_fault4_data = fault_target1.values[0:f4_row1]

fault4_stima2 = pd.read_csv(fault4_stima2_file)
fault4_stima2 = fault4_stima2.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault4_target2 = fault4_stima2.pop('fault')
[f4_row2,f4_col2]=fault4_stima2.values.shape
x2_fault4_data = fault4_stima2.values[0:f4_row2]
y2_fault4_data = fault4_target2.values[0:f4_row2]

fault4_stima3 = pd.read_csv(fault4_stima3_file)
fault4_stima3 = fault4_stima3.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault4_target3 = fault4_stima3.pop('fault')
[f4_row3,f4_col3]=fault4_stima3.values.shape
x3_fault4_data = fault4_stima3.values[0:f4_row3]
y3_fault4_data = fault4_target3.values[0:f4_row3]

fault4_stima4 = pd.read_csv(fault4_stima4_file)
fault4_stima4 = fault4_stima4.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault4_target4 = fault4_stima4.pop('fault')
[f4_row4,f4_col1]=fault4_stima4.values.shape
x4_fault4_data = fault4_stima4.values[0:f4_row4]
y4_fault4_data = fault4_target4.values[0:f4_row4]

fault4_stima5 = pd.read_csv(fault4_stima5_file)
fault4_stima5 = fault4_stima5.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault4_target5 = fault4_stima5.pop('fault')
[f4_row5,f4_col5]=fault4_stima5.values.shape
x5_fault4_data = fault4_stima5.values[0:f4_row5]
y5_fault4_data = fault4_target5.values[0:f4_row5]

fault4_stima6 = pd.read_csv(fault4_stima6_file)
fault4_stima6 = fault4_stima6.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault4_target6 = fault4_stima6.pop('fault')
[f4_row6,f4_col6]=fault4_stima6.values.shape
x6_fault4_data = fault4_stima6.values[0:f4_row6]
y6_fault4_data = fault4_target6.values[0:f4_row6]

fault4_stima7 = pd.read_csv(fault4_stima7_file)
fault4_stima7 = fault4_stima7.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault4_target7 = fault4_stima7.pop('fault')
[f4_row7,f4_col7]=fault4_stima7.values.shape
x7_fault4_data = fault4_stima7.values[0:f4_row7]
y7_fault4_data = fault4_target7.values[0:f4_row7]

fault4_stima8 = pd.read_csv(fault4_stima8_file)
fault4_stima8 = fault4_stima8.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault4_target8 = fault4_stima8.pop('fault')
[f4_row8,f4_col8]=fault4_stima8.values.shape
x8_fault4_data = fault4_stima8.values[0:f4_row8]
y8_fault4_data = fault4_target8.values[0:f4_row8]

fault4_stima9 = pd.read_csv(fault4_stima9_file)
fault4_stima9 = fault4_stima9.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault4_target9 = fault4_stima9.pop('fault')
[f4_row9,f4_col9]=fault4_stima9.values.shape
x9_fault4_data = fault4_stima9.values[0:f4_row9]
y9_fault4_data = fault4_target9.values[0:f4_row9]

fault4_stima10 = pd.read_csv(fault4_stima10_file)
fault4_stima10 = fault4_stima10.loc[:, ['fault','Fx', 'Fy','Fz','Wx','Wy','Wz']]
fault4_target10 = fault4_stima10.pop('fault')
[f4_row10,f4_col10]=fault4_stima10.values.shape
x10_fault4_data = fault4_stima10.values[0:f4_row10]
y10_fault4_data = fault4_target10.values[0:f4_row10]


##DATI CONCATENATI MOTORE 1-2-3-4
x_train=np.concatenate((x1_data, x2_data, x3_data, x4_data, x5_data, x6_data, x7_data, x8_data, x9_data, x10_data,x1_fault_data,x2_fault_data,x3_fault_data,x4_fault_data,x5_fault_data,x6_fault_data,x7_fault_data,x8_fault_data,x9_fault_data,x1_fault2_data,x2_fault2_data,x3_fault2_data,x4_fault2_data,x5_fault2_data,x6_fault2_data,x7_fault2_data,x8_fault2_data,x9_fault2_data,x1_fault3_data,x2_fault3_data,x3_fault3_data,x4_fault3_data,x5_fault3_data,x6_fault3_data,x7_fault3_data,x8_fault3_data,x9_fault3_data,x1_fault4_data,x2_fault4_data,x3_fault4_data,x4_fault4_data,x5_fault4_data,x6_fault4_data,x7_fault4_data,x8_fault4_data,x9_fault4_data))

y_train=np.concatenate((y1_data, y2_data, y3_data, y4_data, y5_data, y6_data, y7_data, y8_data, y9_data, y10_data,y1_fault_data,y2_fault_data,y3_fault_data,y4_fault_data,y5_fault_data,y6_fault_data,y7_fault_data,y8_fault_data,y9_fault_data,y1_fault2_data,y2_fault2_data,y3_fault2_data,y4_fault2_data,y5_fault2_data,y6_fault2_data,y7_fault2_data,y8_fault2_data,y9_fault2_data,y1_fault3_data,y2_fault3_data,y3_fault3_data,y4_fault3_data,y5_fault3_data,y6_fault3_data,y7_fault3_data,y8_fault3_data,y9_fault3_data,y1_fault4_data,y2_fault4_data,y3_fault4_data,y4_fault4_data,y5_fault4_data,y6_fault4_data,y7_fault4_data,y8_fault4_data,y9_fault4_data))

num_rows, num_cols = x_train.shape
train_X = x_train.reshape((num_rows,1,num_cols))
train_Y = y_train.reshape((num_rows,1,1))

validation_x=np.concatenate((x10_data,x10_fault_data,x10_fault2_data,x10_fault3_data,x10_fault4_data))
validation_y=np.concatenate((y10_data,y10_fault_data,y10_fault2_data,y10_fault3_data,y10_fault4_data))

num_rows_val, num_cols_val = validation_x.shape
train_val_X = validation_x.reshape((num_rows_val,1,num_cols_val))
train_val_Y = validation_y.reshape((num_rows_val,1,1))


model = keras.Sequential()
model.add(layers.LSTM(200, input_shape=(1,num_cols)))
model.add(layers.Dense(1))
model.compile(loss='mae', optimizer='adam',metrics=['accuracy'])

history = model.fit(train_X, train_Y,epochs=10,validation_data=(train_val_X, train_val_Y))

model.save('/home/jcacace/Neural_net_v5.model')


import os
import sys
import cv2
import time
import curses
import socket
import pickle
import struct
import tracker_lib
from multiprocessing import Process, Value, Array, Manager
from collections import deque
from itertools import cycle
from yamspy import MSPy
from threading import Thread
from filterpy.memory import FadingMemoryFilter
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.ticker as ticker
from scipy.signal import argrelextrema
import json


# data = {"list_time_thr":list_time_thr, 
#         "list_rc_thr":list_rc_thr, 
#         "list_target_thr":list_target_thr,
#         "list_pid_thr":list_pid_thr,

#         "list_target_yaw":list_target_yaw,
#         "list_rc_yaw":list_rc_yaw,
#         "list_time_yaw":list_time_yaw,
#         "list_pid_yaw":list_pid_yaw

#         }

def ziegler_nichols_tuning(Ku, Tu):

    # PID coefficients
    Kp_pid = 0.6 * Ku
    Ki_pid = 2 * Kp_pid / Tu
    Kd_pid = Kp_pid * Tu / 8
    
    # PI coefficients
    Kp_pi = 0.45 * Ku
    Ki_pi = 1.2 * Kp_pi / Tu
    
    pid_params = (Kp_pid, Ki_pid, Kd_pid)
    pi_params = (Kp_pi, Ki_pi)
    
    return pid_params, pi_params

with open('data.json') as f:
    # data = {"list_time":list_time, 
    #         "list_rc":list_rc, 
    #         "list_target":list_target,
    #         "last_Kp": Kp 
    #         }
    json_data = f.read()

# Parse the JSON data and convert it into a Python dictionary
data = json.loads(json_data)

# START = 950
# END = 1550
OS = "thr"

START = 0
END = len(data["list_time_"+OS])

# visual
temp_time_list = data["list_time_"+OS][START:END]
sumOfNums = sum(temp_time_list)
count = len(temp_time_list)
average = sumOfNums / count
list_time_arr = np.arange(0, count)#np.array(list_time)

np_list_pid_thr = np.array(data["list_pid_"+OS])
sumKp = sum(np_list_pid_thr[START:END][0])
averageKp = sumKp/count


print (averageKp, 
       sum(temp_time_list[:]), 
       ziegler_nichols_tuning(averageKp, sum(temp_time_list[:])))
sum_lisg_rc = sum(data["list_rc_"+OS][START:END])
average_rc = sum_lisg_rc/count

list_rc_arr = np.array(data["list_rc_"+OS][START:END])
ix_max = argrelextrema(list_rc_arr, np.greater)
ix_min = argrelextrema(list_rc_arr, np.less)

plt.scatter(list_time_arr[ix_max], list_rc_arr[ix_max])
plt.scatter(list_time_arr[ix_min], list_rc_arr[ix_min])
plt.plot(data["list_rc_"+OS][START:END])
plt.axline((0, average_rc), (count, average_rc))
plt.title('PID')
plt.xlabel('step')
plt.ylabel('throttle')
plt.show()
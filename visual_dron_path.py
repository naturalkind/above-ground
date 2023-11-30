"""
1 подключиться к среде air sim
2 получать из среды данные положения 
3 визуализация в matplotlib
human robotics
"""
## визуализация matplotlib
## отслеживать положение дрона в реальном времени
## https://stackoverflow.com/questions/38118598/how-to-create-a-3d-animation
## https://stackoverflow.com/questions/9401658/how-to-animate-a-scatter-plot
import airsim
import os
import sys
import cv2
import numpy as np
import time

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation


all_data = []
origin_UE = np.array([0.0, 0.0, 0.0]) #910.0

def convert_pos_UE_to_AS(origin_UE : np.array, pos_UE : np.array):
    pos = np.zeros(3, dtype=np.float)
    pos[0] = pos_UE[0] - origin_UE[0]
    pos[1] = pos_UE[1] - origin_UE[1]
    pos[2] = -5
#    pos[2] = - pos_UE[2] + origin_UE[2]
    return pos / 100


with open('/media/sadko/unrealdir/AboveGenSim/Saved/CoordData/test.txt','r') as file:
#with open('/media/sadko/unrealdir/AboveGenSim/Saved/CoordData/coord_scren.txt','r') as file:
    for line in file:
        coord_list = [float(i.split("=")[-1]) for i in line.split("\n")[0].split(" ")]
        coord_list = convert_pos_UE_to_AS(origin_UE, np.array(coord_list))
        all_data.append(airsim.Vector3r(coord_list[0], coord_list[1], coord_list[2]))
  
print (all_data[0], all_data[-1], "LOADING DONE")        
global len_data
len_data = len(all_data)

data_to_learn = np.array([d.to_numpy_array() for d in all_data])
print (data_to_learn.shape) # (17767, 3)


## connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

client.armDisarm(True)

fig = plt.figure(figsize=(10,10))
ax = fig.add_subplot(projection='3d')
#ax = plt.axes(projection='3d')

def update(num, data, line):
    origin_UE = client.getMultirotorState().kinematics_estimated.position.to_numpy_array()
    p_point._offsets3d = [[origin_UE[0]], [origin_UE[1]], [origin_UE[2]]]
#    p_point.set_3d_properties(origin_UE[2])
#    p_point = ax.scatter3D(origin_UE[0], origin_UE[1], origin_UE[2], cmap='Greens')
    print (origin_UE, data_to_learn[0])
#    [72.69754   12.600049   2.2182498] [ 2.4   0.   -0.05]    

N = 100
data = data_to_learn
zline = data_to_learn[:,2]
xline = data_to_learn[:,0]
yline = data_to_learn[:,1]

ax.plot3D(xline, yline, zline, 'gray')

origin_UE = client.getMultirotorState().kinematics_estimated.position.to_numpy_array()
p_point = ax.scatter3D(origin_UE[0], origin_UE[1], origin_UE[2], cmap='Greens')
#p_point = ax.scatter(origin_UE[0], origin_UE[1], origin_UE[2], cmap='Greens')

ani = animation.FuncAnimation(fig, update, N, fargs=(data, line), interval=10000/N, blit=False)
plt.show()



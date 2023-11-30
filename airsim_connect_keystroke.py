# ready to run example: PythonClient/multirotor/hello_drone.py
import airsim
import os
import sys
import cv2
import numpy as np
import time
from threading import Thread
from multiprocessing import Process

all_data = []
origin_UE = np.array([0.0, 0.0, 910.0])
z = -20

def convert_pos_UE_to_AS(origin_UE : np.array, pos_UE : np.array):
    pos = np.zeros(3, dtype=np.float)
    pos[0] = pos_UE[0] - origin_UE[0]
    pos[1] = pos_UE[1] - origin_UE[1]
    pos[2] = - pos_UE[2] + origin_UE[2]
    return pos / 100


# open file coord from spline
with open("data\coord_scren.txt",'r') as file:
    for line in file:
        coord_list = [float(i.split("=")[-1]) for i in line.split("\n")[0].split(" ")]
        #all_data.append(coord_list)
        print (coord_list)
        all_data.append(airsim.Vector3r(coord_list[0],coord_list[1], z))
        #all_data.append(airsim.Vector3r(convert_pos_UE_to_AS(origin_UE, np.array(coord_list))))
        #print(coord_list)
global idx
idx = 0
global len_data
len_data = len(all_data)

# вывод изображения
def imgs(x):
    cv2.imshow('image', np.array(x))
    key = cv2.waitKey(1) & 0xff
    if key==ord('q'):
        cv2.destroyAllWindows()

# For high speed ascent and descent on PX4 you may need to set these properties:
# param set MPC_Z_VEL_MAX_UP 5
# param set MPC_Z_VEL_MAX_DN 5

# a custom function that blocks for a moment
def task():
    client = airsim.MultirotorClient()
    
    client.confirmConnection()
    #client.enableApiControl(True)

    #client.armDisarm(True)
    while True:
#    for i in range(10):
        
    #    print (dir(client))
    #    r_state = client.getRotorStates()
    #    print (r_state)

        # work
    #    responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False)])
        responses = client.simGetImages([airsim.ImageRequest("3", airsim.ImageType.Segmentation, False, False)])
        response = responses[0]

        # get numpy array
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) 

        # reshape array to 4 channel image array H X W X 4
        img_rgb = img1d.reshape(response.height, response.width, 3)
        #print (img_rgb.shape)
        imgs(img_rgb)
        
#        idx += 1
        pos = client.getMultirotorState().kinematics_estimated.position
        #print (pos)
    


def task2():
    client2 = airsim.MultirotorClient()
    client2.confirmConnection()
##    client2.enableApiControl(True)
##    client2.armDisarm(True)
    landed = client2.getMultirotorState().landed_state
    
    # Save data
##    with open("ttx.txt", "w") as file:
##        while True:
##            file.write(f"{client2.getMultirotorState().rc_data.pitch};{client2.getMultirotorState().rc_data.roll};{client2.getMultirotorState().rc_data.throttle};{client2.getMultirotorState().rc_data.yaw};\n")


    with open("ttx.txt", "r") as file:
        D = file.readlines()            
        for i in D:
            t_data = i.split("\n")[0].split(";")[:-1]
            t_data = [float(x) for x in t_data]
            print (t_data)
            client2.moveByRC(rcdata = airsim.RCData(pitch = t_data[0], throttle = t_data[2], yaw=t_data[3], roll=t_data[1], is_initialized = True, is_valid = True))


if __name__ == "__main__":

    thread = Process(target=task, daemon=True)
    # run the thread
    thread.start()
    # wait for the thread to finish
    print('Waiting for the thread...')
#    thread.join() # orig
 
 
    thread2 = Process(target=task2, daemon=True)
    # run the thread
    thread2.start()
    # wait for the thread to finish
    print('Waiting for the thread...')
    thread.join()   
    thread2.join()



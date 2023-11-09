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
# open file coord from spline
with open('/media/sadko/unrealdir/AboveGenSim/Saved/CoordData/test.txt','r') as file:
    for line in file:
        coord_list = [float(i.split("=")[-1]) for i in line.split("\n")[0].split(" ")]
        all_data.append(coord_list )
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

def task2():
    print ("-------------------->")
    z = 5
    landed = client2.getMultirotorState().landed_state
    if landed == airsim.LandedState.Landed:
        print("taking off...")
        client2.takeoffAsync().join()
    else:
        print("already flying...")
        client2.hoverAsync().join()
    print("make sure we are hovering at {} meters...".format(z))

    if z > 5:
        # AirSim uses NED coordinates so negative axis is up.
        # z of -50 is 50 meters above the original launch point.
        client2.moveToZAsync(-z, 5).join()
        client2.hoverAsync().join()
        time.sleep(5)

    if z > 10:
        print("come down quickly to 10 meters...")
        z = 10
        client2.moveToZAsync(-z, 3).join()
        client2.hoverAsync().join()

    print("landing...")
    client2.landAsync().join()
    print("disarming...")
    client2.armDisarm(False)
    client2.enableApiControl(False)
    print("done.")

if __name__ == "__main__":

    z = 5
    if len(sys.argv) > 1:
        z = float(sys.argv[1])
    ### connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)

    client.armDisarm(True)
    
    
    
    client2 = airsim.MultirotorClient()
    client2.confirmConnection()
    client2.enableApiControl(True)
    client2.armDisarm(True)
    
    
    # create a thread
#    thread = Thread(target=task)
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

###########################
#####
###########################


# 
#def to_NED(coords_UE4, ground_offset_NED, player_start_UE4):
#    ''' Converts Unreal coordinates to NED system.
#        Assumes PlayerStart is at (0, 0, 0) in AirSim's local NED coordinate system. '''
#    #print (coords_UE4, "\n", ground_offset_NED,  "\n",  player_start_UE4)
#    
#    coords_NED = coords_UE4 - player_start_UE4  # Unreal uses cm and +z aiming up
#    coords_NED.z_val *= -1
#    coords_NED *= 0.01
#    coords_NED = coords_NED + ground_offset_NED
#    return coords_NED.to_numpy_array() 
#        
##print ("START->", all_data[10][0], all_data[10][1], all_data[10][2])
#### Async methods returns Future. Call join() to wait for task to complete.
##client.takeoffAsync().join()
##client.moveToPositionAsync(all_data[10][0], all_data[10][1], all_data[10][2], 1).join()

#idx = 0
#len_data = len(all_data)
#while len_data > idx:
##    print (dir(client))
##    r_state = client.getRotorStates()
##    print (r_state)

#    # work
##    responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False)])
#    responses = client.simGetImages([airsim.ImageRequest("3", airsim.ImageType.Segmentation, False, False)])
#    response = responses[0]

#    # get numpy array
#    img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) 

#    # reshape array to 4 channel image array H X W X 4
#    img_rgb = img1d.reshape(response.height, response.width, 3)
#    print (img_rgb.shape)
#    imgs(img_rgb)
#    
#    
#    ## Async methods returns Future. Call join() to wait for task to complete.
##    client.takeoffAsync().join()
#    client.moveToPositionAsync(100, 100, 100, 1).join()
#    print (all_data[idx][0], all_data[idx][1], all_data[idx][2], idx)
#    idx += 1
#    



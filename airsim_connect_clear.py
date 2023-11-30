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
origin_UE = np.array([0.0, 0.0, 0.0]) #910.0
z = -20

def convert_pos_UE_to_AS(origin_UE : np.array, pos_UE : np.array):
    pos = np.zeros(3, dtype=np.float)
    pos[0] = pos_UE[0] - origin_UE[0]
    pos[1] = pos_UE[1] - origin_UE[1]
    pos[2] = -1
#    pos[2] = - pos_UE[2] + origin_UE[2]
    return pos / 100


with open('/media/sadko/unrealdir/AboveGenSim/Saved/CoordData/test.txt','r') as file:
#with open('/media/sadko/unrealdir/AboveGenSim/Saved/CoordData/coord_scren.txt','r') as file:
    for line in file:
        coord_list = [float(i.split("=")[-1]) for i in line.split("\n")[0].split(" ")]
        coord_list = convert_pos_UE_to_AS(origin_UE, np.array(coord_list))
        all_data.append(airsim.Vector3r(coord_list[0], coord_list[1], coord_list[2]))
  
print (all_data[0], "LOADING DONE")        
global len_data
len_data = len(all_data)

# вывод изображения
def imgs(x):
    cv2.imshow('image', np.array(x))
    key = cv2.waitKey(1) & 0xff
    if key==ord('q'):
        cv2.destroyAllWindows()


# a custom function that blocks for a moment
def task():
    while True:

        # work
#        responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False)])
        responses = client.simGetImages([airsim.ImageRequest("3", airsim.ImageType.Segmentation, False, False)])
        response = responses[0]
        # get numpy array
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) 
        # reshape array to 4 channel image array H X W X 4
        img_rgb = img1d.reshape(response.height, response.width, 3)
        #print (img_rgb.shape)
        imgs(img_rgb)
        pos = client.getMultirotorState().kinematics_estimated.position
#        print (pos)
#        print (all_data[-1])
#        print (all_data[0])
#        print ("------------------------")



def task2():
#    landed = client2.getMultirotorState().landed_state
#    if landed == airsim.LandedState.Landed:
#        print("taking off...")
#        client2.takeoffAsync().join()
#    else:
#        print("already flying...")

#        client2.hoverAsync().join()    
#        client2.landAsync().join()
    client2.moveToZAsync(-2, 2).join()
    
#    print("flying on coord position...") 
#    for idx, data in enumerate(all_data[::10]): # all_data
#        data = data.to_numpy_array()
#        client2.moveToPositionAsync(int(data[0]), int(data[1]), int(data[2]), 15).join()
#        print (f"Point idx {idx}, CoordPath:", data)     

    print("flying on path coord position...") 
    result = client2.moveOnPathAsync(all_data, 12, 120, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), 20, 1).join()
    # airsim.DrivetrainType.ForwardOnly  https://github.com/microsoft/AirSim/issues/772 
    

# ----------> https://gitmemories.com/microsoft/AirSim/issues/4691
# ----------> https://github.com/Microsoft/AirSim/blob/main/PythonClient/multirotor/path.py
if __name__ == "__main__":

    ### connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)

    client.armDisarm(True)
    
    client2 = airsim.MultirotorClient()
    client2.reset()
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
 
    # test
    origin_UE = client.getMultirotorState().kinematics_estimated.position.to_numpy_array()
    origin_UE2 = client.getHomeGeoPoint()
#    client.simPlotPoints(all_data, color_rgba=[1.0, 0.0, 0.0, 1.0], size=10, duration=-1.0, is_persistent=False) # is_persistent=True - отображение пути нужным цветом
    print (".................", origin_UE2, client.listVehicles())
 
    thread2 = Process(target=task2, daemon=True)
    # run the thread
    thread2.start()
    # wait for the thread to finish
    print('Waiting for the thread...')
    thread.join()   
    thread2.join()


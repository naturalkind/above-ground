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
    pos[2] = - pos_UE[2] + origin_UE[2]
    return pos / 100


# open file coord from spline
#'/media/sadko/unrealdir/AboveGenSim/Saved/CoordData/test.txt'
with open('/media/sadko/unrealdir/AboveGenSim/Saved/CoordData/coord_scren.txt','r') as file:
    for line in file:
        coord_list = [float(i.split("=")[-1]) for i in line.split("\n")[0].split(" ")]

        
#        all_data.append(airsim.Vector3r(coord_list[0],coord_list[1], z))
        
#        all_data.append(airsim.Vector3r(coord_list[0], coord_list[1], coord_list[2]))
        coord_list = convert_pos_UE_to_AS(origin_UE, np.array(coord_list))
        all_data.append(airsim.Vector3r(coord_list[0], coord_list[1], -coord_list[2]))
        
        #print ("------------------------>", coord_list)
        #print ("------ VECTOR ---------->", airsim.Vector3r(coord_list[0], coord_list[1], coord_list[2]))
        #print ("------ VECTOR ---------->", convert_pos_UE_to_AS(origin_UE, np.array(coord_list)))
  
print (all_data[0], "LOADING DONE")        
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
        pos = client.getMultirotorState().kinematics_estimated.position 

#<Vector3r> 
#{   'x_val': 1.8158369064331055,
#    'y_val': -2.2069244384765625,
#    'z_val': -0.4622829258441925}
#<Vector3r> 
#{   'x_val': 103.3,
#    'y_val': 0.0,
#    'z_val': 228.4}
#<Vector3r> 
#{   'x_val': 9.81014,
#    'y_val': -9.85656,
#    'z_val': 228.4}


#def task2():
#    print ("-------------------->")
#    z = 5
#    landed = client2.getMultirotorState().landed_state
#    if landed == airsim.LandedState.Landed:
#        print("taking off...")
#        client2.takeoffAsync().join()
#    else:
#        print("already flying...")
#        client2.hoverAsync().join()
#    print("make sure we are hovering at {} meters...".format(z))

#    if z > 5:
#        # AirSim uses NED coordinates so negative axis is up.
#        # z of -50 is 50 meters above the original launch point.
#        client2.moveToZAsync(-z, 5).join()
#        client2.hoverAsync().join()
#        time.sleep(5)

#    if z > 10:
#        print("come down quickly to 10 meters...")
#        z = 10
#        client2.moveToZAsync(-z, 3).join()
#        client2.hoverAsync().join()

#    print("landing...")
#    client2.landAsync().join()
#    print("disarming...")
#    client2.armDisarm(False)
#    client2.enableApiControl(False)
#    print("done.")


def task2():
#    landed = client2.getMultirotorState().landed_state
#    if landed == airsim.LandedState.Landed:
#        print("taking off...")
#        client2.takeoffAsync().join()
#    else:
#        print("already flying...")

#        client2.hoverAsync().join()    
#        client2.landAsync().join()
    client2.moveToZAsync(-5, 5).join()
    print("flying on coord position...") 
#    for idx, data in enumerate(all_data):
#        data = data.to_numpy_array()
#        print (f"Point idx {idx}, CoordPath:", data) 
#        
##        client2.moveToPositionAsync(int(data[0]), int(data[1]), -int(data[2]), 5).join()
#        client2.moveToPositionAsync(int(data[0]), int(data[1]), 0, 5).join()
    
    print("flying on path...") 
    result = client2.moveOnPathAsync(all_data, 12, 120, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), 20, 1).join()
    
#    result = client2.moveOnPathAsync(all_data[:10], 12, 120, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), 20, 1).join()
#    result = client2.moveOnPathAsync([airsim.Vector3r(125,0,z),
#                                     airsim.Vector3r(125,-130,z),
#                                     airsim.Vector3r(0,-130,z),
#                                     airsim.Vector3r(0,0,z)],
#                            12, 120,
#                            airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), 20, 1).join()
    
# получать данные пользоватя с пульта    
def task_get_user_data():
    client2 = airsim.MultirotorClient()
    #client2.reset()
    client2.confirmConnection()
##    client2.enableApiControl(True)
##    client2.armDisarm(True)
    landed = client2.getMultirotorState().landed_state
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


##<class 'airsim.types.RCData'> <RCData> {   'is_initialized': True,
##    'is_valid': True,
##    'left_z': 1.0,
##    'pitch': -0.0,
##    'right_z': -1.0,
##    'roll': 0.003999948501586914,
##    'switches': 0,
##    'throttle': 0.49949997663497925,
##    'timestamp': 0,
##    'vendor_id': 'VID_1209',
##    'yaw': 0.054000020027160645}
##        airsim.RCData({""})
        #print (type(client2.getMultirotorState().rc_data), client2.getMultirotorState().rc_data.roll)
##        print(dir(airsim.RCData), airsim.RCData.yaw, airsim.RCData.is_initialized) 




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
 
    origin_UE = client.getMultirotorState().kinematics_estimated.position.to_numpy_array()
 
    thread2 = Process(target=task2, daemon=True)
    # run the thread
    thread2.start()
    # wait for the thread to finish
    print('Waiting for the thread...')
    thread.join()   
    thread2.join()




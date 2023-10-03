# ready to run example: PythonClient/multirotor/hello_drone.py
import airsim
import os
import cv2
import numpy as np

def imgs(x):
    cv2.imshow('image', np.array(x))
    cv2.waitKey(1)
    key = cv2.waitKey(1) & 0xff
    if key==ord('q'):
        cv2.destroyAllWindows()

## connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

all_data = []
# open file coord from spline
with open('/media/sadko/1b32d2c7-3fcf-4c94-ad20-4fb130a7a7d4/PLAYGROUND/above-ground/data/coord_scren.txt','r') as file:
    for line in file:
        coord_list = [float(i.split("=")[-1]) for i in line.split("\n")[0].split(" ")]
        all_data.append(coord_list )
        print(coord_list)
        
#print ("START->", all_data[10][0], all_data[10][1], all_data[10][2])
### Async methods returns Future. Call join() to wait for task to complete.
#client.takeoffAsync().join()
#client.moveToPositionAsync(all_data[10][0], all_data[10][1], all_data[10][2], 1).join()

idx = 0
while True:
#    print (dir(client))
#    r_state = client.getRotorStates()
#    print (r_state)
    responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
    response = responses[0]

    # get numpy array
    img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) 

    # reshape array to 4 channel image array H X W X 4
    img_rgb = img1d.reshape(response.height, response.width, 3)
    
    imgs(img_rgb)
    
    
    ## Async methods returns Future. Call join() to wait for task to complete.
    client.takeoffAsync().join()
    client.moveToPositionAsync(all_data[idx][0], all_data[idx][1], all_data[idx][2], 1).join()
    idx += 1

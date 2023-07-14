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

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)



# Async methods returns Future. Call join() to wait for task to complete.
#client.takeoffAsync().join()
#client.moveToPositionAsync(-10, 10, -10, 5).join()
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
    

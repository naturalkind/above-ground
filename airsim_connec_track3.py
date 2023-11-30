# ready to run example: PythonClient/multirotor/hello_drone.py
import airsim
import os
import sys
import cv2
import numpy as np
from numpy.linalg import norm
import time
import math
import tracker_lib
from threading import Thread
from multiprocessing import Process, Value, Array, Manager
from pyquaternion import Quaternion
from quadcopter_with_PID_controller.PID_Controller import PID_Controller

origin_UE = np.array([0.0, 0.0, 910.0])
z = -20

def convert_pos_UE_to_AS(origin_UE : np.array, pos_UE : np.array):
    pos = np.zeros(3, dtype=np.float)
    pos[0] = pos_UE[0] - origin_UE[0]
    pos[1] = pos_UE[1] - origin_UE[1]
    pos[2] = - pos_UE[2] + origin_UE[2]
    return pos / 100


global lib_start
lib_start = tracker_lib.TrackerLib()


# вывод изображения
def imgs(x):
    cv2.imshow('image', np.array(x))
    key = cv2.waitKey(1) & 0xff
    if key==ord('q'):
        cv2.destroyAllWindows()



# https://stackoverflow.com/questions/56207448/efficient-quaternions-to-euler-transformation
def quaternion_to_euler_angle_vectorized(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2>+1.0,+1.0,t2)

    t2 = np.where(t2<-1.0, -1.0, t2)
    Y = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.degrees(np.arctan2(t3, t4))

    return X, Y, Z 


def quaternion_to_euler_angle_vectorized2(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)

    t2 = np.clip(t2, a_min=-1.0, a_max=1.0)
    Y = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.degrees(np.arctan2(t3, t4))

    return X, Y, Z

#------------>

# DeepAI
def euler_from_quaternion(q):
    """
    Convert a quaternion to Euler angles (roll, pitch, yaw) in radians.
    """
    t0 = +2.0 * (q.w_val * q.x_val + q.y_val * q.z_val)
    t1 = +1.0 - 2.0 * (q.x_val * q.x_val + q.y_val * q.y_val)
    roll = math.atan2(t0, t1)
    
    t2 = +2.0 * (q.w_val * q.y_val - q.z_val * q.x_val)
    t3 = +1.0 - 2.0 * (q.y_val * q.y_val + q.z_val * q.z_val)
    pitch = math.atan2(t2, t3)
    
    yaw = math.atan2(2 * (q.w_val * q.z_val + q.x_val * q.y_val), (q.x_val * q.x_val + q.y_val * q.y_val))
    
    return roll, pitch, yaw

#last_orientation_quat = quaternion(*client.getMultirotorState().kinematics.orientation) # assuming kinematics is a PX4Firmware class with a .orientation tuple of (w, x, y, z) floats in quaternion format
#roll, pitch, yaw = euler_from_quaternion(last_orientation_quat) # converting to Euler angles using the euler_from_quaternion function from the quaternion library


def percent_to_control(percent_x, percent_y):
    # Convert percentages to normalized coordinates (-1 to 1)
    x = (percent_x - 50) / 50.0
    y = (percent_y - 50) / 50.0

    # Calculate desired rotation and translation for drone movement
    rotation = Quaternion(axis=[0, 0, 1], angle=np.arctan2(y, x) * 2)
    print (rotation)
    translation = [x * 2, y * 2]

    # Convert rotation and translation to control signals for airsim API
    #print (dir(rotation))
    roll = rotation.vector[2]
    pitch = -rotation.vector[1]
    yaw = np.arctan2(rotation.vector[0], np.sqrt(rotation.vector[1]**2 + rotation.vector[2]**2))
    linear_x = translation[0]
    linear_y = translation[1]
    linear_z = 0.0

    return [roll, pitch, yaw, linear_x, linear_y, linear_z]

# DeepAI END

# автоматический взлёт airsim
def auto_up_drone(client):
    client.enableApiControl(True)
    client.armDisarm(True)
    
    landed = client.getMultirotorState().landed_state
    if landed == airsim.LandedState.Landed:
        print("taking off...")
        client.takeoffAsync().join()
    else:
        print("already flying...")
        client.hoverAsync().join()    
    client.moveToZAsync(-10, 20).join()
    client.hoverAsync().join()

# INFO
#client.moveByRC(rcdata = airsim.RCData(pitch = t_data[0],
#                                       throttle = t_data[2],
#                                       yaw=t_data[3],
#                                       roll=t_data[1],
#                                       is_initialized = True,
#                                       is_valid = True))

#Вперёд
#    pitch = 1.0,
#    throttle = 1.0,
#    yaw=0.0,
#    roll=0.0
#    
#Назад
#    pitch = -1.0,
#    throttle = 1.0,
#    yaw=0.0,
#    roll=0.0        
#    
#Вправо
#    pitch = 0.0,
#    throttle = 1.0,
#    yaw=0.0,
#    roll=1.0

# Влево
#    pitch = 0.0,
#    throttle = 1.0,
#    yaw=0.0,
#    roll=-1.0

# Вверх 
#    pitch = 0.0,
#    throttle = 2.0,
#    yaw=0.0,
#    roll=0.0  

# Вокруг себя
#    pitch = 0.0,
#    throttle = 0.0,
#    yaw=1.0,
#    roll=0.0  

# Подьём при throttle = 0.6  
# Массив [0.0, 0.0, 0.0, 0.0] 

# a custom function that blocks for a moment
def task(tracker_arg, dict_):
    client = airsim.MultirotorClient()
    client.reset()
    client.confirmConnection()
    #auto_up_drone(client)
    
    # вывод информации
#    print (dir(client))
#    print (dir(client.getMagnetometerData()))
#    print (client.getBarometerData())
#    print (client.getRotorStates())
#    print (client.simGetObjectPose())
    print (dir(client.getMultirotorState().kinematics_estimated.orientation))
    _throttle = 1.0
    while True:
        if dict_["init_switch"]:
            
            # work
            pos = client.getMultirotorState().kinematics_estimated.position.to_numpy_array()
            if float(pos[-1]) > -10.0:
                client.moveByRC(rcdata = airsim.RCData(pitch = 0.0,
                                                        throttle = _throttle,
                                                        yaw=0.0,
                                                        roll=0.0,
                                                        is_initialized = True,
                                                        is_valid = True))  
            elif float(pos[-1]) < -8.0:
                _throttle = 0.8     
            else:
                client.moveByRC(rcdata = airsim.RCData(pitch = 0.0,
                                                        throttle = _throttle,
                                                        yaw=1.0,
                                                        roll=1.0,
                                                        is_initialized = True,
                                                        is_valid = True)) 
                 
#            roll, pitch, yaw, = percent_to_control(10, 20)[:3]  
#            client.moveByRC(rcdata = airsim.RCData(pitch = 0,
#                                                   throttle = 0,
#                                                   yaw=1,
#                                                   roll=1,
#                                                   is_initialized = True,
#                                                   is_valid = True))                                             
            text = client.getMultirotorState().kinematics_estimated.orientation                
#            print ("POSITION",  euler_from_quaternion(text))#pos, client.getRotorStates())
#            print (percent_to_control(20, 10))
            #print (quaternion_to_euler_angle_vectorized(text.w_val, text.x_val, text.y_val, text.z_val))
    
def task2(tracker_arg, dict_):
        client = airsim.MultirotorClient()
        client.reset()
        client.confirmConnection()
        
        
        
        
        pos_controller = PID_Controller(Kp_pos, Kd_pos, Ki_pos, Ki_sat_pos, dt)
        angle_controller = PID_Controller(Kp_ang, Kd_ang, Ki_ang, Ki_sat_ang, dt))
        
        
        
        lib_start.cap = cv2.VideoCapture(0)
        # ROI in video
        cv2.namedWindow('win')#, cv2.WINDOW_NORMAL)   
        lib_start.tracker = cv2.TrackerCSRT_create()
        # Register the mouse callback
        cv2.setMouseCallback('win', lib_start.on_mouse)
        while lib_start.cap.isOpened():
            # FPS варианты
            start_time = time.time()
            timer = cv2.getTickCount()
            success, img = lib_start.cap.read()
            img = lib_start.increase_brightness(img)
            #img = cv2.flip(img, 1)
            img_center = lib_start.get_center(img, 0, 0, img.shape[1], img.shape[0])
            
            if lib_start.state > 1:
                cv2.rectangle(img, lib_start.p1, lib_start.p2, (255, 0, 0), 10)  
                bbox = (lib_start.p1[0], lib_start.p1[1], lib_start.p2[0]-lib_start.p1[0], lib_start.p2[1]-lib_start.p1[1])
                lib_start.tracker.init(img, bbox) 
                lib_start.init_switch = True
                lib_start.state = 0
                tracker_arg.value += 1
                dict_["init_switch"] = True
                
            if lib_start.init_switch:
                success, bbox = lib_start.tracker.update(img)
                
                #print (".........",lib_start.init_switch, tracker_arg.value)
                if success:
                    # центр обьекта
                    obj_center = lib_start.draw_box(img, bbox)
                    cv2.line(img, img_center, obj_center, (255,0,0), 4) 
                    
                    # расстояние от цента обьекта до центра экрана  
                    x_dist = (obj_center[0] - img_center[0])**2
                    y_dist = (obj_center[1] - img_center[1])**2 
#                    cv2.putText(img, "{}".format(int(np.sqrt(x_dist + y_dist))), (bbox[0],bbox[1]),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                 
                    #img_center
                    w_sector, h_sector = img.shape[1]//2, img.shape[0]//2
                    M = int(np.sqrt(w_sector**2 + h_sector**2))
                    g = int(np.sqrt(x_dist + y_dist))
                    p_dist = (g/M) * 100
                    
                    # левая гипотенуза
                    M1 = int(np.sqrt(img.shape[1]**2 + img.shape[0]**2))
                    g1_point = int(np.sqrt(bbox[0]**2 + bbox[1]**2))
                    p_dist_point = (g1_point/M1) * 100
                    cv2.line(img, (0,0), (bbox[0], bbox[1]), (255,0,0), 4)
                    cv2.putText(img, f"{int(p_dist_point)}%", (bbox[0]//2,bbox[1]//2+10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
#                    
                    # площадь в процентах
                    S0=img.shape[1]*bbox[0]
                    S1=bbox[2]*bbox[3]
                    P_s = (S1/S0)*100
                    
                    # правая гипотенуза 
                    cv2.line(img, (bbox[0]+bbox[2], bbox[1]+bbox[3]), (img.shape[1], img.shape[0]), (255,0,0), 4) 
                    x_, y_ = bbox[0]+bbox[2], bbox[1]+bbox[3]
                    x_, y_ = img.shape[1]-x_, img.shape[0]-y_
                    g2_point = int(np.sqrt(x_**2 + y_**2))
                    p_dist_point2 = (g2_point/M1) * 100  
                    A = (bbox[0]+bbox[2])+x_//2 
                    B = (bbox[1]+bbox[3])+y_//2 
                    cv2.putText(img, f"{int(p_dist_point2)}%", (A, B),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                    distance = (p_dist+p_dist_point+p_dist_point2)/3
                    cv2.putText(img, f"distance: {int(p_dist)}%, sum dis: {int(distance)}%", (bbox[0],bbox[1]),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

                   #PID контроллер
                   
#                    client.moveByRC(rcdata = airsim.RCData(pitch = 0.0,
#                                        throttle = 2.0,
#                                        yaw=0.0,
#                                        roll=-5.0,
#                                        is_initialized = True,
#                                        is_valid = True))  
        
                    
                    
            # FPS варианты
            #fps = cv2.getTickFrequency()/(cv2.getTickCount()-timer)
            end_time = time.time()
            seconds = end_time - start_time
            fps = 1.0 / seconds
            
            
            cv2.putText(img, f"{int(fps)} fps", (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,0,255),2) #cv2.FONT_HERSHEY_COMPLEX
            
            cv2.imshow("win", img)
            

            if cv2.waitKey(1) & 0xff == ord('q'):
                break      



#<class 'airsim.types.RCData'> <RCData> {   'is_initialized': True,
#    'is_valid': True,
#    'left_z': 1.0,
#    'pitch': -0.0,
#    'right_z': -1.0,
#    'roll': 0.003999948501586914,
#    'switches': 0,
#    'throttle': 0.49949997663497925,
#    'timestamp': 0,
#    'vendor_id': 'VID_1209',
#    'yaw': 0.054000020027160645}
#        airsim.RCData({""})
        print (type(client2.getMultirotorState().rc_data), client2.getMultirotorState().rc_data.roll)
#        print(dir(airsim.RCData), airsim.RCData.yaw, airsim.RCData.is_initialized) 

if __name__ == "__main__":
    num = Value('d', 0.0)
    with Manager() as manager:
        dict_ = manager.dict()
        dict_["init_switch"] = False
        # run the thread
        thread = Process(target=task, args=(num, dict_), daemon=True)
        thread.start()
     
        thread2 = Process(target=task2, args=(num, dict_), daemon=True)
        thread2.start()
        # wait for the thread to finish
        print('Waiting for the thread...')
        thread.join()   
        thread2.join()


## Connect to the AirSim simulator
#client.confirmConnection()

## Take off
#client.takeoffAsync().join()

## Set the default control parameters
#client.enableApiControl(True)
#client.armDisarm(True)

## Start object tracking
#client.startObjectTracking("object_name", 1)

#while True:
#    # Get the current position of the drone
#    drone_position = client.getPosition()

#    # Get the current tracking state
#    tracking_state = client.getObjectTrackingState()

#    # Check if the object is being tracked
#    if tracking_state[0].isTargetBeingTracked:
#        # Get the position of the tracked object
#        object_position = tracking_state[0].position

#        # Calculate the velocity vector towards the object
#        velocity_vec = airsim.Vector3r(object_position.x_val - drone_position.x_val,
#                                       object_position.y_val - drone_position.y_val,
#                                       object_position.z_val - drone_position.z_val)

#        # Normalize the velocity vector
#        velocity_vec = velocity_vec.normalized()

#        # Set the drone's velocity towards the object
#        client.moveByVelocityAsync(velocity_vec.x_val, velocity_vec.y_val, velocity_vec.z_val, 1)

#    else:
#        # Stop the drone if the object is not being tracked
#        client.moveByVelocityAsync(0, 0, 0, 1)

#    # Wait for a short period of time before repeating the process
#    airsim.time.sleep(0.1)

## Stop object tracking
#client.stopObjectTracking()

## Land the drone
#client.landAsync().join()

#target_x = bbox[0] + bbox[2] / 2
#target_y = bbox[1] + bbox[3] / 2

## Calculate drone control commands based on target position
#control_commands = calculate_drone_control(target_x, target_y)

## Move the drone
#client.moveByVelocityAsync(control_commands[0], control_commands[1], control_commands[2], 1)



#import airsim
#import cv2
#import numpy as np

## Connect to the AirSim simulator
#client = airsim.MultirotorClient()
#client.confirmConnection()

## Set up object tracking parameters
#tracker = cv2.TrackerKCF_create()
#bbox = None

## Set up PID controller parameters
#kp = 0.5
#ki = 0.1
#kd = 0.2
#prev_error = 0
#integral = 0

## Main control loop
#while True:
#    # Get image from AirSim
#    response = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
#    img1d = np.fromstring(response[0].image_data_uint8, dtype=np.uint8)
#    img_rgb = img1d.reshape(response[0].height, response[0].width, 3)

#    # Object detection using computer vision techniques
#    # ...

#    # Update object tracker
#    ok, bbox = tracker.update(img_rgb)

#    # Calculate error for PID controller
#    target_x = response[0].width / 2
#    target_y = response[0].height / 2
#    current_x = bbox[0] + bbox[2] / 2
#    current_y = bbox[1] + bbox[3] / 2
#    error_x = target_x - current_x
#    error_y = target_y - current_y

#    # Update PID controller
#    integral += error_x
#    derivative = error_x - prev_error
#    pid_output = kp * error_x + ki * integral + kd * derivative
#    prev_error = error_x

#    # Control the drone using PID output
#    pitch = pid_output
#    roll = 0
#    yaw = 0
#    throttle = 0.5
#    client.moveByEulerPID(pitch, roll, yaw, throttle, duration=0.1)

#    # Escape loop if object is lost or tracking is completed
#    if not ok:
#        break

## Disconnect from AirSim
#client.reset()
#client.enableApiControl(False)




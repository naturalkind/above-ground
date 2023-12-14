import os
import sys
import cv2
import time
import math
import airsim
import tracker_lib
import numpy as np
from numpy.linalg import norm

from pyquaternion import Quaternion

from threading import Thread
from multiprocessing import Process, Value, Array, Manager
from deap import base, creator, tools, algorithms
import math


class KalmanFilterPID:
    def __init__(self, Kp, Ki, Kd, dt):
        # PID gains
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # Sampling time
        self.dt = dt

        # Kalman filter variables
        self.x = np.zeros(2)  # State vector [position, velocity]
        self.P = np.eye(2)  # Error covariance matrix
        self.Q = np.eye(2)  # Process noise covariance matrix
        self.R = np.eye(1)  # Measurement noise covariance matrix

        # PID variables
        self.error_sum = 0.0
        self.prev_error = 0.0

    def update(self, setpoint, measurement):
        # Prediction step
        self.x[0] += self.dt * self.x[1]  # Predicted position
        self.P[0, 0] += self.dt * (self.P[1, 1] + self.P[0, 1] + self.P[1, 0] + self.Q[0, 0])  # Predicted error covariance

        # Kalman gain calculation
        K = self.P[0, 0] / (self.P[0, 0] + self.R[0, 0])

        # Update step
        self.x[0] += K * (measurement - self.x[0])  # Updated position
        self.x[1] = self.x[1] + K * (measurement - self.x[0]) / self.dt  # Updated velocity
        self.P[0, 0] -= K * self.P[0, 0]  # Updated error covariance

        # PID control calculation
        error = setpoint - self.x[0]
        self.error_sum += error * self.dt
        error_rate = (error - self.prev_error) / self.dt

        output = self.Kp * error + self.Ki * self.error_sum + self.Kd * error_rate

        # Update previous error
        self.prev_error = error

        return output


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def update(self, current_value, target_value):
        error = target_value - current_value
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return output


class DroneSim():
    def __init__(self, ip):
        self.ip = ip
#        self.client = airsim.MultirotorClient(ip=ip, port=41451) 
#        self.client.confirmConnection()
#        self.client.reset()
#        self.all_drone_scene = self.client.listVehicles()
        # Set the target height for the PID controller (in meters)
        self.target_height = -3 
        
        # opencv
        self.lib_start = tracker_lib.TrackerLib() 
        self.lib_start.tracker = cv2.TrackerCSRT_create()
    def image_task(self, tracker_arg, dict_, name_drone):
        cv2.namedWindow('win')#, cv2.WINDOW_NORMAL)  
        # Register the mouse callback
        cv2.setMouseCallback('win', self.lib_start.on_mouse)         
        client_img = airsim.MultirotorClient(ip=self.ip, port=41451) 
        client_img.confirmConnection()
        while True:
            # FPS вариант 1
            start_time = time.time()
            responses = client_img.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)], 
                                                 vehicle_name=name_drone)
            response = responses[0]
            # get numpy array
            img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8) 
            # reshape array to 4 channel image array H X W X 4
            img = img1d.reshape(response.height, response.width, 3)
            
            time.sleep(0.01)
            end_time = time.time()
            seconds = end_time - start_time
            fps = 1.0 / seconds
            cv2.putText(img, f"{int(fps)} fps", (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,0,255),2)
            cv2.imshow("win", img)
            if cv2.waitKey(1) & 0xff == ord('q'):
                break
            
    def test_pid_up(self, tracker_arg, dict_, name_drone):
        client_fly = airsim.MultirotorClient(ip=self.ip, port=41451) 
        client_fly.confirmConnection()
        client_fly.reset()
        #print (self.client.isApiControlEnabled(vehicle_name=name_drone))
        # Set the PID controller gains
        # медленный взлёт
#        Kp = 0.0645136137394194487
#        Ki = 0.00022393195314520642 
#        Kd = 6.404490165264038

        # быстрый взлёт
        Kp = 0.745136137394194487*10
        Ki = 0.00022393195314520642*10 
        Kd = 7.404490165264038*100
        
        # Create a PID controller object Pitch
        pid_controller = PIDController(Kp, Ki, Kd) # throttle

#        f_kalman = KalmanFilterPID(Kp, Ki, Kd, 0.1)

        # Start the main control loop
        while True:
            # FPS вариант 2
            timer = cv2.getTickCount()
            
            # Get the current height from the AirSim simulator
            current_height = client_fly.getMultirotorState(vehicle_name=name_drone).kinematics_estimated.position.z_val
            dict_["current_height"] = current_height
#            pid_output = f_kalman.update(self.target_height, current_height)
            # Calculate the PID output
            pid_output = pid_controller.update(current_height, self.target_height)
            dict_["throttle"] = -pid_output
            # Adjust throttle based on PID output
            client_fly.moveByRC(vehicle_name=name_drone, rcdata = airsim.RCData(#pitch = dict_["pitch"], # 5.0 max
                                                                               throttle = -pid_output,
                                                                               #yaw=0.0,
                                                                               #roll=0.0,
                                                                               is_initialized = True,
                                                                               is_valid = True))
                                                                               
            time.sleep(0.0001)
#            time.sleep(0.01)
            fps = cv2.getTickFrequency()/(cv2.getTickCount()-timer)
            print ("fps:", int(fps), -pid_output, current_height, self.target_height)
            

if __name__ == "__main__":
    Drone = DroneSim("192.168.1.100")
    #print (Drone.all_drone_scene)
    num = Value('d', 0.0)
    with Manager() as manager:
        dict_ = manager.dict()
        dict_["init_switch"] = False
        dict_["current_height"] = 0
        dict_["throttle"] = 0
        dict_["pitch"] = 0
        
        thread1 = Process(target=Drone.test_pid_up, args=(num, dict_, "BP_FlyingPawn_4"), daemon=True) 
        thread1.start() #"BP_FlyingPawn_4" 
        
        thread2 = Process(target=Drone.image_task, args=(num, dict_, "BP_FlyingPawn_4"), daemon=True)
        thread2.start()   #"BP_FlyingPawn_4" #BP_FlyingPawn2_7
        print('Waiting for the thread...')
        thread1.join()  
        thread2.join()
    
    

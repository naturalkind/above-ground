import os
import sys
import cv2
import time
import math
import airsim
import numpy as np
from numpy.linalg import norm

from pyquaternion import Quaternion

from threading import Thread
from multiprocessing import Process, Value, Array, Manager
from deap import base, creator, tools, algorithms
import math
from tracker_lib import tracker_lib
from airsim_lib_exp_ import fly_on_path

"""
Этот код создает экземпляр PID-регулятора с определенными коэффициентами Kp, Ki и Kd. 
Затем он подключается к симулятору AirSim и запускает цикл управления. 
В цикле он получает текущую высоту, вычисляет ошибку относительно целевой высоты, 
обновляет PID-регулятор и применяет получившийся управляющий сигнал к функции moveByRC

Вперёд
    pitch = 1.0,
    throttle = 1.0,
    yaw=0.0,
    roll=0.0
    
Назад
    pitch = -1.0,
    throttle = 1.0,
    yaw=0.0,
    roll=0.0        
    
Вправо
    pitch = 0.0,
    throttle = 1.0,
    yaw=0.0,
    roll=1.0

 Влево
    pitch = 0.0,
    throttle = 1.0,
    yaw=0.0,
    roll=-1.0

 Вверх 
    pitch = 0.0,
    throttle = 2.0,
    yaw=0.0,
    roll=0.0  

 Вокруг себя когда дрон в воздухе
    pitch = 0.0,
    throttle = 0.0,
    yaw=1.0,
    roll=0.0  

 Подьём при throttle = 0.6 в airsim 
 Массив [0.0, 0.0, 0.0, 0.0] 

"""


client = airsim.MultirotorClient() #ip="192.168.1.100", port = 41451
client.confirmConnection()
client.reset()

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



def convert_pos_UE_to_AS(origin_UE : np.array, pos_UE : np.array):
    z = -20
    pos = np.zeros(3, dtype=np.float)
    pos[0] = pos_UE[0] - origin_UE[0]
    pos[1] = pos_UE[1] - origin_UE[1]
    pos[2] = -1
#    pos[2] = - pos_UE[2] + origin_UE[2]
    return pos / 100


# загрузка последовательности координат spline
def load_data(file_name):
    all_data = []
    origin_UE = np.array([0.0, 0.0, 0.0]) #910.0
    with open(file_name,'r') as file:
    #with open('/media/sadko/unrealdir/AboveGenSim/Saved/CoordData/coord_scren.txt','r') as file:
        for line in file:
            coord_list = [float(i.split("=")[-1]) for i in line.split("\n")[0].split(" ")]
            coord_list = convert_pos_UE_to_AS(origin_UE, np.array(coord_list))
            all_data.append(airsim.Vector3r(coord_list[0], coord_list[1], coord_list[2]))
    print ("LOADING DATA SPLINE DONE") 
    return all_data
           
# получение изображения из airsim
def image_task(tracker_arg, dict_, name_drone):
    # Подключение к airsim
    client = airsim.MultirotorClient() #ip="192.168.1.100", port = 41451
    client.confirmConnection()

    _lib  = tracker_lib.TrackerLib()
    _lib.create_win()
        
    while True:
        # FPS варианты
        start_time = time.time()
        timer = cv2.getTickCount()
    
        #'DepthPerspective', 'DepthPlanar', 'DepthVis', 
        #'DisparityNormalized', 'Infrared', 'OpticalFlow', 
        #'OpticalFlowVis', 'Scene', 'Segmentation', 'SurfaceNormals'
        
        responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)], 
                                        vehicle_name=name_drone)
#        responses = client.simGetImages([airsim.ImageRequest("3", airsim.ImageType.Segmentation, False, False)])
        response = responses[0]
        # get numpy array
        img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8) 
        # reshape array to 4 channel image array H X W X 4
        img = img1d.reshape(response.height, response.width, 3)
        img = np.copy(img)
        _, obj_center, img_center = _lib.stream_from_ue(img)
        dict_["y_current"] = img_center[0]
        dict_["y_target"] = obj_center[0]
        dict_["init_switch"] = _lib.init_switch
        cv2.imshow("win", img)
        if cv2.waitKey(1) & 0xff == ord('q'):
            break   
    cv2.destroyAllWindows()

  
 

def test_pid_up(tracker_arg, dict_, name_drone): #
    print (name_drone)
    # Connect to the AirSim simulator
    client = airsim.MultirotorClient()#ip="192.168.1.100", port=41451
    client.confirmConnection()
#    client.reset()
#    client.enableApiControl(False, name_drone) 

    all_drone = client.listVehicles()
    # getRotorStates
    # moveByAngleThrottleAsync(pitch, roll, throttle, yaw_rate, duration, vehicle_name='')[source]
    # moveByAngleZAsync(pitch, roll, z, yaw, duration, vehicle_name='')[source]
    # moveByManualAsync
    print (all_drone, client.isApiControlEnabled(vehicle_name=name_drone))
    # Set the target height for the PID controller (in meters)
    target_height = -3 #-10

    # Set the PID controller gains
    # быстрый взлёт
    Kp = 0.745136137394194487*10
    Ki = 0.00022393195314520642*10
    Kd = 7.404490165264038*100

    # Create a PID controller object Pitch
    pid_controller = PIDController(Kp, Ki, Kd) # throttle
    
    
    Kp = 0.745136137394194487/1000
    Ki = 0#.00022393195314520642/10 
    Kd = 7.404490165264038/1000
    # Initialize PID controller for x and y axes
    pid_x = PIDController(Kp, Ki, Kd)
    pid_y = PIDController(Kp, Ki, Kd)
    
#    # Take off to a safe height
#    client.moveByRC(rcdata = airsim.RCData(pitch = 0.0,
#                                           throttle = 0.6,
#                                           yaw=0.0,
#                                           roll=0.0,
#                                           is_initialized = True,
#                                           is_valid = True)) 
#    time.sleep(1)


    # Start the main control loop
    idx = 0
    while True:
        # Get the current height from the AirSim simulator
        current_height = client.getMultirotorState(vehicle_name=name_drone).kinematics_estimated.position.z_val
        dict_["current_height"] = current_height
        # Calculate the PID output
        pid_output = pid_controller.update(current_height, target_height)
        
        if dict_["init_switch"]:
            control_signal_y = pid_y.update(dict_["y_current"], dict_["y_target"])
        else: 
            control_signal_y = 0
        
        dict_["throttle"] = -pid_output
        client.moveByRC(vehicle_name=name_drone, rcdata = airsim.RCData(#pitch = dict_["pitch"], # 5.0 max
                                                                           throttle = -pid_output,
                                                                           yaw=control_signal_y,
                                                                           #roll=0.0,
                                                                           is_initialized = True,
                                                                           is_valid = True))
        time.sleep(0.0001)
        idx += 1
        print (control_signal_y, dict_["y_current"], dict_["y_target"])


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

def quaternion_to_euler_angle_vectorized(w, x, y, z):
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
    
    
# получать данные пользоватя с пульта и литеть по этим данным  
def user_fly_data():
#    client = airsim.MultirotorClient()
#    client.confirmConnection()
#    client.enableApiControl(False, "BP_FlyingPawn_11")
    landed = client.getMultirotorState().landed_state
#    with open("ttx.txt", "w") as file:
#        while True:
#            file.write(f"{client.getMultirotorState().rc_data.pitch};{client.getMultirotorState().rc_data.roll};{client.getMultirotorState().rc_data.throttle};{client.getMultirotorState().rc_data.yaw};\n")
    
    with open("ttx.txt", "r") as file:
        D = file.readlines()            
        for i in D:
            
            t_data = i.split("\n")[0].split(";")[:-1]
            t_data = [float(x) for x in t_data]
            client.moveByRC(rcdata = airsim.RCData(pitch = t_data[0], 
                                                   throttle = t_data[2], 
                                                   yaw=t_data[3], 
                                                   roll=t_data[1], 
                                                   is_initialized = True, 
                                                   is_valid = True))
            print (t_data)

def fly_on_path(tracker_arg, dict_, all_data, name_drone):
    client.enableApiControl(True, name_drone)
    client.armDisarm(True, name_drone)
    landed = client.getMultirotorState().landed_state
    
    client.moveToZAsync(-2, 2, vehicle_name=name_drone).join()
    time.sleep(5)
#    result = client.moveOnPathAsync(all_data, 12, 120, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), 20, 1, vehicle_name=name_drone).join()
    result = client.moveOnPathAsync(all_data, 12, 12, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), 20, 1, vehicle_name=name_drone).join()
    print("flying on path coord position...", name_drone) 


if __name__ == "__main__":
    all_data = load_data('/media/sadko/unrealdir/AboveGenSim/Saved/CoordData/test.txt')
    num = Value('d', 0.0)
    with Manager() as manager:
        dict_ = manager.dict()
        dict_["init_switch"] = False
        dict_["current_height"] = 0
        dict_["throttle"] = 0
        dict_["pitch"] = 0
        dict_["y_current"] = 0 
        dict_["y_target"] = 0       
        ####################
        # Следить за обьектом
        ####################
        
        # run the thread
        thread0 = Process(target=image_task, args=(num, dict_, "BP_FlyingPawn_2"), daemon=True)
        thread0.start()
        
        thread1 = Process(target=test_pid_up, args=(num, dict_, "BP_FlyingPawn_2"), daemon=True)
        thread1.start()
        
        thread2 = Process(target=fly_on_path, args=(num, dict_, all_data, "BP_FlyingPawn2_5"), daemon=True)
        thread2.start()
        # wait for the thread to finish
        print('Waiting for the thread...')
        thread0.join() 
        thread1.join()
        thread2.join()


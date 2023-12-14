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


client = airsim.MultirotorClient(ip="192.168.1.100", port=41451) #ip="192.168.1.100", port = 41451
client.confirmConnection()
client.reset()
print ("START")
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
    client = airsim.MultirotorClient(ip="192.168.1.100", port=41451) #ip="192.168.1.100", port = 41451
    client.confirmConnection()

    Kp = 0.745136137394194487
    Ki = 0.00022393195314520642 
    Kd = 7.404490165264038
    
    # Initialize PID controller for x and y axes
    pid_x = PIDController(Kp, Ki, Kd)
    pid_y = PIDController(Kp, Ki, Kd)

    lib_start = tracker_lib.TrackerLib() 
    lib_start.tracker = cv2.TrackerCSRT_create()
    cv2.namedWindow('win')#, cv2.WINDOW_NORMAL)  
    # Register the mouse callback
    cv2.setMouseCallback('win', lib_start.on_mouse)
        
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
        img_center = lib_start.get_center(img, 0, 0, img.shape[1], img.shape[0])
        
        #field_angle = calculate_field_angle(img.shape[1], img.shape[0]) # 90*
        #print (field_angle)
        if lib_start.state > 1:
            cv2.rectangle(img, lib_start.p1, lib_start.p2, (255, 0, 0), 10)  
            bbox = (lib_start.p1[0], 
                    lib_start.p1[1], 
                    lib_start.p2[0]-lib_start.p1[0], 
                    lib_start.p2[1]-lib_start.p1[1])
                    
            lib_start.tracker.init(img, bbox) 
            lib_start.init_switch = True
            lib_start.state = 0
            tracker_arg.value += 1
            dict_["init_switch"] = True
            
        if lib_start.init_switch:
            success, bbox = lib_start.tracker.update(img)
            if success:
                # центр обьекта
                obj_center = lib_start.draw_box(img, bbox)
                cv2.line(img, img_center, obj_center, (255,0,0), 4) 
                
                # расстояние от цента обьекта до центра экрана  
                x_dist = (obj_center[0] - img_center[0])**2
                y_dist = (obj_center[1] - img_center[1])**2 
             
                #img_center
                w_sector, h_sector = img.shape[1]//2, img.shape[0]//2
                M = int(np.sqrt(w_sector**2 + h_sector**2))
                g = int(np.sqrt(x_dist + y_dist))
                p_dist = (g/M) * 100
                
                cv2.putText(img, f"distance: {int(p_dist)}%", (bbox[0],bbox[1]-10),cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                
                
                # левая гипотенуза
#                M1 = int(np.sqrt(img.shape[1]**2 + img.shape[0]**2))
#                g1_point = int(np.sqrt(bbox[0]**2 + bbox[1]**2))
#                p_dist_point = (g1_point/M1) * 100
#                cv2.line(img, (0,0), (bbox[0], bbox[1]), (255,0,0), 4)
#                cv2.putText(img, f"{int(p_dist_point)}%", (bbox[0]//2,bbox[1]//2+10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                
                # правая гипотенуза 
#                cv2.line(img, (bbox[0]+bbox[2], bbox[1]+bbox[3]), (img.shape[1], img.shape[0]), (255,0,0), 4) 
#                x_, y_ = bbox[0]+bbox[2], bbox[1]+bbox[3]
#                x_, y_ = img.shape[1]-x_, img.shape[0]-y_
#                g2_point = int(np.sqrt(x_**2 + y_**2))
#                p_dist_point2 = (g2_point/M1) * 100  
#                A = (bbox[0]+bbox[2])+x_//2 
#                B = (bbox[1]+bbox[3])+y_//2 
#                cv2.putText(img, f"{int(p_dist_point2)}%", (A, B),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)                

               #PID контроллер ротация roll
                #angle_to_obj = calculate_angle(img.shape[0], obj_center[0], img_center[0])
                quadcopter_rotation = client.getMultirotorState(vehicle_name=name_drone
                                                                ).kinematics_estimated.orientation 
                                                                    
                roll, pitch, yaw = euler_from_quaternion(quadcopter_rotation)          
                
                X, Y, Z = quaternion_to_euler_angle_vectorized(quadcopter_rotation.w_val, quadcopter_rotation.x_val, quadcopter_rotation.y_val, quadcopter_rotation.z_val)
                
                # Calculate error for x and y axes
                error_x = img_center[0] - obj_center[0] # yaw

                
                w = client.getMultirotorState().kinematics_estimated.position.x_val #vehicle_name=name_drone
                error_w = img_center[0] - obj_center[0]  
                # Calculate control signals using PID controller
#                control_signal_x = pid_x.update(Z, angle_to_obj)
                control_signal_x = pid_x.update(img_center[0], obj_center[0]) # yaw
                control_signal_w = pid_y.update(img_center[0], obj_center[0]) # roll
                
#                if img_center[0] > obj_center[0]:
#                    control_signal_w = control_signal_w
#                else:
#                    control_signal_w = -int(control_signal_w)
                # Adjust throttle based on PID output
                # поворот на месте
                client.moveByRC(rcdata = airsim.RCData(yaw=control_signal_x,
                                                       is_initialized = True,
                                                       is_valid = True), 
                                                       vehicle_name=name_drone) #roll=control_signal_w,
                
#                client.moveByRC(rcdata = airsim.RCData(roll=control_signal_x,
#                                                       is_initialized = True,
#                                                       is_valid = True),
#                                                       vehicle_name=name_drone)

#                client.moveByRC(rcdata = airsim.RCData(pitch = 200.0,
#                                                       throttle = 100.0,
#                                                       yaw=control_signal_x,
#                                                       #roll=0.0,
#                                                       is_initialized = True,
#                                                       is_valid = True)) 



                print (X, Y, Z, 
                      dict_["throttle"], 
                      dict_["pitch"], 
                      control_signal_x, 
                      control_signal_w, 
                      w, 
                      obj_center[0])
                time.sleep(0.01)  # Delay for stability                        
        # FPS варианты
        #fps = cv2.getTickFrequency()/(cv2.getTickCount()-timer)
        end_time = time.time()
        seconds = end_time - start_time
        fps = 1.0 / seconds
        cv2.putText(img, f"{int(fps)} fps", (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,0,255),2) #cv2.FONT_HERSHEY_COMPLEX
        cv2.imshow("win", img)
        if cv2.waitKey(1) & 0xff == ord('q'):
            break      

def test_pid_up(tracker_arg, dict_, name_drone): #
    print (name_drone)
    # Connect to the AirSim simulator
    client = airsim.MultirotorClient(ip="192.168.1.100", port=41451)
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

    # медленный взлёт
    Kp = 0.0645136137394194487
    Ki = 0.00022393195314520642 
    Kd = 6.404490165264038

    # быстрый взлёт
#    Kp = 0.745136137394194487*10
#    Ki = 0.00022393195314520642*10
#    Kd = 7.404490165264038*100

    # Create a PID controller object Pitch
    pid_controller = PIDController(Kp, Ki, Kd) # throttle
    
#    # Take off to a safe height
#    client.moveByRC(rcdata = airsim.RCData(pitch = 0.0,
#                                           throttle = 0.6,
#                                           yaw=0.0,
#                                           roll=0.0,
#                                           is_initialized = True,
#                                           is_valid = True)) 
#    time.sleep(1)

#    f_kalman = KalmanFilterPID(Kp, Ki, Kd, 0.1)

    # Start the main control loop
    idx = 0
    while True:
        # Get the current height from the AirSim simulator
        current_height = client.getMultirotorState(vehicle_name=name_drone).kinematics_estimated.position.z_val
        dict_["current_height"] = current_height
    #    pid_output = f_kalman.update(target_height, current_height)
        # Calculate the PID output
        pid_output = pid_controller.update(current_height, target_height)
        dict_["throttle"] = -pid_output
#        if idx > 300:
#            if -current_height > 5.5:
#                dict_["pitch"] = 2.0
#            else:
#                dict_["pitch"] = 1.0
        # Adjust throttle based on PID output
        client.moveByRC(vehicle_name=name_drone, rcdata = airsim.RCData(#pitch = dict_["pitch"], # 5.0 max
                                                                           throttle = -pid_output,
                                                                           #yaw=0.0,
                                                                           #roll=0.0,
                                                                           is_initialized = True,
                                                                           is_valid = True))
        time.sleep(0.0001)
        #time.sleep(0.01)
        print (-pid_output, current_height, target_height)
        idx += 1
        
"""
В данном примере переменные image_width и image_height представляют ширину и высоту изображения соответственно.

Функция calculate_field_angle проверяет, что ширина и высота больше 0, иначе возвращает None. 
Затем вычисляется гипотенуза изображения с использованием теоремы Пифагора. 
После этого вычисляется угол поля изображения с помощью теоремы косинусов. Результат возвращается в градусах.
"""
def calculate_field_angle(width, height):
    # Проверяем, что ширина и высота больше 0
    if width <= 0 or height <= 0:
        return None
    
    # Вычисляем гипотенузу изображения
    hypotenuse = math.sqrt(width**2 + height**2)
    
    # Вычисляем угол поля, используя теорему косинусов
    field_angle = math.degrees(math.acos(width / hypotenuse))
    
    return field_angle


#def calculate_angle(image_point, image_width, image_height):
#    # Координаты центра изображения
#    image_center_x = image_width / 2
#    image_center_y = image_height / 2
#    
#    # Координаты точки на изображении
#    image_point_x, image_point_y = image_point
#    
#    # Вычисление угла между точкой и углом поля изображения
#    delta_x = image_point_x - image_center_x
#    delta_y = image_point_y - image_center_y
#    angle_rad = math.atan2(delta_y, delta_x)
#    angle_deg = math.degrees(angle_rad)
#    
#    return angle_deg


"""
В этом примере мы используем тангенс угла, чтобы 
вычислить угол точки на изображении относительно угла поля изображения. 
Затем мы нормализуем этот угол от 0 до 360 градусов и переводим его в 
градусы относительно длины изображения, используя коэффициент 100/field_length.
"""

#def calculate_angle(image_width, point_x, point_y, field_angle):
##    field_angle = 90  # Угол поля изображения (в градусах)
#    field_length = math.sqrt(point_x**2 + point_y**2)  # Длина поля изображения

#    point_angle = math.degrees(math.atan(point_y / point_x))  # Угол точки на изображении (в градусах)
#    normalized_angle = (point_angle - field_angle) % 360  # Угол между точкой и углом поля (в градусах), нормализованный от 0 до 360

#    # Переводим угол в градусах относительно длины изображения
#    normalized_length_angle = normalized_angle * 100 / field_length

#    return normalized_length_angle


def calculate_angle(image_width, point_x, image_center_x):
    # Вычисляем расстояние от точки до центра изображения
    distance = abs(point_x - image_center_x)
    
    # Вычисляем угол относительно длины изображения
    angle = math.degrees(math.atan(distance / image_width))
    
    # Если точка находится слева от центра изображения,
    # угол будет положительным, если справа - отрицательным.
    if point_x < image_center_x:
        angle *= -1
    
    return angle


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


def from_fly_path(tracker_arg, dict_, name_drone, name_drone_target):

    Kp = 0.745136137394194487*20
    Ki = 0.00022393195314520642*20
    Kd = 7.404490165264038*60
  
    # Создание подключения к AirSim
    client = airsim.MultirotorClient(ip="192.168.1.100", port=41451)

    # Подключение к симулятору AirSim
    client.confirmConnection()

    # Выравнивание перед движением
    #client.enableApiControl(True)
    #client.armDisarm(True)
    #client.takeoffAsync().join()
    
    # Create a PID controller object throttle
    pid_controller = PIDController(Kp, Ki, Kd) # throttle
    pid_x = PIDController(Kp, Ki, Kd) 
    pid_y = PIDController(Kp, Ki, Kd)
    
    # Initialize PID controller for x and y axes
    pid_pitch = PIDController(Kp, Ki, Kd)
    pid_roll = PIDController(Kp, Ki, Kd)
    pid_yaw = PIDController(Kp, Ki, Kd)
    
#    f_kalman_x = KalmanFilterPID(Kp, Ki, Kd, 0.1)
#    f_kalman_y = KalmanFilterPID(Kp, Ki, Kd, 0.1)
    # Основной цикл движения
    while True:
        # Получение текущего положения quadrocopter'а
#        "BP_FlyingPawn_11"
        quadrocopter_rotation = client.getMultirotorState(vehicle_name=name_drone).kinematics_estimated.orientation         
        quadrocopter_position = client.getMultirotorState(vehicle_name=name_drone).kinematics_estimated.position
        #current_height = quadrocopter_position.z_val
        # Координаты объекта, за которым нужно двигаться
#        "BP_FlyingPawn2_2"
        target_rotation = client.getMultirotorState(vehicle_name=name_drone_target).kinematics_estimated.orientation
        target_position = client.getMultirotorState(vehicle_name=name_drone_target).kinematics_estimated.position
        #target_position = client.simGetObjectPose("BP_FlyingPawn2_2").position
        #target_height = target_position.z_val#-3
                                                            
        roll_t, pitch_t, yaw_t = euler_from_quaternion(target_rotation)   
        roll_q, pitch_q, yaw_q = euler_from_quaternion(quadrocopter_rotation)       
#        print (roll_t, pitch_t, yaw_t)
#        print (roll_q, pitch_q, yaw_q)
#        roll_t, pitch_t, yaw_t = quaternion_to_euler_angle_vectorized(target_rotation.w_val,
#                                                                      target_rotation.x_val, 
#                                                                      target_rotation.y_val, 
#                                                                      target_rotation.z_val)
#        roll_q, pitch_q, yaw_q = quaternion_to_euler_angle_vectorized(quadrocopter_rotation.w_val,
#                                                                      quadrocopter_rotation.x_val, 
#                                                                      quadrocopter_rotation.y_val, 
#                                                                      quadrocopter_rotation.z_val)



        ####################################
        ### Вращение !!!
        ####################################
        #current_value, target_value
        #control_signal_x = pid_x.update(target_position.x_val, quadrocopter_position.x_val) # yaw
        control_signal_pitch = pid_pitch.update(pitch_q, pitch_t)  # pitch 
        control_signal_roll = pid_roll.update(roll_q, roll_t) # roll  
        control_signal_yaw = pid_yaw.update(yaw_q, yaw_t) # yaw  
        
        
        
        ####################################
        ### Высота !!!
        ####################################

        #print (current_height, target_height, pid_output)
        control_signal_x = pid_x.update(quadrocopter_position.x_val, target_position.x_val) # yaw
        control_signal_y = pid_y.update(quadrocopter_position.y_val, target_position.y_val) # roll
        pid_output = pid_controller.update(quadrocopter_position.z_val, target_position.z_val)
#        control_signal_x = f_kalman_x.update(target_position.x_val, quadrocopter_position.x_val) # yaw
#        control_signal_y = f_kalman_y.update(target_position.y_val, quadrocopter_position.y_val) # roll
        
        ####################################
        
        
        client.moveByRC(vehicle_name=name_drone, rcdata = airsim.RCData(pitch = control_signal_x, # наклон
                                                                        throttle = -pid_output, # тяга
                                                                        yaw=control_signal_yaw, # поворот на месте
                                                                        roll=control_signal_roll, # рысканье
                                                                        is_initialized = True,
                                                                        is_valid = True))
        time.sleep(0.0001) 
        # Влияет время задержки и сигналы



if __name__ == "__main__":
    all_data = load_data('/media/sadko/unrealdir/AboveGenSim/Saved/CoordData/test.txt')
    num = Value('d', 0.0)
    with Manager() as manager:
        dict_ = manager.dict()
        dict_["init_switch"] = False
        dict_["current_height"] = 0
        dict_["throttle"] = 0
        dict_["pitch"] = 0
        
        ####################
        # Следить за обьектом
        ####################
        
#        # run the thread
#        thread0 = Process(target=image_task, args=(num, dict_, "BP_FlyingPawn_4"), daemon=True)
#        thread0.start()# "BP_FlyingPawn_11"
#        
#        thread1 = Process(target=test_pid_up, args=(num, dict_, "BP_FlyingPawn_4"), daemon=True)
#        thread1.start()# "BP_FlyingPawn2_2"
#        
#        # wait for the thread to finish
#        print('Waiting for the thread...')
#        thread0.join() 
#        thread1.join()


        ####################
        # Полёт за дроном
        ####################
        
        # run the thread
        thread1 = Process(target=from_fly_path, args=(num, dict_, "BP_FlyingPawn_4", "BP_FlyingPawn2_7"), daemon=True) 
        thread1.start()   # "BP_FlyingPawn_11", "BP_FlyingPawn2_2"          
        thread2 = Process(target=fly_on_path, args=(num, dict_, all_data, "BP_FlyingPawn2_7"), daemon=True)
        thread2.start() #"BP_FlyingPawn2_2"
        # wait for the thread to finish
        print('Waiting for the thread...')
        thread1.join()  
        thread2.join()





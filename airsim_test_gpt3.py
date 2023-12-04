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
Этот код создает экземпляр PID-регулятора с определенными коэффициентами Kp, Ki и Kd. Затем он подключается к симулятору AirSim и запускает цикл управления. В цикле он получает текущую высоту, вычисляет ошибку относительно целевой высоты, обновляет PID-регулятор и применяет получившийся управляющий сигнал к функции moveByRC

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


def track_point(target_x, target_y):
    # Подключение к airsim
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)

    # Настройка PID регулятора
    pid_controller = PIDController()

    while True:
        # Получение текущего положения дрона
        current_x, current_y = get_current_position(client)

        # Вычисление ошибки между текущим и целевым положением
        error_x = target_x - current_x
        error_y = target_y - current_y

        # Вычисление управляющего сигнала с использованием PID регулятора
        control_signal = pid_controller.calculate(error_x, error_y)

        # Применение управляющего сигнала к дрону
        client.moveByVelocity(control_signal[0], control_signal[1], control_signal[2], 1)

def get_current_position(client):
    # Получение текущего положения дрона из airsim
    position = client.getPosition()
    current_x = position.x_val
    current_y = position.y_val

    return current_x, current_y

def test_pid_up(tracker_arg, dict_):
    # Connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.reset()

    # Set the target height for the PID controller (in meters)
    target_height = -10

    # Set the PID controller gains

    # 25 сек
    #Kp = 0.06
    #Ki = 0.0001 
    #Kd = 7.0


    ## 10 сек
    #Kp = 0.06
    #Ki = 0.0003 
    #Kd = 9.0

    Kp = 0.045136137394194487
    Ki = 0.00022393195314520642 
    Kd = 6.404490165264038

    # Create a PID controller object Pitch
    pid_controller = PIDController(Kp, Ki, Kd) # throttle
    
    # Take off to a safe height
    client.moveByRC(rcdata = airsim.RCData(pitch = 0.0,
                        throttle = 0.6,
                        yaw=0.0,
                        roll=0.0,
                        is_initialized = True,
                        is_valid = True)) 
    time.sleep(1)


    start_time = time.time()
    f_kalman = KalmanFilterPID(Kp, Ki, Kd, 0.1)

    # Start the main control loop
    while True:
        # Get the current height from the AirSim simulator
        current_height = client.getMultirotorState().kinematics_estimated.position.z_val
        dict_["current_height"] = current_height
    #    pid_output = f_kalman.update(target_height, current_height)
        # Calculate the PID output
        pid_output = pid_controller.update(current_height, target_height)

        # Adjust throttle based on PID output
        client.moveByRC(rcdata = airsim.RCData(pitch = 0.0,
                                               throttle = -pid_output,
                                               yaw=0.0,
                                               roll=0.0,
                                               is_initialized = True,
                                               is_valid = True))
        
        time.sleep(0.01)
#        if current_height < 0.04:
#            tracker_arg.value += 1
#            dict_["init_switch"] = True
#            print (-pid_output, current_height, "error:", pid_controller.prev_error, "time:", time.time()-start_time)


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
    
    
def main(tracker_arg, dict_):
    # airsim
    client = airsim.MultirotorClient()
    client.reset()
    client.confirmConnection()
    # Set PID controller gains
    Kp = 0.045136137394194487
    Ki = 0.00022393195314520642 
    Kd = 6.404490165264038

    # Set target position on the screen
#    target_position = (client.getMultirotorState().kinematics_estimated.position.x_val+2.0, 
#                       client.getMultirotorState().kinematics_estimated.position.y_val-2.0)
    
    # Initialize PID controller for x and y axes
    pid_x = PIDController(Kp, Ki, Kd)
    pid_y = PIDController(Kp, Ki, Kd)
        
    # opencv
    lib_start = tracker_lib.TrackerLib()
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
        
        field_angle = calculate_field_angle(img.shape[1], img.shape[0])
        
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

               #PID контроллер ротация roll
                if -(dict_["current_height"]) > 7.0:
                    angle_to_obj = calculate_angle(img.shape[1], obj_center[0], img_center[0])
                    quadcopter_rotation = client.getMultirotorState().kinematics_estimated.orientation     
                    roll, pitch, yaw = euler_from_quaternion(quadcopter_rotation)          
                    
                    X, Y, Z = quaternion_to_euler_angle_vectorized(quadcopter_rotation.w_val, quadcopter_rotation.x_val, quadcopter_rotation.y_val, quadcopter_rotation.z_val)
                    
                    
                    # Calculate error for x and y axes
                    error_x = Z - angle_to_obj

                    # Calculate control signals using PID controller
                    control_signal_x = pid_x.update(Z, angle_to_obj)
#                    control_signal_y = pid_y.update(quadcopter_position.y_val, target_position[1])
                    
                    # Adjust throttle based on PID output
                    client.moveByRC(rcdata = airsim.RCData(yaw=control_signal_x,
                                                           is_initialized = True,
                                                           is_valid = True))

#                    print (control_signal_x, f"error Z {error_x}", Z,  angle_to_obj)
                    print (f"Вращение по Z: {Z},  угол к обьекту: {angle_to_obj}")
#                    print ("------------>", target_position, quadcopter_position.x_val, quadcopter_position.y_val)
#                    angle_point = calculate_angle(obj_center, img.shape[1], img.shape[0])

#                    angle = calculate_angle(img.shape[1], obj_center[0], obj_center[1], field_angle)

                    
                    
#                     Влево
#                        pitch = 0.0,
#                        throttle = 1.0,
#                        yaw=0.0,
#                        roll=-1.0                    
#                    print (field_angle, angle_to_obj, img.shape[1], img.shape[0])
                    
                    
                    
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




        

if __name__ == "__main__":
    num = Value('d', 0.0)
    with Manager() as manager:
        dict_ = manager.dict()
        dict_["init_switch"] = False
        dict_["current_height"] = 0
        # run the thread
        thread = Process(target=test_pid_up, args=(num, dict_), daemon=True)
        thread.start()
     
        thread2 = Process(target=main, args=(num, dict_), daemon=True)
        thread2.start()
        # wait for the thread to finish
        print('Waiting for the thread...')
        thread.join()   
        thread2.join()


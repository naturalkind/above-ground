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

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation


"""
Этот код создает экземпляр PID-регулятора с определенными коэффициентами Kp, Ki и Kd. 
Затем он подключается к симулятору AirSim и запускает цикл управления. 
В цикле он получает текущую позичу, вычисляет ошибку относительно целевой позиции обьекта, 
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


class TestPid:
    # Инициализация PID-регулятора
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0
        self.total_error = 0
        self.last_time = time.time()
        
    # Вычисление управляющего сигнала с помощью PID-регулятора
    def update(self, error):
        # Вычисление времени прошедшего с момента последнего обновления
        current_time = time.time()
        dt = current_time - self.last_time
        
        # Вычисление ошибки изменения положения
        delta_error = error - self.last_error
        
        # Обновление последнего времени и ошибки
        self.last_time = current_time
        self.last_error = error
        
        # Вычисление компонентов PID-регулятора
        p_term = self.kp * error
        i_term = self.ki * self.total_error
        d_term = self.kd * (delta_error / dt)
        
        # Вычисление управляющего сигнала
        control = p_term + i_term + d_term
        
        # Обновление суммарной ошибки
        self.total_error += error * dt
        
        return control

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
    

def fly_on_path(tracker_arg, dict_, all_data, name_drone):
    client.enableApiControl(True, name_drone)
    client.armDisarm(True, name_drone)
    landed = client.getMultirotorState().landed_state
    
    client.moveToZAsync(-2, 2, vehicle_name=name_drone).join()
    time.sleep(5)
#    result = client.moveOnPathAsync(all_data, 12, 120, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), 20, 1, vehicle_name=name_drone).join()
    result = client.moveOnPathAsync(all_data, 12, 12, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), 20, 1, vehicle_name=name_drone).join()
    print("flying on path coord position...", name_drone) 

def calc_angle(drone_yaw, object_yaw):
    angle = object_yaw - drone_yaw
    if angle > math.pi:
        angle -= 2 * math.pi
    elif angle < -math.pi:
        angle += 2 * math.pi
    return angle


def calculate_angle(drone_position, target_position):
    # Рассчитываем разницу в позициях между дроном и целевым объектом
    dx = target_position.x_val - drone_position.x_val
    dy = target_position.y_val - drone_position.y_val
    
    # Рассчитываем угол между двумя точками
    angle = math.atan2(dy, dx) * (180 / math.pi)
    
    # Приводим угол к диапазону от -180 до 180 градусов
    while angle <= -180:
        angle += 360
    while angle > 180:
        angle -= 360
    
    return angle

def yaw_control(pid, target_pos, object_pos):
    # Вычисляем разницу в позиции между целевой точкой и объектом
    diff_pos = target_pos - object_pos
    
    # Вычисляем ошибку угла поворота (yaw)
    yaw_error = math.atan2(diff_pos.y_val, diff_pos.x_val)
    print ("YAW ERROR", yaw_error)
    # Применяем ПИД-регулятор к ошибке угла поворота
    yaw_output = pid.update(yaw_error)
    
    # Возвращаем значение угла поворота
    return yaw_output


def from_fly_path(tracker_arg, dict_, name_drone, name_drone_target):

    # быстрый взлёт
    Kp = 0.745136137394194487*10
    Ki = 0.00022393195314520642*10
    Kd = 7.404490165264038*100

    # Создание подключения к AirSim
    client = airsim.MultirotorClient()#ip="192.168.1.100", port=41451

    # Подключение к симулятору AirSim
    client.confirmConnection()

    # Create a PID controller object throttle
#    pid_x = PIDController(Kp, Ki, Kd) 
#    pid_y = PIDController(Kp, Ki, Kd)
    pid_z = PIDController(Kp, Ki, Kd) # throttle
    
    # медленный взлёт
    Kp = 0.000645136137394194487
    Ki = 0.0000012393195314520642 
    Kd = 0.404490165264038  
    pid_x = PIDController(Kp, Ki, Kd)
    pid_y = PIDController(Kp, Ki, Kd) 
    # Initialize PID controller for x and y axes
    pid_pitch = PIDController(Kp, Ki, Kd)
    pid_roll = PIDController(Kp, Ki, Kd)

    pid_yaw = TestPid(Kp, Ki, Kd)
#    pid_yaw = PIDController(Kp, Ki, Kd)
    
    ix = 0
    # Основной цикл движения
    while True:

        # Целоевой дрон
        target_kinematics = client.simGetVehiclePose(vehicle_name=name_drone_target)
#        target_kinematics = client.simGetObjectPose(name_drone_target)
#        target_kinematics = client.getMultirotorState(vehicle_name=name_drone_target).kinematics_estimated
        
        # Основной дрон
        quad_kinematics = client.getMultirotorState(vehicle_name=name_drone).kinematics_estimated        
        
        # Получение текущего положения quadrocopter'а
        quad_rotation = quad_kinematics.orientation         
        quad_position = quad_kinematics.position
        # Координаты объекта, за которым нужно двигаться
        target_rotation = target_kinematics.orientation
        target_position = target_kinematics.position
                                                            
        roll_t, pitch_t, yaw_t = euler_from_quaternion(target_rotation)   
        roll_q, pitch_q, yaw_q = euler_from_quaternion(quad_rotation)        
        
        ####################################
        ### Ускорение и скорость!!!
        #################################### 
        
        
#        target_angular_velocity = target_kinematics.angular_velocity
#        target_angular_acceleration = target_kinematics.angular_acceleration
#        
#        target_linear_velocity = target_kinematics.linear_velocity
#        target_linear_acceleration = target_kinematics.linear_acceleration
#        

#        quad_angular_velocity = quad_kinematics.angular_velocity
#        quad_angular_acceleration = quad_kinematics.angular_acceleration
#        
#        quad_linear_velocity = quad_kinematics.linear_velocity
#        quad_linear_acceleration = quad_kinematics.linear_acceleration
        
        
        
        ####################################
        ### Вращение !!!
        ####################################
        #current_value, target_value
        control_signal_pitch = pid_pitch.update(pitch_q, pitch_t)  # pitch 
        control_signal_roll = pid_roll.update(roll_q, roll_t) # roll  
#        control_signal_yaw = pid_yaw.update(yaw_q, yaw_t) # yaw  
        
        
        ####################################
        ### Расположение !!!
        ####################################

        control_signal_x = pid_x.update(quad_position.x_val, target_position.x_val) # yaw 
        control_signal_y = pid_y.update(quad_position.y_val, target_position.y_val) # roll
        control_signal_z = pid_z.update(quad_position.z_val, target_position.z_val+(-1.0))
#        control_signal_x = f_kalman_x.update(target_position.x_val, quadrocopter_position.x_val) # yaw
#        control_signal_y = f_kalman_y.update(target_position.y_val, quadrocopter_position.y_val) # roll
        
        
        # Ошибка ориентации
        error_yaw = yaw_t - yaw_q
        error_roll = roll_t - roll_q
        error_pitch = pitch_t - pitch_q
        
        # Ошибка позиции
        error_x = target_position.x_val - quad_position.x_val
        error_y = target_position.y_val - quad_position.y_val
        error_z = target_position.z_val - quad_position.z_val
       
        ########################
        ## Место экспериментов
        ########################
        drone_yaw = airsim.to_eularian_angles(quad_rotation)[0]
        target_object_yaw = airsim.to_eularian_angles(target_rotation)[0]       
#        # Вычисляем угол между текущей ориентацией дрона и ориентацией целевого объекта
#        angle = calc_angle(drone_yaw, target_object_yaw)
        # Вычисляем угол между ориентацией дрона и целевым объектом
        angle1 = calculate_angle(quad_position, target_position)
               
        control_signal_yaw = yaw_control(pid_yaw, target_position, quad_position) 
#        control_signal_yaw = pid_yaw.update(angle1) # yaw 
       
        #t_test = client.simGetFocusDistance("BP_PIPCamera_C_510", vehicle_name=name_drone)
        #data_car = client.getDistanceSensorData(vehicle_name=name_drone)
        #t_test = client.simGetObjectPose(name_drone).position
        
        
#        print (f"X ---> Основной {quad_position.x_val} Цель {target_position.x_val} Ошибка {error_x}")
#        print (f"Y ---> Основной {quad_position.y_val} Цель {target_position.y_val} Ошибка {error_y}")
#        print (f"Z ---> Основной {quad_position.z_val} Цель {target_position.z_val} Ошибка {error_z}")
        print (":::::::::::::::::::::::::::::::::::::::::::::::::::::::::::", ix)
        print ("roll", roll_q, roll_t, "ERROR", error_roll)
        print ("pitch", pitch_q, pitch_t, "ERROR", error_pitch)
        print ("yaw", yaw_q, yaw_t, "ERROR", error_yaw)
#        print ((error_yaw+error_roll+error_pitch)/3)

        ####################################
        
        if ix > 600:
            client.moveByRC(vehicle_name=name_drone, rcdata = airsim.RCData(pitch = control_signal_x, # наклон
                                                                            throttle = -control_signal_z, # тяга
                                                                            yaw=control_signal_yaw, # поворот на месте
                                                                            roll=control_signal_y, # рысканье
                                                                            is_initialized = True,
                                                                            is_valid = True))
            time.sleep(0.0001) 
        # Влияет время задержки и сигналы
        ix += 1

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
        # Полёт за дроном
        ####################
        
        # run the thread
        thread1 = Process(target=from_fly_path, args=(num, dict_, "BP_FlyingPawn_2", "BP_FlyingPawn2_5"), daemon=True) 
        thread1.start()           
        thread2 = Process(target=fly_on_path, args=(num, dict_, all_data, "BP_FlyingPawn2_5"), daemon=True)
        thread2.start() 
        # wait for the thread to finish
        print('Waiting for the thread...')
        thread1.join()  
        thread2.join()


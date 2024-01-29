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

    # Применяем ПИД-регулятор к ошибке угла поворота
    yaw_output = pid.update(yaw_error)
    
    # Возвращаем значение угла поворота
    return yaw_output


def from_fly_path(tracker_arg, dict_, name_drone, name_drone_target):


    # Создание подключения к AirSim
    client = airsim.MultirotorClient()#ip="192.168.1.100", port=41451

    # Подключение к симулятору AirSim
    client.confirmConnection()

    # быстрый взлёт
    Kp = 0.745136137394194487*10
    Ki = 0.00022393195314520642*10
    Kd = 7.404490165264038*100
    # Create a PID controller object throttle
    pid_z = PIDController(Kp, Ki, Kd) # throttle
    
    # медленный roll
    Kp = 0.15136137394194487
    Ki = 0.000022393195314520642 
    Kd = 8.404490165264038
    pid_y = PIDController(Kp, Ki, Kd) 

#    # yaw
    Kp = 0.045136137394194487
    Ki = 0.000022393195314520642 
    Kd = 1.404490165264038
    pid_yaw = TestPid(Kp, Ki, Kd)
#    pid_yaw = PIDController(Kp, Ki, Kd)
    
    # pitch
#    Kp = 0.0045136137394194487
#    Ki = 0.0012393195314520642 
#    Kd = 6.404490165264038
    Kp = 0.75136137394194487
    Ki = 0.000412393195314520642 
    Kd = 100.404490165264038
    pid_x = PIDController(Kp, Ki, Kd)

    
    ix = 0
    # Основной цикл движения
    while True:

        # Целоевой дрон
#        target_kinematics = client.simGetVehiclePose(vehicle_name=name_drone_target)
        target_kinematics = client.simGetObjectPose(name_drone_target)
#        target_kinematics = client.getMultirotorState(vehicle_name=name_drone_target).kinematics_estimated
        
        # Основной дрон
        quad_kinematics = client.getMultirotorState(vehicle_name=name_drone).kinematics_estimated        
        
        # Получение текущего положения quadrocopter'а
        quad_rotation = quad_kinematics.orientation         
        quad_position = quad_kinematics.position
        # Координаты объекта, за которым нужно двигаться
        target_rotation = target_kinematics.orientation
        target_position = target_kinematics.position

        ####################################
        ### Вращение !!!
        ####################################                  
                                                   
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
        ### Расположение !!!
        ####################################

#        control_signal_x = pid_x.update(quad_position.x_val, target_position.x_val) # yaw 
#        control_signal_y = pid_y.update(quad_position.y_val, target_position.y_val) # roll
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
               
#        control_signal_yaw = yaw_control(pid_yaw, target_position, quad_position) 
#        control_signal_yaw = pid_yaw.update(angle1) # yaw 
       
        #t_test = client.simGetFocusDistance("BP_PIPCamera_C_510", vehicle_name=name_drone)
        #data_car = client.getDistanceSensorData(vehicle_name=name_drone)
        #t_test = client.simGetObjectPose(name_drone).position
        
        
        print (f"X ---> Основной {quad_position.x_val} Цель {target_position.x_val} Ошибка {error_x}")
#        print (f"Y ---> Основной {quad_position.y_val} Цель {target_position.y_val} Ошибка {error_y}")
#        print (f"Z ---> Основной {quad_position.z_val} Цель {target_position.z_val} Ошибка {error_z}")
        print (":::::::::::::::::::::::::::::::::::::::::::::::::::::::::::", ix)
#        print ("roll", roll_q, roll_t, "ERROR", error_roll)
#        print ("pitch", pitch_q, pitch_t, "ERROR", error_pitch)
#        print ("yaw", yaw_q, yaw_t, "ERROR", error_yaw)
#        print ((error_yaw+error_roll+error_pitch)/3)

        ####################################
        
        if ix > 100:
            control_signal_x = pid_x.update(quad_position.x_val, target_position.x_val-1) # yaw target_position.x_val
            control_signal_y = pid_y.update(quad_position.y_val, target_position.y_val) # roll
            control_signal_yaw = yaw_control(pid_yaw, target_position, quad_position) # yaw 
            client.moveByRC(vehicle_name=name_drone, rcdata = airsim.RCData(pitch = control_signal_x, # наклон
                                                                            throttle = -control_signal_z, # тяга
                                                                            yaw=control_signal_yaw, # поворот на месте
                                                                            #roll=control_signal_y, # рысканье
                                                                            is_initialized = True,
                                                                            is_valid = True))
            time.sleep(0.006)
        else:
            client.moveByRC(vehicle_name=name_drone, rcdata = airsim.RCData(throttle = -control_signal_z, # тяга
                                                                            is_initialized = True,
                                                                            is_valid = True))            
            time.sleep(0.004) 
            
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
        thread1 = Process(target=from_fly_path, args=(num, dict_, "BP_FlyingPawn_2", "BP_FlyingPawn2_5"), daemon=True) #"BP_FlyingPawn_4", "BP_FlyingPawn2_7"
        thread1.start()   # "BP_FlyingPawn_11", "BP_FlyingPawn2_2"          
        thread2 = Process(target=fly_on_path, args=(num, dict_, all_data, "BP_FlyingPawn2_5"), daemon=True)
        thread2.start() #"BP_FlyingPawn2_2"#"BP_FlyingPawn2_7"
        # wait for the thread to finish
        print('Waiting for the thread...')
        thread1.join()  
        thread2.join()

"""
Настроить генетический алгоритм
для движения за дроном вперёд


import airsim
import math

# Коэффициенты PID-регулятора
kp = 1.0
ki = 0.0
kd = 0.0

# Параметры для работы PID-регулятора
prev_error = 0
integral = 0

# Конфигурация для соединения с AirSim
config = airsim.MultirotorClient.MultirotorClientConfig()
config.synchronous_mode = True

# Создание подключения к AirSim
client = airsim.MultirotorClient.MultirotorClient(config=config)

# Подключение к симулятору AirSim
client.confirmConnection()

# Выравнивание перед движением
client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync().join()

# Координаты объекта, за которым нужно двигаться
target_object = "TargetObject"
target_position = client.simGetObjectPose(target_object).position

# Основной цикл движения
while True:
    # Получение текущего положения quadrocopter'а
    quadrocopter_position = client.simGetVehiclePose().position

    # Расчет ошибки по x и y
    error_x = target_position.x_val - quadrocopter_position.x_val
    error_y = target_position.y_val - quadrocopter_position.y_val

    # Расчет P-компонента
    p_term_x = kp * error_x
    p_term_y = kp * error_y

    # Расчет I-компонента
    integral += error_x + error_y
    i_term_x = ki * integral
    i_term_y = ki * integral

    # Расчет D-компонента
    d_term_x = kd * (error_x - prev_error)
    d_term_y = kd * (error_y - prev_error)

    # Вычисление выходного значения
    output_x = p_term_x + i_term_x + d_term_x
    output_y = p_term_y + i_term_y + d_term_y

    # Ограничение выходного значения
    output_x = max(min(output_x, 1), -1)
    output_y = max(min(output_y, 1), -1)

    # Применение выходного значения на quadrocopter
    client.moveByRC(rcdata=airsim.RCData(roll=output_y, pitch=-output_x))

    # Обновление значения предыдущей ошибки
    prev_error = error_x

    # Проверка условия остановки движения
    if math.sqrt(error_x ** 2 + error_y ** 2) < 0.1:
        break

# Остановка движения и выключение quadrocopter'а
client.moveByRC(rcdata=airsim.RCData())
client.armDisarm(False)
client.enableApiControl(False)


import airsim
import time

class PidController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0
        self.error_integral = 0

    def calculate_control_signal(self, error):
        control_signal = self.kp * error + self.ki * self.error_integral + self.kd * (error - self.last_error)
        self.error_integral += error
        self.last_error = error
        return control_signal

def flight_control():
    # Подключение к AirSim
    client = airsim.MultirotorClient()
    client.confirmConnection()

    # Запуск симуляции
    client.enableApiControl(True)
    client.armDisarm(True)

    # Создание PID-регуляторов
    pitch_pid = PidController(0.5, 0, 0)
    throttle_pid = PidController(0.4, 0.01, 0)
    yaw_pid = PidController(0.2, 0, 0)
    roll_pid = PidController(0.5, 0, 0)

    # Целевая высота полета
    target_altitude = -10

    while True:
        # Получение текущей позиции и высоты
        position = client.getPosition()
        altitude = -position.z_val

        # Вычисление ошибки высоты
        altitude_error = target_altitude - altitude

        # Вычисление управляющих сигналов с использованием PID-регуляторов
        pitch_control_signal = pitch_pid.calculate_control_signal(altitude_error)
        throttle_control_signal = throttle_pid.calculate_control_signal(altitude_error)
        yaw_control_signal = yaw_pid.calculate_control_signal(0)  # Контроль направления на текущем уровне
        roll_control_signal = roll_pid.calculate_control_signal(0)  # Контроль наклона на текущем уровне

        # Отправка управляющих сигналов
        client.moveByRC(pitch_control_signal, throttle_control_signal, yaw_control_signal, roll_control_signal)

        # Добавить задержку для обновления позиции
        time.sleep(0.1)

if __name__ == "__main__":
    flight_control()


Алгоритм работы при выполнении функции самонаведения и слежения в квадрокоптерах может быть следующим:

Получение желаемой позиции или траектории наведения или слежения (например, с помощью видеоаналитики или других сенсоров).
Получение текущей позиции и ориентации квадрокоптера (например, с помощью GPS, инерциальных измерений, комплиментарного фильтра и т. д.).
Вычисление ошибки позиции и ориентации путем вычитания текущих значений из желаемых значений.
Применение контроллера для вычисления управляющих сигналов для моторов квадрокоптера.
Применение этих управляющих сигналов к моторам, чтобы изменить тягу каждого мотора и, таким образом, изменить позицию и ориентацию квадрокоптера.
Повторение шагов 2-5 для обновления позиции и ориентации до достижения желаемой точности или при необходимости.
Пример псевдокода на языке Python:

scss
Copy code
def self_stabilization(target_position, target_orientation):
    while True:
        current_position = get_current_position()
        current_orientation = get_current_orientation()
        
        position_error = target_position - current_position
        orientation_error = target_orientation - current_orientation

        control_signal = calculate_control_signal(position_error, orientation_error)
        apply_control_signal(control_signal)
В этом примере функция self_stabilization принимает желаемую позицию и ориентацию в качестве входных данных и выполняет цикл, который вычисляет ошибку позиции и ориентации, затем применяет контроллер и применяет управляющий сигнал к моторам квадрокоптера.

Вызовы функций get_current_position() и get_current_orientation() должны быть заменены на соответствующий код для получения текущей позиции и ориентации. Аналогично, функции calculate_control_signal() и apply_control_signal() должны быть заменены на соответствующий код для вычисления управляющего сигнала и его применения к моторам квадрокоптера.




def calculate_control_signal(position_error, orientation_error):
    # Коэффициенты PID регулятора
    Kp_position = 0.5
    Ki_position = 0.1
    Kd_position = 0.2

    Kp_orientation = 0.3
    Ki_orientation = 0.05
    Kd_orientation = 0.1

    # Интегральная ошибка
    integral_error_position = 0
    integral_error_orientation = 0

    # Прошлая ошибка
    previous_error_position = 0
    previous_error_orientation = 0

    # Вычисление позиционного компонента управляющего сигнала
    position_component = (
        Kp_position * position_error +
        Ki_position * integral_error_position +
        Kd_position * (position_error - previous_error_position)
    )

    # Обновление интегральной ошибки для позиционного компонента
    integral_error_position += position_error

    # Обновление прошлой ошибки для позиционного компонента
    previous_error_position = position_error

    # Вычисление ориентационного компонента управляющего сигнала
    orientation_component = (
        Kp_orientation * orientation_error +
        Ki_orientation * integral_error_orientation +
        Kd_orientation * (orientation_error - previous_error_orientation)
    )

    # Обновление интегральной ошибки для ориентационного компонента
    integral_error_orientation += orientation_error

    # Обновление прошлой ошибки для ориентационного компонента
    previous_error_orientation = orientation_error

    # Вычисление итогового управляющего сигнала
    pitch = position_component + orientation_component
    throttle = position_component - orientation_component
    roll = position_component + orientation_component

    # Возвращаем управляющий сигнал movebyrcdata
    return pitch, throttle, roll
    
Для управления PID-регулятором yaw (поворотом вокруг вертикальной оси) при использовании функции moveByRC в AirSim, следует использовать следующие параметры:

Roll: Задает угол крена дрона. Диапазон значений: от -1 до 1.
Pitch: Задает угол тангажа дрона. Диапазон значений: от -1 до 1.
Yaw: Задает угол поворота дрона вокруг вертикальной оси. Диапазон значений: от -1 до 1.
Throttle: Задает уровень газа/тяги дрона. Диапазон значений: от 0 до 1.
Эти параметры позволяют управлять дроном во всех трех осях (вперед/назад, влево/вправо, вверх/вниз) и осуществлять повороты вокруг вертикальной оси.    
    
    
    
Для управления PID-регулятором yaw moveByRC в AirSim при задаче слежения за объектом необходимо использовать следующие данные:

Координаты текущего положения дрона - это позволяет определить его текущую позицию в пространстве.
Координаты положения целевого объекта - позволяют определить положение объекта, за которым необходимо следить.
Данные об ошибке - вычисляются как разница между текущим положением дрона и положением целевого объекта. Эти данные позволяют оценить "отклонение" от желаемого положения.
Угол между текущей ориентацией дрона и целевым объектом - помогает определить направление, в котором дрон должен повернуться для слежения за объектом.
На основе этих данных PID-регулятор вычисляет управляющий сигнал, который в свою очередь используется для регулирования угла поворота дрона вокруг оси yaw. Этот угол yaw позволяет дрону выровняться с целевым объектом и поддерживать его слежение.    
"""


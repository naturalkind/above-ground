"""
В этом примере используются константы KP, KI и KD для настройки PID-регулятора. 
THROTTLE, YAW_RATE, PITCH и ROLL - это значения, описывающие желаемое управление 
квадрокоптером для каждой оси.

Функция move_quadcopter получает текущие данные RC с помощью метода 
getRCData и вычисляет ошибки управления для каждой оси. Затем, используя 
PID-регулятор, вычисляется управляющий сигнал для каждой оси и отправляется 
через метод moveByRC для установки движения квадрокоптера в AirSim.

Цикл управления повторяется бесконечно, чтобы обеспечить постоянное обновление управления. 
Это можно изменить, чтобы прекратить цикл после определенного количества шагов или по условию.

"""

#import airsim
#import time

## Константы PID-регулятора
#KP = 0.045136137394194487
#KI = 0.00022393195314520642 
#KD = 6.404490165264038

## Константы управления
#THROTTLE = 0.8 # Управление газом
#YAW_RATE = 0 # Управление рысканием (yaw)
#PITCH = 0 # Управление тангажом (pitch)
#ROLL = 0 # Управление креном (roll)

## Создание экземпляра клиента AirSim
#client = airsim.MultirotorClient()

## Подключение к AirSim
#client.confirmConnection()

## Установка RC-режима управления
#client.enableApiControl(True)
#client.armDisarm(True)

## Инициализация PID-регулятора
#def init_pid():
#    global last_error
#    global total_error
#    global last_time
#    
#    last_error = 0
#    total_error = 0
#    last_time = time.time()
#    
## Вычисление управляющего сигнала с помощью PID-регулятора
#def pid_control(error):
#    global last_error
#    global total_error
#    global last_time
#    
#    # Вычисление времени прошедшего с момента последнего обновления
#    current_time = time.time()
#    dt = current_time - last_time
#    
#    # Вычисление ошибки изменения положения
#    delta_error = error - last_error
#    
#    # Обновление последнего времени и ошибки
#    last_time = current_time
#    last_error = error
#    
#    # Вычисление компонентов PID-регулятора
#    p_term = KP * error
#    i_term = KI * total_error
#    d_term = KD * (delta_error / dt)
#    
#    # Вычисление управляющего сигнала
#    control = p_term + i_term + d_term
#    
#    # Обновление суммарной ошибки
#    total_error += error * dt
#    
#    return control

## Функция движения коптера с помощью PID-регулятора
#def move_quadcopter():
#    # Получение текущих данных RC
#    rcdata = client.getMultirotorState().rc_data
#    # Расчет ошибок управления
#    x_error = PITCH - rcdata.pitch
#    y_error = ROLL - rcdata.roll
#    z_error = YAW_RATE - rcdata.yaw
#    
#    # Вычисление управляющих сигналов с помощью PID-регуляторов
#    x_control = pid_control(x_error)
#    y_control = pid_control(y_error)
#    z_control = pid_control(z_error)
#    
#    # Отправка управляющих сигналов в AirSim
#    #client.moveByRC(rcdata.throttle, y_control, x_control, z_control)
#    client.moveByRC(rcdata = airsim.RCData(pitch = y_control,
#                                           throttle = rcdata.throttle,
#                                           yaw=z_control,
#                                           roll=x_control,
#                                           is_initialized = True,
#                                           is_valid = True))  
## Инициализация PID-регулятора
#init_pid()

## Начало цикла управления
#while True:
#    move_quadcopter()
#    time.sleep(0.1)
#    
#    
#    
#    


############################################################################
############################################################################
############################################################################
"""
Вот пример функции на языке Python, которая использует PID-регулятор 
для движения quadrocopter'ом в сторону объекта. Код написан 
для работы с библиотекой AirSim.

Обратите внимание, что данный код не предназначен для работы с реальным Quadrocopter'ом, а 
для симулятора AirSim. Также, вам потребуется установить библиотеку AirSim и 
создать объект TargetObject в симуляторе, за которым будет двигаться quadrocopter.
"""


#import airsim
#import math

## Коэффициенты PID-регулятора
#kp = 1.0
#ki = 0.0
#kd = 0.0

## Параметры для работы PID-регулятора
#prev_error = 0
#integral = 0

## Конфигурация для соединения с AirSim
#config = airsim.MultirotorClient.MultirotorClientConfig()
#config.synchronous_mode = True

## Создание подключения к AirSim
#client = airsim.MultirotorClient.MultirotorClient(config=config)

## Подключение к симулятору AirSim
#client.confirmConnection()

## Выравнивание перед движением
#client.enableApiControl(True)
#client.armDisarm(True)
#client.takeoffAsync().join()

## Координаты объекта, за которым нужно двигаться
#target_object = "TargetObject"
#target_position = client.simGetObjectPose(target_object).position

## Основной цикл движения
#while True:
#    # Получение текущего положения quadrocopter'а
#    quadrocopter_position = client.simGetVehiclePose().position

#    # Расчет ошибки по x и y
#    error_x = target_position.x_val - quadrocopter_position.x_val
#    error_y = target_position.y_val - quadrocopter_position.y_val

#    # Расчет P-компонента
#    p_term_x = kp * error_x
#    p_term_y = kp * error_y

#    # Расчет I-компонента
#    integral += error_x + error_y
#    i_term_x = ki * integral
#    i_term_y = ki * integral

#    # Расчет D-компонента
#    d_term_x = kd * (error_x - prev_error)
#    d_term_y = kd * (error_y - prev_error)

#    # Вычисление выходного значения
#    output_x = p_term_x + i_term_x + d_term_x
#    output_y = p_term_y + i_term_y + d_term_y

#    # Ограничение выходного значения
#    output_x = max(min(output_x, 1), -1)
#    output_y = max(min(output_y, 1), -1)

#    # Применение выходного значения на quadrocopter
#    client.moveByRC(rcdata=airsim.RCData(roll=output_y, pitch=-output_x))

#    # Обновление значения предыдущей ошибки
#    prev_error = error_x

#    # Проверка условия остановки движения
#    if math.sqrt(error_x ** 2 + error_y ** 2) < 0.1:
#        break

## Остановка движения и выключение quadrocopter'а
#client.moveByRC(rcdata=airsim.RCData())
#client.armDisarm(False)
#client.enableApiControl(False)



############################################################################
############################################################################
############################################################################

"""
Вышеуказанный код представляет собой общую структуру, и для его работы 
потребуется изменить некоторые детали, такие как настройки PID-контроллера, 
подключение к AirSim, запрос данных расстояния к объекту и функции управления Quadrocopter.
"""

#import airsim
#import time

#class PIDController:
#    def __init__(self, kp, ki, kd):
#        self.kp = kp
#        self.ki = ki
#        self.kd = kd
#        self.last_error = 0
#        self.integral = 0

#    def update(self, current_value, target_value):
#        error = target_value - current_value
#        self.integral += error
#        derivative = error - self.last_error
#        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
#        self.last_error = error
#        return output

## Подключение к AirSim
#client = airsim.MultirotorClient()
#client.confirmConnection()
#client.enableApiControl(True)

## Настройки PID-контроллера
#kp = 0.045136137394194487
#ki = 0.00022393195314520642 
#kd = 6.404490165264038
#pid_controller = PIDController(kp, ki, kd)

## Настройки движения
#target_distance = client.getMultirotorState().kinematics_estimated.position.x_val+4.0 # Целевое расстояние до объекта (в метрах)
#max_speed = 5 # Максимальная скорость движения (в м/с)

## Начальные значения
#current_distance = 0
#start_time = time.time()

#while current_distance < target_distance:
#    # Получение данных расстояния к цели
#    current_distance = target_distance-client.getMultirotorState().kinematics_estimated.position.x_val

#    # Вычисление PID-коррекции скорости
#    correction = pid_controller.update(current_distance, target_distance)

#    # Ограничение значения коррекции до максимальной скорости
#    correction = min(correction, max_speed)
#    
#    # Применение коррекции к управлению Quadrocopter
#    rcdata = airsim.RCData()
#    print (dir(rcdata))
##    rcdata.setStickData(correction, 0, 0, 0)
##    client.moveByRC(rcdata)
##    client.moveByRC(rcdata = airsim.RCData(pitch = y_control,
##                                           throttle = rcdata.throttle,
##                                           yaw=z_control,
##                                           roll=x_control,
##                                           is_initialized = True,
##                                           is_valid = True))  



#    # Вывод текущих значений
#    print('Distance:', current_distance, 'Correction:', correction)

#    # Добавление неболькой паузы для симуляции реального времени
#    time.sleep(0.1)

# Выключение управления Quadrocopter
#client.disableApiControl()



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
    #client.enableApiControl(True)
    #client.armDisarm(True)
    # Создание PID-регуляторов
    pitch_pid = PidController(4.0, 0, 0)
    throttle_pid = PidController(0.045136137394194487, 0.00022393195314520642, 6.404490165264038)
    yaw_pid = PidController(0.2, 0, 0)
    roll_pid = PidController(0.5, 0, 0)

    # Целевая высота полета
    target_altitude = -4

    while True:
        # Получение текущей позиции и высоты
        position = client.getMultirotorState().kinematics_estimated.position
        altitude = position.z_val

        # Вычисление ошибки высоты
        altitude_error = target_altitude - altitude
        
        # Вычисление управляющих сигналов с использованием PID-регуляторов
        pitch_control_signal = pitch_pid.calculate_control_signal(altitude_error)
        throttle_control_signal = throttle_pid.calculate_control_signal(altitude_error)
        yaw_control_signal = yaw_pid.calculate_control_signal(0)  # Контроль направления на текущем уровне
        roll_control_signal = roll_pid.calculate_control_signal(0)  # Контроль наклона на текущем уровне

        # Отправка управляющих сигналов
#        client.moveByRC(pitch_control_signal, throttle_control_signal, yaw_control_signal, roll_control_signal)
        client.moveByRC(rcdata = airsim.RCData(#pitch = pitch_control_signal,
                                               throttle =-throttle_control_signal,
                                               #yaw=yaw_control_signal,
                                               #roll=roll_control_signal,
                                               is_initialized = True,
                                               is_valid = True)) 
        print (altitude, target_altitude, pitch_control_signal, throttle_control_signal)
        # Добавить задержку для обновления позиции
        time.sleep(0.01)

if __name__ == "__main__":
    flight_control()










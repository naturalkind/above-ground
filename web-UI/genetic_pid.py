import os
import sys
import cv2
import time
import socket
import pickle
import struct
import tracker_lib
from multiprocessing import Process, Value, Array, Manager
from collections import deque
from itertools import cycle
from yamspy import MSPy
from threading import Thread
import numpy as np
from deap import base, creator, tools, algorithms

class VideoStreamWidget(object):
    def __init__(self, src=1):
        self.capture = cv2.VideoCapture(src)
        # Start the thread to read frames from the video stream
        self.thread = Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()

    def update(self):
        # Read the next frame from the stream in a different thread
        while True:
            if self.capture.isOpened():
                (self.status, self.frame) = self.capture.read()
                self.img, self.obj_center, self.img_center = lib_start.process_img_server(self.frame, init_tracker)   
            #time.sleep(.1)
            #time.sleep(.01)
    def get_frame(self):
        return self.img, self.obj_center, self.img_center

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

# Функция для симуляции квадрокоптера с заданными параметрами PID
def simulate_pid(params):
    # Задание параметров PID
    Kp, Ki, Kd = params
    lib_start = tracker_lib.TrackerLib()
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

    # Создание сокета
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    host_name = socket.gethostname()
    #host_ip = '10.42.0.1'
    #host_ip = socket.gethostbyname(host_name)
    #host_ip ='192.168.0.104' 
    host_ip = '192.168.1.123'
    print('Хост IP:', host_ip)
    port = 9999
    socket_address = (host_ip, port)
    # Привязка сокета
    server_socket.bind(socket_address)

    # Ожидание подключения клиента
    server_socket.listen(5)
    print("Ожидание подключения клиента...")
    cap = cv2.VideoCapture(1)

    payload_size = struct.calcsize("Q")
    data = b""
    init_tracker = False
    client_socket = False
    dict_ = {}
    dict_["init_tracker"] = False


    # Create a PID controller object throttle
    pid_throttle = PIDController(Kp, Ki, Kd) # throttle
    pid_pitch = PIDController(Kp, Ki, Kd) 
    pid_roll = PIDController(Kp, Ki, Kd)


    # Max periods for:
    CTRL_LOOP_TIME = 1/100
    SLOW_MSGS_LOOP_TIME = 1/5 # these messages take a lot of time slowing down the loop...

    NO_OF_CYCLES_AVERAGE_GUI_TIME = 10

    #
    # On Linux, your serial port will probably be something like
    # /dev/ttyACM0 or /dev/ttyS0 or the same names with numbers different from 0
    #
    # On Windows, I would expect it to be 
    # COM1 or COM2 or COM3...
    #
    # This library uses pyserial, so if you have more questions try to check its docs:
    # https://pyserial.readthedocs.io/en/latest/shortintro.html
    #
    #
    SERIAL_PORT = "/dev/ttyACM0"
        
    CMDS = {
            'roll':     1500,
            'pitch':    1500,
            'throttle': 1000,
            'yaw':      1500,
            'aux1':     1000,
            'aux2':     1000
            }

    # This order is the important bit: it will depend on how your flight controller is configured.
    # Below it is considering the flight controller is set to use AETR.
    # The names here don't really matter, they just need to match what is used for the CMDS dictionary.
    # In the documentation, iNAV uses CH5, CH6, etc while Betaflight goes aux2, aux3...
    CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']
    start_fly = False

    ix = 0
    # "print" doesn't work with curses, use addstr instead
    try:
        with MSPy(device=SERIAL_PORT, loglevel='WARNING', baudrate=115200) as board:
            average_cycle = deque([0]*NO_OF_CYCLES_AVERAGE_GUI_TIME)

            command_list = ['MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO', 
                            'MSP_BOARD_INFO', 'MSP_UID', 'MSP_ACC_TRIM', 'MSP_NAME', 'MSP_STATUS', 'MSP_STATUS_EX',
                            'MSP_BATTERY_CONFIG', 'MSP_BATTERY_STATE', 'MSP_BOXNAMES']

            if board.INAV:
                command_list.append('MSPV2_INAV_ANALOG')
                command_list.append('MSP_VOLTAGE_METER_CONFIG')

            for msg in command_list: 
                if board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                    dataHandler = board.receive_msg()
                    board.process_recv_data(dataHandler)
            if board.INAV:
                cellCount = board.BATTERY_STATE['cellCount']
            else:
                cellCount = 0  
            min_voltage = board.BATTERY_CONFIG['vbatmincellvoltage']*cellCount
            warn_voltage = board.BATTERY_CONFIG['vbatwarningcellvoltage']*cellCount
            max_voltage = board.BATTERY_CONFIG['vbatmaxcellvoltage']*cellCount


            slow_msgs = cycle(['MSP_ANALOG', 'MSP_STATUS_EX', 'MSP_MOTOR', 'MSP_RC'])

            last_loop_time = last_slow_msg_time = last_cycleTime = time.time()
            
            local_fast_read_attitude = board.fast_read_attitude
            local_fast_read_imu = board.fast_read_imu
            local_fast_read_altitude = board.fast_read_altitude
            while(cap.isOpened()):
                if client_socket:
                
                    try:
                        # Сжатие кадра в формат JPEG
#                        _img, obj_center, img_center = video_stream_widget.get_frame()
                        (status, frame) = cap.read()
                        _img, obj_center, img_center = lib_start.process_img_server(frame, init_tracker)
                        _, img = cv2.imencode('.jpg', _img, encode_param)
                    except AttributeError:
                        pass
                
                    # отправка данных
                    a = pickle.dumps([img, init_tracker])
                    message = struct.pack("Q", len(a)) + a
                    client_socket.sendall(message)
                    
                    # получение данных
                    while len(data) < payload_size:
                        packet = client_socket.recv(4*1024)
                        if not packet: break
                        data += packet
                    packed_msg_size = data[:payload_size]
                    data = data[payload_size:]
                    
                    if not packed_msg_size: 
                        client_socket.close()
                        break
                    msg_size = struct.unpack("Q", packed_msg_size)[0]
                    
                    while len(data) < msg_size:
                        data += client_socket.recv(4*1024)
                    frame_data = data[:msg_size]
                    data = data[msg_size:]
                    bbox, state, init_switch = pickle.loads(frame_data)

                    if state > 1:
                        if sum(bbox[-2:]) > 10:
                            lib_start.init_tracker(_img, bbox)
                            lib_start.state = 0
                            init_tracker = True

                    lib_start.init_switch = init_switch
                    dict_["init_tracker"] = init_tracker   
                             
                    start_time = time.time()
                    
                    if init_tracker:
                        CMDS['aux1'] = 1500
                        start_fly = True
                        
                    if start_fly:
                        if init_tracker:
                            pid_output_roll = pid_roll.update(obj_center[0], img_center[0]) # yaw
                            if 1000 <= CMDS['yaw']+ pid_output_roll <= 1800:
                                print ("YAW", pid_output_roll, CMDS['yaw'], CMDS['yaw'] + pid_output_roll)
                                CMDS['yaw'] = CMDS['yaw'] + pid_output_roll
                            pid_output_throttle = pid_throttle.update(obj_center[1], img_center[1])        
                            if 1000 <= CMDS['throttle']+pid_output_throttle <= 1800:
                                print ("THRO", pid_output_throttle, CMDS['throttle'], CMDS['throttle']+pid_output_throttle)
#                                CMDS['throttle'] = CMDS['throttle'] + pid_output_throttle 
                                CMDS['throttle'] = Kp * abs(obj_center[1]-img_center[1]) +Ki * abs(obj_center[1]-img_center[1])
                                ix += 1

                            
                            
                            
                    if (time.time()-last_loop_time) >= CTRL_LOOP_TIME:
                        last_loop_time = time.time()
                        if board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER]):
                            dataHandler = board.receive_msg()
                            board.process_recv_data(dataHandler)
                    local_fast_read_imu() 
                    local_fast_read_attitude()
                    local_fast_read_altitude()
                    

                    end_time = time.time()
                    last_cycleTime = end_time-start_time
                    if (end_time-start_time)<CTRL_LOOP_TIME:
                        time.sleep(CTRL_LOOP_TIME-(end_time-start_time))
                        
                    average_cycle.append(end_time-start_time)
                    average_cycle.popleft()
                    print (dict_["init_tracker"], init_tracker)
                    
                    if ix == 10:
                        # Получение финальной позиции
                        x_val = abs(obj_center[1]-img_center[1])
                        y_val = abs(obj_center[1]-img_center[1])
                        return np.sqrt(x_val**2 + y_val**2)
                else:
                    print ('Connecting to the FC... connected!')
                    client_socket, addr = server_socket.accept()
                    print('Получено соединение от:', addr, client_socket)
    finally:
       #print ("ERROR") 
       pass
        
    
# Функция для симуляции квадрокоптера с заданными параметрами PID
#def simulate_pid(params):
#    # Задание параметров PID
#    kp, ki, kd = params

#    # Задание других параметров симуляции
#    max_time = 10  # Продолжительность симуляции в секундах
#    dt = 0.1  # Шаг времени в секундах

#    # Начальные условия
#    initial_pose = airsim.Pose(airsim.Vector3r(0, 0, -10), airsim.to_quaternion(0, 0, 0))
#    client.simSetVehiclePose(initial_pose, True)

#    # Симуляция
#    for _ in range(int(max_time / dt)):
#        # Получение текущей позиции
#        pose = client.simGetVehiclePose()
#        position = pose.position
#        print (position)
#        # Рассчет управляющего воздействия с использованием PID
#        control_input = kp * position.z + ki * position.z + kd * position.z

#        # Применение управляющего воздействия
#        client.moveByRC(rcdata = airsim.RCData(pitch = 0.0,
#                                               throttle = control_input,
#                                               yaw=0.0,
#                                               roll=0.0,
#                                               is_initialized = True,
#                                               is_valid = True))        

#    # Получение финальной позиции
#    final_pose = client.simGetVehiclePose()
#    final_position = final_pose.position


    # Возвращение значения функции приспособленности (цель - минимизация расстояния до целевой точки)
#    return np.sqrt(final_position.x_val**2 + final_position.y_val**2 + final_position.z_val**2)

# Создание класса FitnessMin для минимизации функции приспособленности
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))

# Создание класса Individual с одним атрибутом, представляющим параметры PID
creator.create("Individual", list, fitness=creator.FitnessMin)

# Определение функции для инициализации особи
def init_individual():
    return [np.random.uniform(0, 1) for _ in range(3)]  # Инициализация случайных значений для параметров PID

init_individual = init_individual()
print ("....", simulate_pid(init_individual))

## Определение генетических операторов
toolbox = base.Toolbox()
toolbox.register("individual", tools.initIterate, creator.Individual, init_individual)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)
#toolbox.register("evaluate", simulate_pid)
#toolbox.register("mate", tools.cxBlend, alpha=0.5)
#toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=0.2, indpb=0.2)
#toolbox.register("select", tools.selTournament, tournsize=3)

## Создание начальной популяции
#population = toolbox.population(n=10)

## Запуск генетического алгоритма
#algorithms.eaMuPlusLambda(population, toolbox, mu=10, lambda_=20, cxpb=0.7, mutpb=0.3, ngen=10, stats=None, halloffame=None)

## Вывод лучшей особи
#best_individual = tools.selBest(population, k=1)[0]
#print("Best Individual:", best_individual)
#print("Best Fitness:", best_individual.fitness.values)

    
    
    
    
    
    
    
    

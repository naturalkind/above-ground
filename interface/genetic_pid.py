import os
import sys
import cv2
import time
import socket
import pickle
import struct
import signal
import tracker_lib
from multiprocessing import Process, Value, Array, Manager, Pipe
from threading import Thread, BrokenBarrierError
from collections import deque
from itertools import cycle
from yamspy import MSPy
from threading import Thread
import numpy as np
from deap import base, creator, tools, algorithms


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


## Функция для симуляции квадрокоптера с заданными параметрами PID
def simulate_pid(params):
    # Задание параметров PID
    Kp, Ki, Kd = params

    init_tracker = False

    # Create a PID controller object throttle
    pid_throttle = PIDController(Kp, Ki, Kd) # throttle
    pid_pitch = PIDController(Kp, Ki, Kd) 
    pid_roll = PIDController(Kp, Ki, Kd)

    DEBUG = True
    board = None

    PRINT_VALUES_FREQ = 5
    JOYSTICK_FREQ = 20
    MAIN_FREQ = 50
    READ_VOLT_FC_FREQ = 1
    READ_IMU_FC_FREQ = 15

    wfc = []
    rfc = []
    rfcbat = []

    CMDS_init = {
            'roll':     1500,
            'pitch':    1500,
            'throttle': 1000,
            'yaw':      1500,
            'aux1':     1500, # DISARMED (1000) / ARMED (2000)
            'aux2':     1000, # ANGLE (1000) / HORIZON (1500) / FLIP (1800)
            'aux3':     1000, # FAILSAFE (1800)
            'aux4':     1000  # HEADFREE (1800)
            }

    CMDS = CMDS_init.copy()

    command_list = ['MSP_API_VERSION', 'MSP_FC_VARIANT', 'MSP_FC_VERSION', 'MSP_BUILD_INFO', 
    'MSP_BOARD_INFO', 'MSP_UID', 'MSP_ACC_TRIM', 'MSP_NAME', 'MSP_STATUS', 'MSP_STATUS_EX',
    'MSP_BATTERY_CONFIG', 'MSP_BATTERY_STATE', 'MSP_BOXNAMES', 'MSP_ANALOG']

    CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2', 'aux3', 'aux4']
    shutdown = False
    fc_reboot = False

    try:
        while not shutdown:
            with MSPy(device="COM5", loglevel='WARNING', baudrate=115200) as board:
                if board == 1: # an error occurred...
                    print("Not connected to the FC...")              
                    continue
                else:
                    try:
                        for msg in command_list: 
                            if board.send_RAW_msg(MSPy.MSPCodes[msg], data=[]):
                                dataHandler = board.receive_msg()
                                board.process_recv_data(dataHandler)
                        prev_time = time.time()
                        ix = 0
                        ix_output = 0
                        while not shutdown:
                            CMDS_RC = [CMDS[ki] for ki in CMDS_ORDER]

                            if board.send_RAW_RC(CMDS_RC):
                                dataHandler = board.receive_msg()
                                board.process_recv_data(dataHandler)
                            
                            board.fast_read_analog()
                            board.fast_read_attitude()
                            board.fast_read_imu()
                            accelerometer = board.SENSOR_DATA['accelerometer']
                            gyroscope = board.SENSOR_DATA['gyroscope']
                            voltage = board.ANALOG['voltage']
                            attitude = board.SENSOR_DATA['kinematics']

                            #print(accelerometer)
                            #print(gyroscope)
                            #print(attitude)
                            #print(voltage)
                            #print (dict_)
                            
                            # Read info from the FC
                            if board.send_RAW_msg(MSPy.MSPCodes['MSP_STATUS_EX'], data=[]):
                                dataHandler = board.receive_msg()
                                board.process_recv_data(dataHandler)
                            ARMED = board.bit_check(board.CONFIG['mode'],0)
                            print(ARMED, board.process_mode(board.CONFIG['mode']), CMDS)
                
                            #print ("Read speed: %2.2f Hz"%(1/(time.time()-prev_time)))
                            prev_time = time.time()
                            ix += 1
                            if ix > 50:
                                
                                CMDS["aux3"] = 1500
                                #init_tracker = True
                            if ARMED:
                                if dict_["init_tracker"]:
    #                                pid_output_throttle = pid_throttle.update(dict_["z_target"], dict_["z_current"]) 
                                    pid_output_throttle = Kp * dict_["z_target"] +Ki * dict_["z_target"] + Kd * dict_["z_target"] 
                                    #print (CMDS['throttle'] + pid_output_throttle)    
                                    if 1000 <= CMDS['throttle'] + pid_output_throttle <= 1850:
                                        CMDS['throttle'] = CMDS['throttle'] + pid_output_throttle 
                                    ix_output += 1
                                if ix_output == 2000:
                                    # Получение финальной позиции
                                    x_val = dict_["z_target"]
                                    y_val = dict_["y_target"] 
                                    print ("END...", np.sqrt(x_val**2 + y_val**2), ix)
                                    CMDS["aux3"] = 200
                                    CMDS_RC = [CMDS[ki] for ki in CMDS_ORDER]
                                    board.fast_msp_rc_cmd(CMDS_RC)
                                    return np.sqrt(x_val**2 + y_val**2),
#                            else:
#                                print ("???")
#                                CMDS_RC = [CMDS[ki] for ki in CMDS_ORDER]
#                                board.fast_msp_rc_cmd(CMDS_RC)
                    except KeyboardInterrupt:
                        shutdown = True
    finally:
        print("FINISHED")

def image_task(dict_):
    # Создание сокета
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    host_name = socket.gethostname()
    host_ip = socket.gethostbyname(host_name)
    #host_ip = '10.42.0.1'
    # host_ip = '192.168.1.123'
    print('Хост IP:', host_ip)
    port = 9999
    socket_address = (host_ip, port)
    # Привязка сокета
    server_socket.bind(socket_address)

    # Ожидание подключения клиента
    server_socket.listen(5)
    
    lib_start = tracker_lib.TrackerLib()
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
    cap = cv2.VideoCapture(0)
    # video_stream_widget = VideoStreamWidget()
    print("Ожидание подключения клиента...")
    payload_size = struct.calcsize("Q")
    data = b""
    init_tracker = False
    client_socket = False
    while True:
        client_socket, addr = server_socket.accept()
        #print('Получено соединение от:', addr, client_socket)
        try:
            # while(video_stream_widget.capture.isOpened()):
           while(cap.isOpened()):
                (status, frame) = cap.read()
                if status:
                    frame = cv2.resize(frame, (int(frame.shape[1]*1.4), int(frame.shape[0]*1.4)))
                    _img, obj_center, img_center = lib_start.process_img_server(frame, dict_["init_tracker"])  
                    # Сжатие кадра в формат JPEG
                    # _img, obj_center, img_center = video_stream_widget.get_frame()
                    _, img = cv2.imencode('.jpg', _img, encode_param)
                    
                    dict_["y_current"] = img_center[0]
                    dict_["z_current"] = img_center[1]
                    
                    dict_["y_target"] = obj_center[0]
                    dict_["z_target"] = obj_center[1]
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
                            lib_start.init_tracker(_img, bbox, A = True, B = True)
                            lib_start.state = 0
                            init_tracker = True

                    lib_start.init_switch = init_switch
                    dict_["init_tracker"] = init_tracker
        except ConnectionResetError:
            print("Клиент отключился")
            client_socket.close()
    
    # Close the server socket
    server_socket.close()

if __name__ == '__main__':

    with Manager() as manager:
        dict_ = manager.dict()
        dict_["init_tracker"] = False
        thread2 = Process(target=image_task, args=(dict_,), daemon=True)
        thread2.start()
        # thread2.join() 

        # Создание класса FitnessMin для минимизации функции приспособленности
        creator.create("FitnessMin", base.Fitness, weights=(-1.0,))

        ## Создание класса Individual с одним атрибутом, представляющим параметры PID
        creator.create("Individual", list, fitness=creator.FitnessMin)

        ## Определение функции для инициализации особи
        def init_individual():
            return [np.random.uniform(0, 0.004) for _ in range(3)]  # Инициализация случайных значений для параметров PID

        ## Определение генетических операторов
        toolbox = base.Toolbox()
        toolbox.register("individual", tools.initIterate, creator.Individual, init_individual)
        toolbox.register("population", tools.initRepeat, list, toolbox.individual)
        toolbox.register("evaluate", simulate_pid)
        toolbox.register("mate", tools.cxBlend, alpha=0.5)
        toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=0.2, indpb=0.2)
        toolbox.register("select", tools.selTournament, tournsize=3)

        ## Создание начальной популяции
        population = toolbox.population(n=10)

        ## Запуск генетического алгоритма
        algorithms.eaMuPlusLambda(population, toolbox, mu=10, lambda_=20, cxpb=0.7, mutpb=0.3, ngen=10, stats=None, halloffame=None)

        ## Вывод лучшей особи
        best_individual = tools.selBest(population, k=1)[0]
        print("Best Individual:", best_individual)
        print("Best Fitness:", best_individual.fitness.values)

#Best Individual: [0.0006422135466285553, 0.0016968316583189755, 5.241418595445369e-05]
    
    
    
    
    
    
    
    

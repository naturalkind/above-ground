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
from matplotlib import pyplot as plt

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
def simulate_pid(dict_):
    # Задание параметров PID
    Kp = 0.002 
    Ki = 0.0002 
    Kd = 0.0#2
    

    init_tracker = False

    # Create a PID controller object throttle
    pid_throttle = PIDController(Kp, Ki, Kd) # throttle
    pid_pitch = PIDController(Kp, Ki, Kd) 
    pid_roll = PIDController(Kp, Ki, Kd)

    board = None

    CMDS_init = {
            'roll':     1500,
            'pitch':    1500,
            'throttle': 1000,
            'yaw':      1500,
            'aux1':     1000, # DISARMED (1000) / ARMED (2000)
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

    list_target = []
    list_rc = []
#    try:
    while not shutdown:
        with MSPy(device="/dev/ttyACM0", loglevel='WARNING', baudrate=115200) as board:
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
            
                        #print ("Read speed: %2.2f Hz"%(1/(time.time()-prev_time)))
                        
                        ix += 1
                        if ix > 50:
                            CMDS["aux1"] = 2000
                            CMDS["aux3"] = 1500
                            #init_tracker = True
                        if ARMED:
                            if dict_["init_tracker"]:
                                pid_output_throttle = pid_throttle.update(dict_["z_target"], dict_["z_current"]) 
#                                pid_output_throttle = Kp * dict_["z_target"] +Ki * dict_["z_target"] + Kd * dict_["z_target"] 
                                #print (CMDS['throttle'] + pid_output_throttle)    
                                if 1000 <= CMDS['throttle'] + pid_output_throttle <= 1850:
                                    CMDS['throttle'] = CMDS['throttle'] + pid_output_throttle 
                                list_target.append(dict_["z_target"])
                                list_rc.append(CMDS['throttle'])
                                
                        #if ix > 250:
                            #time.sleep(100)
                        
                        print(ARMED, board.process_mode(board.CONFIG['mode']), CMDS, (time.time()-prev_time))
                        prev_time = time.time()
                except KeyboardInterrupt:
                    shutdown = True
#    finally:
#        print("FINISHED")
#        simulate_pid(dict_)

def image_task(dict_):
    # Создание сокета
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    host_name = socket.gethostname()
    #host_ip = socket.gethostbyname(host_name)
    #host_ip = '10.42.0.1'
    host_ip = '192.168.1.123'
    print('Хост IP:', host_ip)
    port = 9999
    socket_address = (host_ip, port)
    # Привязка сокета
    server_socket.bind(socket_address)

    # Ожидание подключения клиента
    server_socket.listen(5)
    
    lib_start = tracker_lib.TrackerLib()
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
    cap = cv2.VideoCapture(1)
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
                            lib_start.init_tracker(_img, bbox)
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
        # run the thread
        thread1 = Process(target=simulate_pid, args=(dict_,), daemon=True)              
        thread1.start()     
                
        thread2 = Process(target=image_task, args=(dict_,), daemon=True)
        thread2.start() 
        
        # wait for the thread to finish
        print('Waiting for the thread...')
        thread2.join()    
    
    
    
    
    
    
    
    

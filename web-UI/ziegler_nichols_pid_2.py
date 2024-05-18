import os
import sys
import cv2
import time
import curses
import socket
import pickle
import struct
import tracker_lib
from multiprocessing import Process, Value, Array, Manager
from collections import deque
from itertools import cycle
from yamspy import MSPy
from threading import Thread
from filterpy.memory import FadingMemoryFilter
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.ticker as ticker
from scipy.signal import argrelextrema
import json


lib_start = tracker_lib.TrackerLib()
encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]



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
# SERIAL_PORT = "/dev/ttyACM0"
SERIAL_PORT = "COM5"

"""
Этот код создает экземпляр PID-регулятора с определенными коэффициентами Kp, Ki и Kd. 
Затем он подключается к симулятору AirSim и запускает цикл управления. В цикле он 
получает текущую высоту, вычисляет ошибку относительно целевой высоты, обновляет 
PID-регулятор и применяет получившийся управляющий сигнал к функции moveByRC

"""

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


def run_curses(dict_):
    result=1

    try:
        # get the curses screen window
        screen = curses.initscr()

        # turn off input echoing
        curses.noecho()

        # respond to keys immediately (don't wait for enter)
        curses.cbreak()

        # non-blocking
        screen.timeout(0)

        # map arrow keys to special values
        screen.keypad(True)

        screen.addstr(1, 0, "Press 'q' to quit, 'r' to reboot, 'a' to arm, 'd' to disarm, 's' start flight and arrow keys to control altitude", curses.A_BOLD)
        
        result = keyboard_controller(screen, dict_)
    finally:
        # shut down cleanly
        curses.nocbreak(); screen.keypad(0); curses.echo()
        curses.endwin()
        if result==1:
            print("An error occurred... probably the serial port is not available ;)")

def value_limit(output, limit):
    '''Set the value not exceed the limited value'''
    if abs(output) > limit:
        return limit*int(output/abs(output))
    else:
        return output 

def keyboard_controller(screen, dict_):
    # PID up 

    # Kp_y = 0.0 
    # Ki_y = 0.0 
    # Kd_y = 0.0 

    # Лучший 1
    # Kp_y = -0.0012 
    # Ki_y = -0.0000012 
    # Kd_y = -0.028799709320068358

    # Лучший 2
    # Kp_y = -0.0012 
    # Ki_y = -0.00012 
    # Kd_y = -0.28799709320068358

    # Лучший 3
    Kp_y = -0.0012 
    Ki_y = -0.00022 
    Kd_y = -0.48799709320068358
    # Create a PID controller object yaw 
    pid_yaw = PIDController(Kp_y, Ki_y, Kd_y) # throttle   



    # дросель
    # лучший 1
    # Kp_z = 0.0014229999999999853
    # Ki_z = 0.00015179783099688674 
    # Kd_z = 0.3334910958051647

    # лучший 5 4400Ah
    # Kp_z = 0.0014#42
    # Ki_z = 0.0002#5
    # Kd_z = 0.3335

    # # лучший 6  4400Ah
    # Kp_z = 0.00014#42
    # Ki_z = 0.00002#5
    # Kd_z = 0.3335

    # # лучший 7  4400Ah
    # Kp_z = 0.00014#42
    # Ki_z = 0.00002#5
    # Kd_z = 0.37

    # лучший 8  4400Ah
    # Kp_z = 0.00074#42
    # Ki_z = 0.000031#5
    # Kd_z = 0.297


    # лучший 5 5500Ah
    Kp_z = 0.0005#42
    Ki_z = 0.00005#5
    Kd_z = 0.4435


    # genetic PID
    # Kp_z = 0.0048
    # Ki_z = 0.00011
    # Kd_z = 0.22


    # Create a PID controller object throttle
    pid_throttle = PIDController(Kp_z, Ki_z, Kd_z) # throttle   


    
    CMDS = {
            'roll':     1500,
            'pitch':    1500,
            'throttle': 1000,
            'yaw':      1500,
            'aux1':     1500,
            'aux2':     1000,
            'aux3':     1000
            }

    # This order is the important bit: it will depend on how your flight controller is configured.
    # Below it is considering the flight controller is set to use AETR.
    # The names here don't really matter, they just need to match what is used for the CMDS dictionary.
    # In the documentation, iNAV uses CH5, CH6, etc while Betaflight goes aux2, aux3...
    CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2', 'aux3']
    start_fly = False
    start_track = False
    height = 0.0
    # "print" doesn't work with curses, use addstr instead
    try:
        screen.addstr(15, 0, "Connecting to the FC...")

        with MSPy(device=SERIAL_PORT, loglevel='WARNING', baudrate=115200) as board:
            if board == 1: # an error occurred...
                return 1
            screen.addstr(15, 0, "Connecting to the FC... connected!")
            screen.clrtoeol()
            screen.move(1,0)

            average_cycle = deque([0]*NO_OF_CYCLES_AVERAGE_GUI_TIME)

            # It's necessary to send some messages or the RX failsafe will be activated
            # and it will not be possible to arm.
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
                cellCount = 0 # MSPV2_INAV_ANALOG is necessary
            min_voltage = board.BATTERY_CONFIG['vbatmincellvoltage']*cellCount
            warn_voltage = board.BATTERY_CONFIG['vbatwarningcellvoltage']*cellCount
            max_voltage = board.BATTERY_CONFIG['vbatmaxcellvoltage']*cellCount

            screen.addstr(15, 0, "apiVersion: {}".format(board.CONFIG['apiVersion']))
            screen.clrtoeol()
            screen.addstr(15, 50, "flightControllerIdentifier: {}".format(board.CONFIG['flightControllerIdentifier']))
            screen.addstr(16, 0, "flightControllerVersion: {}".format(board.CONFIG['flightControllerVersion']))
            screen.addstr(16, 50, "boardIdentifier: {}".format(board.CONFIG['boardIdentifier']))
            screen.addstr(17, 0, "boardName: {}".format(board.CONFIG['boardName']))

            slow_msgs = cycle(['MSP_ANALOG', 'MSP_STATUS_EX', 'MSP_MOTOR', 'MSP_RC'])

            cursor_msg = ""
            last_loop_time = last_slow_msg_time = last_cycleTime = time.time()
            
            local_fast_read_attitude = board.fast_read_attitude
            local_fast_read_imu = board.fast_read_imu
            local_fast_read_altitude = board.fast_read_altitude
            local_fast_msp_rc_cmd = board.fast_msp_rc_cmd
            prev_step_time = 0


            # throtle
            list_target_thr = []
            list_rc_thr = []
            list_time_thr = []
            list_pid_thr = []

            # yaw
            list_target_yaw = []
            list_rc_yaw = []
            list_time_yaw = []
            list_pid_yaw = []

            while True:
                start_time = time.time()
                char = screen.getch() # get keypress
                curses.flushinp() # flushes buffer
                

                local_fast_read_imu() 
                local_fast_read_attitude()
                local_fast_read_altitude()
   
                           
                #
                # Key input processing
                #

                #
                # KEYS (NO DELAYS)
                #
                if char == ord('q') or char == ord('Q'):
                    break

                elif char == ord('d') or char == ord('D'):
                    cursor_msg = 'Sending Disarm command...'
                    CMDS['aux3'] = 1000
                    CMDS['throttle'] = 1000
                    start_fly = False
                    start_track = False


                    with open('data.json', 'w') as f:
                        data = {"list_time_thr":list_time_thr, 
                                "list_rc_thr":list_rc_thr, 
                                "list_target_thr":list_target_thr,
                                "list_pid_thr":list_pid_thr,

                                "list_target_yaw":list_target_yaw,
                                "list_rc_yaw":list_rc_yaw,
                                "list_time_yaw":list_time_yaw,
                                "list_pid_yaw":list_pid_yaw

                                }
                        json.dump(data, f)



                elif char == ord('r') or char == ord('R'):
                    screen.addstr(3, 0, 'Sending Reboot command...')
                    screen.clrtoeol()
                    board.reboot()
                    time.sleep(0.5)
                    break

                elif char == ord('a') or char == ord('A'):
                    cursor_msg = 'Sending Arm command...'
                    CMDS['aux3'] = 1500
                elif char == 259:
                    # Kp += 0.001
                    # Create a PID controller object throttle
                    # pid_throttle = PIDController(Kp_z, Ki_z, Kd_z) # throttle

                    # Yaw
                    Kp_y += 0.001
                    pid_yaw = PIDController(Kp_y, Ki_y, Kd_y)
                elif char == 258:

                    # Kp -= 0.001
                    # Create a PID controller object throttle
                    # pid_throttle = PIDController(Kp_z, Ki_z, Kd_z) # throttle

                    # Yaw
                    Kp_y -= 0.001
                    pid_yaw = PIDController(Kp_y, Ki_y, Kd_y)
                elif char == ord('z') or char == ord('Z'):
                    if ARMED:
                        start_track = True
                        CMDS['throttle'] = 1380
                #
                # IMPORTANT MESSAGES (CTRL_LOOP_TIME based)
                #
                if (time.time()-last_loop_time) >= CTRL_LOOP_TIME:
                    last_loop_time = time.time()
                    # Send the RC channel values to the FC
                    if board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER]):
                        dataHandler = board.receive_msg()
                        board.process_recv_data(dataHandler)
#                    local_fast_msp_rc_cmd([CMDS[ki] for ki in CMDS_ORDER])
                
                #
                # SLOW MSG processing (user GUI)
                #


                if dict_["init_tracker"]: # KEY "Z"
                    if start_track:

                        # Throttle

                        pid_output_throttle = pid_throttle.update(dict_["z_target"], dict_["z_current"])        
                        if 1000 <= CMDS['throttle']+pid_output_throttle <= 1900:
                            CMDS['throttle'] = CMDS['throttle'] + pid_output_throttle 
                        list_target_thr.append(dict_["z_target"]-dict_["z_current"])
                        list_rc_thr.append(CMDS['throttle'])
                        list_pid_thr.append([Kp_z, Ki_z, Kd_z])


                        # Yaw

                        # CMDS['throttle'] = 1250
                        pid_output_yaw = pid_yaw.update(dict_["y_target"], dict_["y_current"]) 
                        CMDS['yaw'] = CMDS['yaw'] + pid_output_yaw
                        list_target_yaw.append(dict_["y_target"]-dict_["y_current"])
                        list_rc_yaw.append(CMDS['yaw'])
                        list_pid_yaw.append([Kp_y, Ki_y, Kd_y])

                        l_time = time.time()-start_time
                        list_time_thr.append(l_time)
                        list_time_yaw.append(l_time)


                        cursor_msg = f'Init tracker is True, {CMDS["throttle"]}, target pos: {dict_["z_target"]}, corrent: {dict_["z_current"]}, {pid_output_throttle}, Target Kp: {Kp_z}'
                        # cursor_msg = f'Init tracker is True, {CMDS["yaw"]}, target pos: {dict_["y_target"]}, corrent: {dict_["y_current"]}, {pid_output_yaw}, Target Kp_y: {Kp_y}'
                    

                        # pitch 
                        #CMDS['pitch'] = 1800

                screen.addstr(7, 100, "Start track: {}".format(str(start_track)), curses.A_BOLD)
                screen.clrtoeol()                
                  
                if (time.time()-last_slow_msg_time) >= SLOW_MSGS_LOOP_TIME:
                    last_slow_msg_time = time.time()

                    next_msg = next(slow_msgs) # circular list

                    # Read info from the FC
                    if board.send_RAW_msg(MSPy.MSPCodes[next_msg], data=[]):
                        dataHandler = board.receive_msg()
                        board.process_recv_data(dataHandler)
                        
                    if next_msg == 'MSP_ANALOG':
                        voltage = board.ANALOG['voltage']
                        voltage_msg = ""
                        if min_voltage < voltage <= warn_voltage:
                            voltage_msg = "LOW BATT WARNING"
                        elif voltage <= min_voltage:
                            voltage_msg = "ULTRA LOW BATT!!!"
                        elif voltage >= max_voltage:
                            voltage_msg = "VOLTAGE TOO HIGH"

                        screen.addstr(18, 0, "Battery Voltage: {:2.2f}V".format(board.ANALOG['voltage']))
                        screen.clrtoeol()
                        screen.addstr(18, 24, voltage_msg, curses.A_BOLD + curses.A_BLINK)
                        screen.clrtoeol()

                    elif next_msg == 'MSP_STATUS_EX':
                        ARMED = board.bit_check(board.CONFIG['mode'],0)
                        screen.addstr(5, 0, "ARMED: {}".format(ARMED), curses.A_BOLD)
                        screen.clrtoeol()

                        screen.addstr(5, 50, "armingDisableFlags: {}".format(board.process_armingDisableFlags(board.CONFIG['armingDisableFlags'])))
                        screen.clrtoeol()

                        screen.addstr(6, 0, "cpuload: {}".format(board.CONFIG['cpuload']))
                        screen.clrtoeol()
                        screen.addstr(6, 50, "cycleTime: {}".format(board.CONFIG['cycleTime']))
                        screen.clrtoeol()

                        screen.addstr(7, 0, "mode: {}".format(board.CONFIG['mode']))
                        screen.clrtoeol()

                        screen.addstr(7, 50, "Flight Mode: {}".format(board.process_mode(board.CONFIG['mode'])))
                        screen.clrtoeol()
                    elif next_msg == 'MSP_MOTOR':
                        screen.addstr(19, 0, "Motor Values: {}".format(board.MOTOR_DATA))
                        screen.clrtoeol()

                    elif next_msg == 'MSP_RC':
                        screen.addstr(20, 0, "RC Channels Values: {}".format(board.RC['channels']))
                        screen.clrtoeol()
                        
                    screen.addstr(17, 50, "GUI cycleTime: {0:2.2f}ms (average {1:2.2f}Hz)".format((last_cycleTime)*1000,
                                  (sum(average_cycle)/len(average_cycle))))
                    screen.clrtoeol()

                    screen.addstr(3, 0, cursor_msg)
                    screen.clrtoeol()
                    

                end_time = time.time()
                last_cycleTime = end_time-start_time
                if (end_time-start_time)<CTRL_LOOP_TIME:
                    time.sleep(CTRL_LOOP_TIME-(end_time-start_time))
                    
                average_cycle.append(end_time-start_time)
                average_cycle.popleft()

    finally:
        screen.addstr(5, 0, "Disconneced from the FC!")
        screen.clrtoeol()

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
    
    cap = cv2.VideoCapture(0)
    print("Ожидание подключения клиента...")
    payload_size = struct.calcsize("Q")
    data = b""
    init_tracker = False
    client_socket = False
    while True:
        client_socket, addr = server_socket.accept()
        #print('Получено соединение от:', addr, client_socket)
        try:
           while(cap.isOpened()):
#                try:
                (status, frame) = cap.read()
                if status:
                    frame = cv2.resize(frame, (int(frame.shape[1]*1.4), int(frame.shape[0]*1.4)))
                    _img, obj_center, img_center = lib_start.process_img_server(frame, dict_["init_tracker"])  
                    # Сжатие кадра в формат JPEG
                    _, img = cv2.imencode('.jpg', _img, encode_param)
                    
                    dict_["y_current"] = img_center[0]
                    dict_["z_current"] = img_center[1]
                    
                    dict_["y_target"] = obj_center[0]
                    dict_["z_target"] = obj_center[1]
    #                except AttributeError:
    #                    pass
                    num.value = obj_center[1]
                    #time.sleep(0.04)
                    #print (dict_, obj_center, img_center)  
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
    
num = Value('d', 0.0)    
if __name__ == '__main__':
    
    with Manager() as manager:
        dict_ = manager.dict()
        dict_["init_tracker"] = False
        # run the thread
        thread1 = Process(target=run_curses, args=(dict_,), daemon=True)              
        thread1.start()   # "BP_FlyingPawn_11", "BP_FlyingPawn2_2"  
                
        thread2 = Process(target=image_task, args=(dict_,), daemon=True)
        thread2.start() #"BP_FlyingPawn2_2"#"BP_FlyingPawn2_7"
        
        # wait for the thread to finish
        print('Waiting for the thread...')
        thread1.join()  
        thread2.join()    
        
        
"""
1 запускать автоматически 
2 при нажатии армить
3 при нажатии запускать выбор цели

"""    

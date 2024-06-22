"""
1 запускать автоматически 
2 при нажатии армить
3 при нажатии запускать выбор цели

"""  
import os
import sys
import cv2
import time
import curses
import socket
import pickle
import struct
from tracker_lib import tracker_lib
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
SERIAL_PORT = "/dev/ttyACM0" # Linux
#SERIAL_PORT = "COM5" # Windows

"""
Экземпляр PID-регулятора с определенными коэффициентами Kp, Ki, Kd. 
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

    ##########
    # yaw PID
    ##########
    
    # Лучший 3
    Kp_y = -0.0012 
    Ki_y = -0.00022 
    Kd_y = -0.48799709320068358
    
    pid_yaw = PIDController(Kp_y, Ki_y, Kd_y) 


    ##########
    # roll PID
    ##########
    
    Kp_y = 0 
    Ki_y = 0 
    Kd_y = 0
    pid_roll = PIDController(Kp_y, Ki_y, Kd_y)  
    

    ##########
    # throttle PID
    ##########

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
    pid_throttle = PIDController(Kp_z, Ki_z, Kd_z) 

    
    CMDS = {
            'roll':     1500,
            'pitch':    1500,
            'throttle': 1000,
            'yaw':      1500,
            'aux1':     1500, # auto 
            'aux2':     1000, # arm
            'aux3':     1000,
            'aux4':     1000
            }

    # This order is the important bit: it will depend on how your flight controller is configured.
    # Below it is considering the flight controller is set to use AETR.
    # The names here don't really matter, they just need to match what is used for the CMDS dictionary.
    # In the documentation, iNAV uses CH5, CH6, etc while Betaflight goes aux2, aux3...
    CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2', 'aux3', 'aux4']
    autopilot = False
    ARMED = False
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
            
            slow_msgs = cycle(['MSP_ANALOG', 'MSP_STATUS_EX', 'MSP_MOTOR', 'MSP_RC', 'MSP_RX_MAP', 
                               'MSP_FEATURE_CONFIG'])

            cursor_msg = ""
            last_loop_time = last_slow_msg_time = last_cycleTime = time.time()
            
            local_fast_read_attitude = board.fast_read_attitude
            local_fast_read_imu = board.fast_read_imu
            local_fast_read_altitude = board.fast_read_altitude
            local_fast_msp_rc_cmd = board.fast_msp_rc_cmd
            prev_step_time = 0
            last_channels = []
            
            # throttle
            list_target_thr = []
            list_rc_thr = []
            list_pid_thr = []

            # yaw
            list_target_yaw = []
            list_rc_yaw = []
            list_pid_yaw = []

            # roll
            list_target_roll = []
            list_rc_roll = []
            list_pid_roll = []            

            # time
            list_time = []
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
                elif char == ord('a') or char == ord('A'):
                    cursor_msg = 'Sending Arm command...'
                    CMDS['aux2'] = 1500
                elif char == ord('d') or char == ord('D'):
                    cursor_msg = 'Sending Disarm command...'
                    CMDS['aux2'] = 1000
                    CMDS['throttle'] = 1000

                    # Получите текущую дату и время
                    current_time = time.localtime(time.time())
                    current_date = time.strftime('%Y-%m-%d_%H-%M-%S', current_time)
                    with open(f'data/pid_data/data_{current_date}.json', 'w') as f:
                        data = {"list_time":list_time, 
                        
                                "list_rc_thr":list_rc_thr, 
                                "list_target_thr":list_target_thr,
                                "list_pid_thr":list_pid_thr,

                                "list_target_yaw":list_target_yaw,
                                "list_rc_yaw":list_rc_yaw,
                                "list_pid_yaw":list_pid_yaw,
                                
                                "list_target_roll":list_target_roll,
                                "list_rc_roll":list_rc_roll,
                                "list_pid_roll":list_pid_roll                                
                                }
                        json.dump(data, f)

                elif char == ord('r') or char == ord('R'):
                    screen.addstr(3, 0, 'Sending Reboot command...')
                    screen.clrtoeol()
                    board.reboot()
                    time.sleep(0.5)
                    break
                    
                elif char == ord('w') or char == ord('W'):
                    CMDS['throttle'] = CMDS['throttle'] + 10 if CMDS['throttle'] + 10 <= 2000 else CMDS['throttle']
                    cursor_msg = 'W Key - throttle(+):{}'.format(CMDS['throttle'])

                # Ручное управление PID
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
                    
                # Управление двигателями принудительно
                
#                elif char == ord('z') or char == ord('Z'):
#                    board.send_RAW_MOTORS(data = [1100, 1100, 1100, 1100, 0, 0, 0, 0])
#                elif char == ord('x') or char == ord('X'):
#                    board.send_RAW_MOTORS(data = [0, 0, 0, 0, 0, 0, 0, 0])


                #
                # IMPORTANT MESSAGES (CTRL_LOOP_TIME based)
                #
                if (time.time()-last_loop_time) >= CTRL_LOOP_TIME:
                    last_loop_time = time.time()
                    # Send the RC channel values to the FC
                    if board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER]):
                        dataHandler = board.receive_msg()
                        board.process_recv_data(dataHandler)

                    # Работает
#                    local_fast_msp_rc_cmd([CMDS[ki] for ki in CMDS_ORDER])
                #
                # SLOW MSG processing (user GUI)
                #
                if ARMED == autopilot == dict_["init_tracker"] == True:
                        
                    # throttle
                    
                    pid_output_throttle = pid_throttle.update(dict_["z_target"], dict_["z_current"])        
                    if 1000 <= CMDS['throttle']+pid_output_throttle <= 1900:
                        CMDS['throttle'] = CMDS['throttle'] + pid_output_throttle 
                    list_target_thr.append(dict_["z_target"]-dict_["z_current"])
                    list_rc_thr.append(CMDS['throttle'])
                    list_pid_thr.append([Kp_z, Ki_z, Kd_z])


                    # yaw

                    # CMDS['throttle'] = 1250 
                    pid_output_yaw = pid_yaw.update(dict_["y_target"], dict_["y_current"]) 
                    CMDS['yaw'] = CMDS['yaw'] + pid_output_yaw
                    list_target_yaw.append(dict_["y_target"]-dict_["y_current"])
                    list_rc_yaw.append(CMDS['yaw'])
                    list_pid_yaw.append([Kp_y, Ki_y, Kd_y])

                    # roll

                    pid_output_roll = pid_roll.update(dict_["y_target"], dict_["y_current"]) 
                    #CMDS['roll'] = CMDS['roll'] + pid_output_roll
                    list_target_roll.append(dict_["y_target"]-dict_["y_current"])
                    list_rc_roll.append(CMDS['yaw'])
                    list_pid_roll.append([Kp_y, Ki_y, Kd_y])


                    # time
                    l_time = time.time()-start_time
                    list_time.append(l_time)


                    cursor_msg = f'Init tracker is True, {CMDS["throttle"]}, target pos: {dict_["z_target"]}, corrent: {dict_["z_current"]}, {pid_output_throttle}, Target Kp: {Kp_z}'
                    # cursor_msg = f'Init tracker is True, {CMDS["yaw"]}, target pos: {dict_["y_target"]}, corrent: {dict_["y_current"]}, {pid_output_yaw}, Target Kp_y: {Kp_y}'
                

                    # pitch 
                    #CMDS['pitch'] = 1700

                screen.addstr(7, 100, "Start track: {}".format(str(dict_["init_tracker"])), curses.A_BOLD)
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

                        error_type = board.process_armingDisableFlags(board.CONFIG['armingDisableFlags'])
                        if "RXLOSS" in error_type:
                            screen.addstr(5, 50, f"armingDisableFlags: {error_type}, MSP ON")
                            """
                            переключение на msp
                            """
                        else:
                            screen.addstr(5, 50, "armingDisableFlags: {}".format(error_type))
                        screen.clrtoeol()

                        screen.addstr(6, 0, "cpuload: {}".format(board.CONFIG['cpuload']))
                        screen.clrtoeol()
                        screen.addstr(6, 50, "cycleTime: {}".format(board.CONFIG['cycleTime']))
                        screen.clrtoeol()

                        screen.addstr(7, 0, "mode: {}".format(board.CONFIG['mode']))
                        screen.clrtoeol()
                        mode = board.process_mode(board.CONFIG['mode'])
                        if 'MSP OVERRIDE' in mode:
                            autopilot = True
#                            CMDS['aux2'] = 1500
                            screen.addstr(7, 50, "Autopilot ON Flight Mode: {}".format(mode))
                        else:
                            autopilot = False
#                            CMDS['aux2'] = 1000
                            screen.addstr(7, 50, "Autopilot OFF Flight Mode: {}".format(mode))
                        screen.clrtoeol()
                    elif next_msg == 'MSP_MOTOR':
                        screen.addstr(19, 0, "Motor Values: {}".format(board.MOTOR_DATA))
                        screen.clrtoeol()

                    elif next_msg == 'MSP_RC':
                        if board.RC['channels'][7] == 2011:
                            # зафиксировать обьект
                            dict_["controller_init_tracker"] = True
                            CMDS['aux2'] = 1500
                        else:
                            # открепить обьект
                            dict_["controller_init_tracker"] = False
                        if autopilot == False:    
                            CMDS['throttle'] = board.RC['channels'][3]
                            CMDS['yaw'] = board.RC['channels'][2]
                            CMDS['pitch'] = board.RC['channels'][1]
                            CMDS['roll'] = board.RC['channels'][0]  
#                            CMDS['aux2'] = 1500
                        screen.addstr(20, 0, "RC Channels Values: {}".format(board.RC['channels']))
                        screen.addstr(21, 0, f"RC Channels Client: {[CMDS[ki] for ki in CMDS_ORDER]}")
                        screen.clrtoeol()

                    elif next_msg == 'MSP_FEATURE_CONFIG':
                        screen.addstr(22, 0, "C: {}".format(board.FEATURE_CONFIG['features'][3])) 
                        screen.addstr(23, 0, "C: {}".format(board.FEATURE_CONFIG['features'][14])) 
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
    
#    k_scale = 1.4
    k_scale = 0.8
    cap = cv2.VideoCapture(0)
    print("Ожидание подключения клиента...")
    payload_size = struct.calcsize("Q")
    data = b""
    init_tracker = False
    client_socket = False
    size_box = 70
    pressed_activate_key_track = 0
    while True:
        client_socket, addr = server_socket.accept()
        #print('Получено соединение от:', addr, client_socket)
        try:
           while(cap.isOpened()):
#                try:
                (status, frame) = cap.read()
                if status:
                    frame = cv2.resize(frame, (int(frame.shape[1]*k_scale), int(frame.shape[0]*k_scale)))
                    _img, obj_center, img_center = lib_start.process_img_server(frame, dict_["init_tracker"])  

                    area_OIU = [img_center[0]-size_box, img_center[1]-size_box, img_center[0]+size_box, img_center[1]+size_box]
                    area_OIU = [int(d) for d in area_OIU]
                    bbox_OIU = [area_OIU[0], area_OIU[1], area_OIU[2]-area_OIU[0], area_OIU[3]-area_OIU[1]]
                    
                    # Сжатие кадра в формат JPEG
                    _, img = cv2.imencode('.jpg', _img, encode_param)
                    
                    dict_["y_current"] = img_center[0]
                    dict_["z_current"] = img_center[1]
                    # Координаты центара обьекта
                    dict_["y_target"] = obj_center[0]
                    dict_["z_target"] = obj_center[1]
                    num.value = obj_center[1]
                    
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

                    # включение выключение слежения cmd 
                    if state > 1:
                        if sum(bbox[-2:]) > 10:
                            lib_start.init_tracker(_img, bbox, A = True, B = True)
                            lib_start.state = 0
                            init_tracker = True
                            
                    # включение выключение слежения с пульта        
                    if dict_["controller_init_tracker"]:
                        pressed_activate_key_track += 1
                        if pressed_activate_key_track == 1:
                            lib_start.init_tracker(_img, bbox_OIU, A = True, B = True)
                            init_tracker = True
                    else:
                        if pressed_activate_key_track > 2:
                            pressed_activate_key_track = 0
                            init_tracker = False
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
        dict_["controller_init_tracker"] = False
        # run the thread
        thread1 = Process(target=run_curses, args=(dict_,), daemon=True)              
        thread1.start() 
                
        thread2 = Process(target=image_task, args=(dict_,), daemon=True)
        thread2.start() 
        
        # wait for the thread to finish
        print('Waiting for the thread...')
        thread1.join()  
        thread2.join()    
        

  

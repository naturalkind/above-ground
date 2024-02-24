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


# video stream
class VideoStreamWidget(object):
    def __init__(self, src=0):
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
                self.img, self.obj_center, self.img_center = lib_start.process_img_server(self.frame)   
            time.sleep(.01)
    def get_frame(self):
        return self.img, self.obj_center, self.img_center

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
SERIAL_PORT = "/dev/ttyACM0"

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

def keyboard_controller(screen, dict_):
    Kp = 0.745136137394194487 /100#*20
    Ki = 0.000022393195314520642 /100#*20
    Kd = 7.404490165264038 /100#*60
    
    # Create a PID controller object throttle
    pid_throttle = PIDController(Kp, Ki, Kd) # throttle
    pid_pitch = PIDController(Kp, Ki, Kd) 
    pid_roll = PIDController(Kp, Ki, Kd)
    
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
            while True:
                start_time = time.time()

                char = screen.getch() # get keypress
                curses.flushinp() # flushes buffer
                
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
                    CMDS['aux2'] = 1000
                    start_fly = False
                    start_track = False

                elif char == ord('r') or char == ord('R'):
                    screen.addstr(3, 0, 'Sending Reboot command...')
                    screen.clrtoeol()
                    board.reboot()
                    time.sleep(0.5)
                    break

                elif char == ord('a') or char == ord('A'):
                    cursor_msg = 'Sending Arm command...'
                    CMDS['aux2'] = 1500
                elif char == 259:
                    height += 0.1
                    cursor_msg = f'Target altitude: {height}' 
                elif char == 258:
                    height -= 0.1
                    cursor_msg = f'Target altitude: {height}'
                elif char == ord('s') or char == ord('S'):
                    if ARMED:
                        start_fly = True
                        cursor_msg = f'Start flight...'
                elif char == ord('z') or char == ord('Z'):
                    if ARMED:
                        start_track = True
                        cursor_msg = f'Start flight...' 
                #
                # IMPORTANT MESSAGES (CTRL_LOOP_TIME based)
                #
                if (time.time()-last_loop_time) >= CTRL_LOOP_TIME:
                    last_loop_time = time.time()
                    # Send the RC channel values to the FC
                    if board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER]):
                        dataHandler = board.receive_msg()
                        board.process_recv_data(dataHandler)
                local_fast_read_imu() 
                local_fast_read_attitude()
                local_fast_read_altitude()
                
                #
                # SLOW MSG processing (user GUI)
                #
                #board.SENSOR_DATA
                dict_["current_height"] = board.SENSOR_DATA['altitude']
                dict_["rotation"] = board.SENSOR_DATA['kinematics']
                
                if start_fly:
                    cursor_msg = f'start flight mode {CMDS["throttle"]}'
                    #pid_output_throttle = pid_throttle.update(height, dict_["current_height"])
                    pid_output_throttle = pid_throttle.update(dict_["current_height"], height)
#                    if 1000 <= CMDS['throttle'] + pid_output_throttle  <= 1700:
                    if CMDS['throttle'] <= 1680 or CMDS['throttle'] <= 1000:
                        CMDS['throttle'] = CMDS['throttle'] + pid_output_throttle 
                    time.sleep(0.001)
                if dict_["init_tracker"]:
                    if ARMED and start_track:
#                        pid_output_roll = pid_roll.update(dict_["y_target"], dict_["y_current"]) # yaw
#                        if 1000 <= CMDS['yaw']+ pid_output_roll <= 1680:
#                            CMDS['yaw'] = CMDS['yaw'] + pid_output_roll

                        cursor_msg = f'Init tracker is True, {CMDS["throttle"]}, target pos: {dict_["z_target"]}, corrent: {dict_["z_current"]}'
                        #cursor_msg = f'{dict_["z_current"]}, {dict_["z_target"]}, {dict_["y_current"]}, {dict_["y_target"]}, {num.value}}'
                        pid_output_throttle = pid_throttle.update(dict_["z_target"], dict_["z_current"])        
                        #if 1000 <= CMDS['throttle']+pid_output_throttle <= 1680:
                        if CMDS['throttle'] <= 1680 or CMDS['throttle'] <= 1000:
                            CMDS['throttle'] = CMDS['throttle'] + pid_output_throttle 
                        #time.sleep(0.001)
                screen.addstr(5, 100, "Altitude: {}".format(dict_["current_height"]))
                screen.clrtoeol()      
                screen.addstr(6, 100, "Kinematics: {}".format(dict_["rotation"]))
                screen.clrtoeol()   
                screen.addstr(7, 100, "Start flight: {}".format(str(start_fly)), curses.A_BOLD)
                screen.clrtoeol()  
                screen.addstr(8, 100, "Target height: {}".format(str(height)), curses.A_BOLD)
                screen.clrtoeol()  
               
                screen.addstr(8, 0, "init_tracker: {}".format(dict_["init_tracker"]))
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
    print("Ожидание подключения клиента...")
    cap = cv2.VideoCapture(1)
    # video_stream_widget = VideoStreamWidget()

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
#                try:
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
    #                except AttributeError:
    #                    pass
                    num.value = obj_center[1]
                    #time.sleep(0.04)
                    #print (dict_, num.value)  
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
    num = Value('d', 0.0)
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
        #thread1.join()  !!!
        thread2.join()    
        
        
"""
1 запускать автоматически 
2 при нажатии армить
3 при нажатии запускать выбор цели

"""    

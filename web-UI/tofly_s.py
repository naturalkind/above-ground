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
                self.img, self.obj_center, self.img_center = lib_start.process_img_server(self.frame)   
            time.sleep(.1)
            #time.sleep(.01)
    def get_frame(self):
        return self.img, self.obj_center, self.img_center

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
#cap = cv2.VideoCapture(1)
video_stream_widget = VideoStreamWidget()

payload_size = struct.calcsize("Q")
data = b""
init_tracker = False
client_socket = False


Kp = 0.745136137394194487*20
Ki = 0.00022393195314520642*20
Kd = 7.404490165264038*60

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

ixx = 0
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
        while(video_stream_widget.capture.isOpened()):
            if client_socket:
                try:
                    # Сжатие кадра в формат JPEG
                    _img, obj_center, img_center = video_stream_widget.get_frame()
                    _, img = cv2.imencode('.jpg', _img, encode_param)
                except AttributeError:
                    pass
                if init_tracker == False:
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
                msg_size = struct.unpack("Q",packed_msg_size)[0]
                
                while len(data) < msg_size:
                    data += client_socket.recv(4*1024)
                frame_data = data[:msg_size]
                data = data[msg_size:]
                bbox, state, init_switch = pickle.loads(frame_data)  
                
                #print (state, init_switch, init_tracker)
                if state > 1 and init_switch is True:
                    if init_tracker == False:
                        lib_start.init_tracker(_img, bbox)
                    init_tracker = True
                  
                
                if init_tracker == True:
                    a = pickle.dumps([img, init_tracker])
                    message = struct.pack("Q", len(a)) + a
                    client_socket.sendall(message)

        
                start_time = time.time()

                if ixx > 40:
                    CMDS['aux2'] = 1500
                    start_fly = True
                    
                print (img_center[0], obj_center[0], ixx)
                if start_fly:
                    if init_tracker:
                        pid_output_roll = pid_roll.update(obj_center[0], img_center[0]) # yaw
                        if 1000 <= CMDS['yaw']+ pid_output_roll <= 1680:
                            print ("YAW", pid_output_roll, CMDS['yaw'], CMDS['yaw'] + pid_output_roll)
                            CMDS['yaw'] = CMDS['yaw'] + pid_output_roll
                            
                        pid_output_throttle = pid_throttle.update(obj_center[1], img_center[1])        
                        if 1000 <= CMDS['throttle']+pid_output_throttle <= 1680:
                            print ("THRO", pid_output_throttle, CMDS['throttle'], CMDS['throttle']+pid_output_throttle)
                            CMDS['throttle'] = CMDS['throttle'] + pid_output_throttle 
                ixx += 1
                time.sleep(0.0001)
                
                
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
            else:
                print ('Connecting to the FC... connected!')
                client_socket, addr = server_socket.accept()
                print('Получено соединение от:', addr, client_socket)
finally:
   #print ("ERROR") 
   pass
    
    
    
    
#{'apiVersion': '1.45.0', 'flightControllerIdentifier': 'BTFL', 'flightControllerVersion': '4.4.3', 'version': 0, 'buildInfo': 'Nov 14 2023 16:10:10', 'multiType': 0, 'msp_version': 0, 'capability': 0, 'cycleTime': 313, 'i2cError': 0, 'activeSensors': 33, 'mode': 0, 'profile': 0, 'uid': [2883621, 808666896, 943206708], 'accelerometerTrims': [0, 0], 'name': '', 'displayName': 'JOE PILOT', 'numProfiles': 4, 'rateProfile': 0, 'boardType': 2, 'armingDisableCount': 26, 'armingDisableFlags': 0, 'armingDisabled': False, 'runawayTakeoffPreventionDisabled': False, 'boardIdentifier': 'S405', 'boardVersion': 0, 'commCapabilities': 55, 'targetName': 'STM32F405', 'boardName': 'SPEEDYBEEF405V3', 'manufacturerId': 'SPBE', 'signature': [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 'mcuTypeId': 1, 'mspProtocolVersion': 0, 'cpuload': 18, 'flightModeFlags': []}
#['ADJUSTMENT_RANGES', 'ADVANCED_TUNING', 'ANALOG', 'ARMING_CONFIG', 'AUX_CONFIG', 'AUX_CONFIG_IDS', 'BATTERY_CONFIG', 'BATTERY_STATE', 'BAUD_RATES', 'BEEPER_CONFIG', 'BLACKBOX', 'BOARD_ALIGNMENT_CONFIG', 'COMPASS_CONFIG', 'CONFIG', 'CURRENT_METERS', 'CURRENT_METER_CONFIGS', 'DATAFLASH', 'FAILSAFE_CONFIG', 'FC_CONFIG', 'FEATURE_CONFIG', 'FILTER_CONFIG', 'GPS_CONFIG', 'GPS_DATA', 'GPS_RESCUE', 'INAV', 'JUMBO_FRAME_SIZE_LIMIT', 'MISC', 'MIXER_CONFIG', 'MODE_RANGES', 'MODE_RANGES_EXTRA', 'MOTOR_3D_CONFIG', 'MOTOR_CONFIG', 'MOTOR_DATA', 'MSPCodes', 'MSPCodes2Str', 'PID', 'PIDNAMES', 'PID_ADVANCED_CONFIG', 'PIDs', 'RC', 'RC_DEADBAND_CONFIG', 'RC_MAP', 'RC_TUNING', 'REBOOT_TYPES', 'RSSI_CONFIG', 'RXFAIL_CONFIG', 'RX_CONFIG', 'SDCARD', 'SENSOR_ALIGNMENT', 'SENSOR_CONFIG', 'SENSOR_DATA', 'SERIAL_CONFIG', 'SERIAL_PORT_FUNCTIONS', 'SERVO_CONFIG', 'SERVO_DATA', 'SIGNATURE_LENGTH', 'TRANSPONDER', 'VOLTAGE_METERS', 'VOLTAGE_METER_CONFIGS', '__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__enter__', '__eq__', '__exit__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_crc8_dvb_s2', 'armingDisableFlagNames_BF', 'armingDisableFlagNames_INAV', 'basic_info', 'bit_check', 'conn', 'connect', 'convert', 'dataHandler_init', 'fast_msp_rc_cmd', 'fast_read_altitude', 'fast_read_analog', 'fast_read_attitude', 'fast_read_imu', 'is_serial_open', 'process_MSP2_INAV_DEBUG', 'process_MSP2_PID', 'process_MSPV2_INAV_ANALOG', 'process_MSPV2_INAV_MISC', 'process_MSP_ACC_CALIBRATION', 'process_MSP_ACC_TRIM', 'process_MSP_ADJUSTMENT_RANGES', 'process_MSP_ADVANCED_CONFIG', 'process_MSP_ALTITUDE', 'process_MSP_ANALOG', 'process_MSP_API_VERSION', 'process_MSP_ARMING_CONFIG', 'process_MSP_ARMING_DISABLE', 'process_MSP_ATTITUDE', 'process_MSP_BATTERY_CONFIG', 'process_MSP_BATTERY_STATE', 'process_MSP_BEEPER_CONFIG', 'process_MSP_BLACKBOX_CONFIG', 'process_MSP_BOARD_ALIGNMENT_CONFIG', 'process_MSP_BOARD_INFO', 'process_MSP_BOXIDS', 'process_MSP_BOXNAMES', 'process_MSP_BUILD_INFO', 'process_MSP_CF_SERIAL_CONFIG', 'process_MSP_COMPASS_CONFIG', 'process_MSP_COMP_GPS', 'process_MSP_COPY_PROFILE', 'process_MSP_CURRENT_METERS', 'process_MSP_CURRENT_METER_CONFIG', 'process_MSP_DATAFLASH_ERASE', 'process_MSP_DATAFLASH_SUMMARY', 'process_MSP_DEBUG', 'process_MSP_EEPROM_WRITE', 'process_MSP_FAILSAFE_CONFIG', 'process_MSP_FC_VARIANT', 'process_MSP_FC_VERSION', 'process_MSP_FEATURE_CONFIG', 'process_MSP_FILTER_CONFIG', 'process_MSP_GPSSTATISTICS', 'process_MSP_GPS_CONFIG', 'process_MSP_GPS_RESCUE', 'process_MSP_GPS_SV_INFO', 'process_MSP_LOOP_TIME', 'process_MSP_MAG_CALIBRATION', 'process_MSP_MISC', 'process_MSP_MIXER_CONFIG', 'process_MSP_MODE_RANGES', 'process_MSP_MODE_RANGES_EXTRA', 'process_MSP_MOTOR', 'process_MSP_MOTOR_3D_CONFIG', 'process_MSP_MOTOR_CONFIG', 'process_MSP_NAME', 'process_MSP_OSD_CHAR_READ', 'process_MSP_OSD_CHAR_WRITE', 'process_MSP_OSD_CONFIG', 'process_MSP_PID', 'process_MSP_PIDNAMES', 'process_MSP_PID_ADVANCED', 'process_MSP_PID_CONTROLLER', 'process_MSP_RAW_GPS', 'process_MSP_RAW_IMU', 'process_MSP_RC', 'process_MSP_RC_DEADBAND', 'process_MSP_RC_TUNING', 'process_MSP_RESET_CONF', 'process_MSP_RSSI_CONFIG', 'process_MSP_RXFAIL_CONFIG', 'process_MSP_RX_CONFIG', 'process_MSP_RX_MAP', 'process_MSP_SDCARD_SUMMARY', 'process_MSP_SELECT_SETTING', 'process_MSP_SENSOR_ALIGNMENT', 'process_MSP_SENSOR_CONFIG', 'process_MSP_SERVO', 'process_MSP_SERVO_CONFIGURATIONS', 'process_MSP_SET_ACC_TRIM', 'process_MSP_SET_ADJUSTMENT_RANGE', 'process_MSP_SET_ADVANCED_CONFIG', 'process_MSP_SET_ARMING_CONFIG', 'process_MSP_SET_BEEPER_CONFIG', 'process_MSP_SET_BLACKBOX_CONFIG', 'process_MSP_SET_BOARD_ALIGNMENT_CONFIG', 'process_MSP_SET_CF_SERIAL_CONFIG', 'process_MSP_SET_CURRENT_METER_CONFIG', 'process_MSP_SET_FAILSAFE_CONFIG', 'process_MSP_SET_FEATURE_CONFIG', 'process_MSP_SET_FILTER_CONFIG', 'process_MSP_SET_GPS_CONFIG', 'process_MSP_SET_LOOP_TIME', 'process_MSP_SET_MIXER_CONFIG', 'process_MSP_SET_MODE_RANGE', 'process_MSP_SET_MOTOR', 'process_MSP_SET_MOTOR_3D_CONFIG', 'process_MSP_SET_MOTOR_CONFIG', 'process_MSP_SET_NAME', 'process_MSP_SET_OSD_CONFIG', 'process_MSP_SET_PID', 'process_MSP_SET_PID_ADVANCED', 'process_MSP_SET_PID_CONTROLLER', 'process_MSP_SET_RAW_RC', 'process_MSP_SET_RC_DEADBAND', 'process_MSP_SET_RC_TUNING', 'process_MSP_SET_REBOOT', 'process_MSP_SET_RESET_CURR_PID', 'process_MSP_SET_RSSI_CONFIG', 'process_MSP_SET_RTC', 'process_MSP_SET_RXFAIL_CONFIG', 'process_MSP_SET_RX_CONFIG', 'process_MSP_SET_RX_MAP', 'process_MSP_SET_SENSOR_ALIGNMENT', 'process_MSP_SET_SENSOR_CONFIG', 'process_MSP_SET_SERVO_CONFIGURATION', 'process_MSP_SET_TRANSPONDER_CONFIG', 'process_MSP_SET_VOLTAGE_METER_CONFIG', 'process_MSP_SET_VTX_CONFIG', 'process_MSP_SONAR', 'process_MSP_STATUS', 'process_MSP_STATUS_EX', 'process_MSP_UID', 'process_MSP_VOLTAGE_METERS', 'process_MSP_VOLTAGE_METER_CONFIG', 'process_MSP_VTX_CONFIG', 'process_armingDisableFlags', 'process_mode', 'process_recv_data', 'readbytes', 'reboot', 'receive_msg', 'receive_raw_msg', 'save2eprom', 'send_RAW_MOTORS', 'send_RAW_RC', 'send_RAW_msg', 'ser_trials', 'serialPortFunctionMaskToFunctions', 'serial_port_read_lock', 'serial_port_write_lock', 'set_ARMING_DISABLE', 'set_FEATURE_CONFIG', 'set_RX_MAP']
    
    
    
    
    
    
    
    
    
    
    
    
    
    

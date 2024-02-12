import cv2
import socket
import pickle
import struct
import time
import tracker_lib
from yamspy import MSPy

# Max periods for:
CTRL_LOOP_TIME = 1/100
SLOW_MSGS_LOOP_TIME = 1/5 

SERIAL_PORT = "/dev/ttyACM0"

lib_start = tracker_lib.TrackerLib()
# Создание сокета
server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
host_name = socket.gethostname()
#host_ip = #'10.42.0.1'
#host_ip ='192.168.1.123'#socket.gethostbyname(host_name)
host_ip ='192.168.0.102'
print('Хост IP:', host_ip)
port = 9999
socket_address = (host_ip, port)

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

CMDS = {
        'roll':     1500,
        'pitch':    1500,
        'throttle': 1000,
        'yaw':      1500,
        'aux1':     1000,
        'aux2':     1000
        }

CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2']
start_fly = False
ixx = 0

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
ARMED = None
with MSPy(device=SERIAL_PORT, loglevel='WARNING', baudrate=115200) as board:
    local_fast_read_attitude = board.fast_read_attitude
    local_fast_read_imu = board.fast_read_imu
    local_fast_read_altitude = board.fast_read_altitude
    last_loop_time = last_slow_msg_time = last_cycleTime = time.time()
    while True:
        start_time = time.time()
        if board == 1: # an error occurred...
            print ("ERROR")
            break
        CMDS['aux2'] = 1500
        if (time.time()-last_loop_time) >= CTRL_LOOP_TIME:
            last_loop_time = time.time()
            # Send the RC channel values to the FC
#            if board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER]):
#                dataHandler = board.receive_msg()
#                board.process_recv_data(dataHandler)  
                 
            print ("...........", board.SENSOR_DATA['altitude'], [CMDS[ki] for ki in CMDS_ORDER], ARMED)     
        local_fast_read_imu() 
        local_fast_read_attitude()
        local_fast_read_altitude()
        board.fast_msp_rc_cmd([CMDS[ki] for ki in CMDS_ORDER])
        time.sleep(0.1)
        end_time = time.time()
        
        if (time.time()-last_slow_msg_time) >= SLOW_MSGS_LOOP_TIME:
            last_slow_msg_time = time.time()
            ARMED = board.bit_check(board.CONFIG['mode'],0)
        last_cycleTime = end_time-start_time
        if (end_time-start_time)<CTRL_LOOP_TIME:
            time.sleep(CTRL_LOOP_TIME-(end_time-start_time))
     

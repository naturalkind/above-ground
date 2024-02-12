import cv2
import socket
import pickle
import struct
import time
import tracker_lib
from yamspy import MSPy

# Max periods for:
CTRL_LOOP_TIME = 1/100

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
with MSPy(device=SERIAL_PORT, loglevel='WARNING', baudrate=115200) as board:

    last_loop_time = last_cycleTime = time.time()
    while True:
        if board == 1: # an error occurred...
            print ("ERROR")
            break
        print ('Connecting to the FC... connected!')
        client_socket, addr = server_socket.accept()
        print('Получено соединение от:', addr)
    
        if client_socket:
            
            local_fast_read_attitude = board.fast_read_attitude
            local_fast_read_imu = board.fast_read_imu
            local_fast_read_altitude = board.fast_read_altitude
            while(cap.isOpened()):
                success, _img = cap.read()
                start_time = time.time()
                if init_tracker == False:
                    # отправка данных
                    a = pickle.dumps([_img, init_tracker])
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
                img, obj_center, img_center = lib_start.process_img_server(_img)     
                
                                
                if init_tracker == True:
                    a = pickle.dumps([img, init_tracker])
                    message = struct.pack("Q", len(a)) + a
                    client_socket.sendall(message)
                    
                if start_fly:
                    if ixx > 400:
                        pid_output_roll = pid_roll.update(img_center[0], obj_center[0]) # yaw
                        if CMDS['yaw'] <= 1680 or CMDS['yaw'] <= 1000:
                            CMDS['yaw'] = CMDS['yaw'] + pid_output_roll
                else:
                    print ("START ARM")
                    CMDS['aux2'] = 1500
                    start_fly = True
    #                if start_fly:
    #                    if ixx > 200:
    #                        pid_output_throttle = pid_throttle.update(board.SENSOR_DATA['altitude'], 0.3)
    #                        if CMDS['throttle'] <= 1680 or CMDS['throttle'] <= 1000:
    #                            CMDS['throttle'] = CMDS['throttle'] + pid_output_throttle 
    #                        pid_output_pitch = pid_pitch.update(board.SENSOR_DATA['kinematics'][1] , 0.0)
    #                        pid_output_roll = pid_roll.update(board.SENSOR_DATA['kinematics'][0], 0.0)
    #                        if CMDS['pitch'] <= 1680 or CMDS['pitch'] <= 1000:
    #                            CMDS['pitch'] = CMDS['pitch'] + pid_output_pitch
    #                            #cursor_msg = ' pitch: {}'.format(CMDS['pitch'])
    #                        if CMDS['roll'] <= 1680 or CMDS['roll'] <= 1000:
    #                            CMDS['roll'] = CMDS['roll'] + pid_output_roll
    #                            #cursor_msg = ' roll: {}'.format(CMDS['roll'])
    #                    ixx += 1
                #time.sleep(0.0001)
                if (time.time()-last_loop_time) >= CTRL_LOOP_TIME:
                    last_loop_time = time.time()
                    ARMED = board.bit_check(board.CONFIG['mode'],0)
                    
                    # Send the RC channel values to the FC
                    if board.send_RAW_RC([CMDS[ki] for ki in CMDS_ORDER]):
                        dataHandler = board.receive_msg()
                        print ("---->", CMDS, board.SENSOR_DATA['altitude'], ARMED)
                        # dataHandler
                        board.process_recv_data(dataHandler)
                local_fast_read_imu() 
                local_fast_read_attitude()
                local_fast_read_altitude()  
                
                end_time = time.time()
                last_cycleTime = end_time-start_time
                if (end_time-start_time)<CTRL_LOOP_TIME:
                    print ("AAAAA")
                    time.sleep(CTRL_LOOP_TIME-(end_time-start_time))              
                # визуализация
    #            cv2.imshow("win", img)
    #            if cv2.waitKey(1) & 0xff == ord('q'):
    #                client_socket.close()
    #                break 
                    
                    
            cap.release()
            cv2.destroyAllWindows()

"""
1 запускать автоматически 
2 при нажатии армить
3 при нажатии запускать выбор цели

"""  
import contextlib
import io
import os
import sys
import cv2
import time
import json
import curses
import socket
import pickle
import struct
import numpy as np
import queue
from tracker_lib import tracker_lib
from multiprocessing import Process, Value, Array, Manager, Queue
from collections import deque
from itertools import cycle
from yamspy import MSPy
from threading import Thread, Lock
from filterpy.memory import FadingMemoryFilter
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from scipy.signal import argrelextrema
from concurrent.futures import ThreadPoolExecutor
import VL53L0X
import logging
from logging.handlers import RotatingFileHandler
from scipy.optimize import curve_fit

# Настройка логирования в файл
log_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
log_file = 'drone_system.log'
log_handler = RotatingFileHandler(log_file, maxBytes=5*1024*1024, backupCount=2)
log_handler.setFormatter(log_formatter)
logger = logging.getLogger('DroneSystem')
logger.setLevel(logging.INFO)
logger.addHandler(log_handler)

# ----------------------------->

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

def sensor_process(dict_):
    sensor = VL53L0X.VL53L0XSensor()
    sensor.start()
    try:
        while True:
            data = sensor.get_data()
            if data:
                msg_type, content = data
                if msg_type == "RESULT":
                    dict_['filtered_distance'] = content['filtered_distance']
                    logger.info(f"Sensor distance: {content['filtered_distance']:.2f}")
            time.sleep(0.005)
    except KeyboardInterrupt:
        sensor.stop()
    except Exception as e:
        logger.error(f"Error in sensor process: {str(e)}")
    finally:
        sensor.stop()

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

    def update(self, current_value, target_value, dt):
        error = target_value - current_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error 
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return output


##########################


class PrioritizedExperience:
    def __init__(self, capacity=1000):
        self.buffer = deque(maxlen=capacity)
        self.priorities = deque(maxlen=capacity)

    def add(self, experience, priority):
        self.buffer.append(experience)
        self.priorities.append(priority)

    def sample(self, batch_size):
        probs = np.array(self.priorities) / sum(self.priorities)
        indices = np.random.choice(len(self.buffer), batch_size, p=probs)
        samples = [self.buffer[i] for i in indices]
        return samples

class AdamOptimizer:
    def __init__(self, learning_rate=0.001, beta1=0.9, beta2=0.999, epsilon=1e-8):
        self.lr = learning_rate
        self.beta1 = beta1
        self.beta2 = beta2
        self.epsilon = epsilon
        self.m = 0
        self.v = 0
        self.t = 0

    def update(self, param, gradient):
        self.t += 1
        self.m = self.beta1 * self.m + (1 - self.beta1) * gradient
        self.v = self.beta2 * self.v + (1 - self.beta2) * (gradient ** 2)
        m_hat = self.m / (1 - self.beta1 ** self.t)
        v_hat = self.v / (1 - self.beta2 ** self.t)
        param -= self.lr * m_hat / (np.sqrt(v_hat) + self.epsilon)
        return param

class AdaptivePIDController:
    def __init__(self, kp=1.0, ki=0.1, kd=0.05, save_path='pid_params.json'):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error_sum = 0
        self.last_error = 0
        self.last_time = time.time()
        
        self.save_path = save_path
        self.load_parameters()
        
        # Параметры экстремального поиска
        self.a = 0.1
        self.omega = 1.0
        self.gamma = 0.01
        
        # Параметры обучения с подкреплением
        self.q_values = np.zeros((3, 3))
        self.epsilon = 0.1
        self.alpha = 0.1
        
        # Параметры адаптивного шага обучения
        self.learning_rate = 0.01
        self.min_learning_rate = 0.001
        self.max_learning_rate = 0.1
        self.performance_window = []
        self.window_size = 50
        
        self.running = True
        self.optimization_interval = 0.1
        self.performance_history = []
        
        # Блокировки для синхронизации
        self.optimization_lock = Lock()
        self.learning_params_lock = Lock()
        self.learning_rate_lock = Lock()
        
        # Приоритетный опыт
        self.experience_buffer = PrioritizedExperience()
        
        # Adam оптимизаторы
        self.adam_kp = AdamOptimizer()
        self.adam_ki = AdamOptimizer()
        self.adam_kd = AdamOptimizer()
        
        # Асинхронная оптимизация
        self.executor = ThreadPoolExecutor(max_workers=3)
        
        self.optimization_thread = Thread(target=self.optimize_parameters)
        self.optimization_thread.start()

    def compute(self, current_value, target_value, dt):
        try:
            current_time = time.time()
            dt = current_time - self.last_time
            
            error = target_value - current_value
            self.error_sum += error * dt
            error_diff = (error - self.last_error) / dt if dt > 0 else 0
            
            output = (self.kp * error +
                      self.ki * self.error_sum +
                      self.kd * error_diff)
            
            self.last_error = error
            self.last_time = current_time
            
            return output
        except Exception as e:
            print(f"Error in compute method: {e}")
            return 0

    def adapt_learning_rate(self):
        with self.learning_rate_lock:
            if len(self.performance_window) >= self.window_size:
                performance_trend = np.mean(self.performance_window[-10:]) - np.mean(self.performance_window[:10])
                if performance_trend > 0:
                    self.learning_rate = min(self.max_learning_rate, self.learning_rate * 1.05)
                else:
                    self.learning_rate = max(self.min_learning_rate, self.learning_rate * 0.95)

    def adapt_optimization_frequency(self):
        with self.optimization_lock:
            recent_performance = np.mean(self.performance_history[-10:])
            if recent_performance > -0.05:  # Если производительность хорошая
                self.optimization_interval = min(1.0, self.optimization_interval * 1.1)
            else:
                self.optimization_interval = max(0.01, self.optimization_interval * 0.9)

    def adapt_window_size(self):
        with self.learning_rate_lock:
            if len(self.performance_history) > 100:
                performance_variance = np.var(self.performance_history[-100:])
                if performance_variance < 0.001:
                    self.window_size = max(10, self.window_size - 1)
                else:
                    self.window_size = min(100, self.window_size + 1)

    def adapt_extremum_seeking_params(self):
        with self.optimization_lock:
            recent_performance = np.mean(self.performance_history[-20:])
            if recent_performance > -0.01:
                self.a *= 0.95  # Уменьшаем амплитуду возмущения
                self.omega *= 1.05  # Увеличиваем частоту возмущения
            else:
                self.a = min(0.5, self.a * 1.05)
                self.omega = max(0.1, self.omega * 0.95)

    def update_performance_history(self, performance):
        alpha = 0.1  # Коэффициент сглаживания
        if not self.performance_history:
            self.performance_history.append(performance)
        else:
            smoothed_performance = alpha * performance + (1 - alpha) * self.performance_history[-1]
            self.performance_history.append(smoothed_performance)

    def adapt_q_learning_rate(self):
        with self.learning_params_lock:
            recent_performance = np.mean(self.performance_history[-20:])
            if recent_performance > -0.01:
                self.alpha = max(0.01, self.alpha * 0.99)
            else:
                self.alpha = min(0.5, self.alpha * 1.01)

    def async_optimize(self):
        self.executor.submit(self.adapt_learning_rate)
        self.executor.submit(self.adapt_optimization_frequency)
        self.executor.submit(self.adapt_extremum_seeking_params)
        self.executor.submit(self.adapt_window_size)
        self.executor.submit(self.adapt_q_learning_rate)

    def optimize_parameters(self):
        t = 0
        while self.running:
            try:
                # Экстремальный поиск для Kp
                perturbation = self.a * np.sin(self.omega * t)
                gradient_kp = self.last_error * perturbation
                self.kp = self.adam_kp.update(self.kp, gradient_kp)
                
                # Измерение производительности
                performance = -abs(self.last_error)
                self.update_performance_history(performance)
                self.performance_window.append(performance)
                if len(self.performance_window) > self.window_size:
                    self.performance_window.pop(0)
                
                # Q-learning для Kp, Ki и Kd
                for param_index, param in enumerate(['kp', 'ki', 'kd']):
                    if np.random.random() < self.epsilon:
                        action = np.random.choice(3)
                    else:
                        action = np.argmax(self.q_values[param_index])
                    
                    old_value = getattr(self, param)
                    with self.learning_rate_lock:
                        if action == 0:
                            setattr(self, param, old_value * (1 - self.learning_rate))
                        elif action == 2:
                            setattr(self, param, old_value * (1 + self.learning_rate))
                    
                    new_performance = -abs(self.last_error)
                    reward = new_performance - performance
                    
                    experience = (param_index, action, reward)
                    priority = abs(reward) + 0.01
                    self.experience_buffer.add(experience, priority)
                
                if len(self.experience_buffer.buffer) >= 32:
                    batch = self.experience_buffer.sample(32)
                    for exp in batch:
                        param_index, action, reward = exp
                        old_q = self.q_values[param_index][action]
                        self.q_values[param_index][action] += self.alpha * (reward - old_q)
                
                self.async_optimize()

                t += self.optimization_interval
                time.sleep(self.optimization_interval)
                
                # Сохранение параметров
                if t % 10 == 0:
                    self.save_parameters()
            
            except Exception as e:
                #print(f"Error in optimization loop: {e}")
                time.sleep(1)

    def save_parameters(self):
        params = {
            'kp': self.kp,
            'ki': self.ki,
            'kd': self.kd,
            'q_values': self.q_values.tolist(),
            'epsilon': self.epsilon,
            'alpha': self.alpha,
            'learning_rate': self.learning_rate,
            'a': self.a,
            'omega': self.omega,
            'window_size': self.window_size,
            'optimization_interval': self.optimization_interval
        }
        try:
            with open(self.save_path, 'w') as f:
                json.dump(params, f)
        except Exception as e:
            print(f"Error saving parameters: {e}")

    def load_parameters(self):
        if os.path.exists(self.save_path):
            try:
                with open(self.save_path, 'r') as f:
                    params = json.load(f)
                self.kp = params['kp']
                self.ki = params['ki']
                self.kd = params['kd']
                self.q_values = np.array(params['q_values'])
                self.epsilon = params['epsilon']
                self.alpha = params['alpha']
                self.learning_rate = params['learning_rate']
                self.a = params['a']
                self.omega = params['omega']
                self.window_size = params['window_size']
                self.optimization_interval = params['optimization_interval']
                print("Parameters loaded successfully")
            except Exception as e:
                print(f"Error loading parameters: {e}")

    def stop(self):
        self.running = False
        self.optimization_thread.join()
        self.executor.shutdown()
        self.save_parameters()


#################################


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
    # PID up 

    ##########
    # yaw PID
    ##########
    
    # Лучший 3
    # Kp_y = -0.0012 
    # Ki_y = -0.00022 
    # Kd_y = -0.48799709320068358
    Kp_y = -0.0012 
    Ki_y = -0.00022 
    Kd_y = -0.48799709320068358   


    pid_yaw = PIDController(Kp_y, Ki_y, Kd_y) 
    # pid_yaw = AdaptivePIDController(kp=Kp_y, ki=Ki_y, kd=Kd_y)

    ##########
    # roll PID
    ##########
    
    # Kp_y = 0 
    # Ki_y = 0 
    # Kd_y = 0
    pid_roll = PIDController(Kp_y, Ki_y, Kd_y)  
    # pid_roll = AdaptivePIDController(kp=Kp_y, ki=Ki_y, kd=Kd_y)

    ##########
    # throttle PID
    ##########
    Kp_z = 7.0 #42
    Ki_z = 0.3 #5
    Kd_z = 3.0

    # лучший 8  4400Ah
    # Kp_z = 0.00074#42
    # Ki_z = 0.000031#5
    # Kd_z = 0.297


    # лучший 5 5500Ah
#    Kp_z = 0.0005#42
#    Ki_z = 0.00005#5
#    Kd_z = 0.4435

    # genetic PID
    # Kp_z = 0.0048
    # Ki_z = 0.00011
    # Kd_z = 0.22


    # Create a PID controller object throttle
#    pid_throttle = PIDController(Kp_z, Ki_z, Kd_z) 
    pid_throttle = AdaptivePIDController(kp=Kp_z, ki=Ki_z, kd=Kd_z)

#    pid_pitch = PIDController(Kp_z, Ki_z, Kd_z) 
    pid_pitch = AdaptivePIDController(kp=Kp_z, ki=Ki_z, kd=Kd_z)
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
    ########################################################
    # "print" doesn't work with curses, use addstr instead
    ########################################################
    # This order is the important bit: it will depend on how your flight controller is configured.
    # Below it is considering the flight controller is set to use AETR.
    # The names here don't really matter, they just need to match what is used for the CMDS dictionary.
    # In the documentation, iNAV uses CH5, CH6, etc while Betaflight goes aux2, aux3...
    CMDS_ORDER = ['roll', 'pitch', 'throttle', 'yaw', 'aux1', 'aux2', 'aux3', 'aux4']
    autopilot = False
    ARMED = False
    height = 0.0
    filtered_distance = 0
    
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
            cursor_msg1 = ""
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

            # pitch
            list_target_pitch = []
            list_rc_pitch = []
            list_pid_pitch = [] 

            # time
            list_time = []
            while True:
                start_time = time.time()
                char = screen.getch() # get keypress
                curses.flushinp() # flushes buffer
                

                local_fast_read_imu() 
                local_fast_read_attitude()
                local_fast_read_altitude()
   
                filtered_distance = dict_["filtered_distance"]
   
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

                    # Создаем директорию, если она не существует
                    directory = './data/pid_data/'
                    os.makedirs(directory, exist_ok=True)

                    # Теперь открываем файл для записи
                    file_path = os.path.join(directory, f'data_{current_date}.json')
                    try:
                        with open(file_path, 'w') as f:
                            data = {
                                "list_time": list_time,
                                "list_rc_thr": list_rc_thr,
                                "list_target_thr": list_target_thr,
                                "list_pid_thr": list_pid_thr,
                                "list_target_yaw": list_target_yaw,
                                "list_rc_yaw": list_rc_yaw,
                                "list_pid_yaw": list_pid_yaw,
                                "list_target_roll": list_target_roll,
                                "list_rc_roll": list_rc_roll,
                                "list_pid_roll": list_pid_roll
                            }
                            json.dump(data, f)
                        cursor_msg = f'Data saved to {file_path}'
                        cursor_msg1 = ""
                    except Exception as e:
                        cursor_msg = f'Error saving data: {str(e)}'


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
                    # pid_ = PIDController(Kp, Ki, Kd) 
                    
                    height += 10
                    
                elif char == 258:
                    # Kp -= 0.001
                    # Create a PID controller object throttle
                    # pid_ = PIDController(Kp, Ki, Kd) 
                    
                    height -= 10

    
                    
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
                      
                    dt = time.time()-start_time  
                    # THROTTLE
                    
#                    pid_output_throttle = pid_throttle.update(dict_["z_target"], dict_["z_current"], dt)   
#                    pid_output_throttle = pid_throttle.compute(dict_["z_target"], dict_["z_current"], dt)  

                    # VL53L0X altitude
                    pid_output_throttle = pid_throttle.compute(filtered_distance, height, dt)  
                    
                    # Управление дроном
                    CMDS['throttle'] = int(np.clip(1500 + pid_output_throttle, 1000, 1800))  # Базовое значение 1500 для подьема 



                    list_target_thr.append(dict_["z_target"]-dict_["z_current"])
                    list_rc_thr.append(CMDS['throttle'])
                    list_pid_thr.append([Kp_z, Ki_z, Kd_z])

                    # YAW

                    pid_output_yaw = pid_yaw.update(dict_["y_target"], dict_["y_current"], dt)
                    #CMDS['yaw'] = CMDS['yaw'] + int(np.clip(pid_output_yaw, -500, 500))
                    
                    # pid_output_yaw = pid_yaw.compute(dict_["y_target"], dict_["y_current"], dt) 
                    #CMDS['yaw'] = CMDS['yaw'] + pid_output_yaw
                    

                    list_target_yaw.append(dict_["y_target"]-dict_["y_current"])
                    list_rc_yaw.append(CMDS['yaw'])
                    list_pid_yaw.append([Kp_y, Ki_y, Kd_y])
                    # ROLL

                    pid_output_roll = pid_roll.update(dict_["y_target"], dict_["y_current"], dt)
                    # pid_output_roll = pid_roll.compute(dict_["y_target"], dict_["y_current"], dt)  
                    #CMDS['roll'] = CMDS['roll'] + pid_output_roll
                    list_target_roll.append(dict_["y_target"]-dict_["y_current"])
                    list_rc_roll.append(CMDS['yaw'])
                    list_pid_roll.append([Kp_y, Ki_y, Kd_y])


                    # PITCH
#                    pid_output_pitch = pid_pitch.update(dict_["z_target"], dict_["z_current"], dt)
                    pid_output_pitch = pid_pitch.compute(dict_["z_target"], dict_["z_current"], dt)
                    #CMDS['pitch'] = CMDS['pitch'] + pid_output_pitch
                    #CMDS['pitch'] = 1700
                    list_target_pitch.append(dict_["z_target"]-dict_["z_current"])
                    list_rc_pitch.append(CMDS['pitch'])
                    list_pid_pitch.append([Kp_z, Ki_z, Kd_z])

                    # time
                    l_time = time.time()-start_time
                    list_time.append(l_time)

                    cursor_msg = f'Throttle: {CMDS["throttle"]}, PID: {pid_throttle.kp}, {pid_throttle.kd}, {pid_throttle.ki}, OUT: {pid_output_throttle}'
                    cursor_msg1 = f'Yaw: {CMDS["yaw"]}, PID: {pid_yaw.kp}, {pid_yaw.kd}, {pid_yaw.ki}, OUT: {pid_output_yaw}'

                    # Управление дроном пример #2
                    # roll = int(np.clip(output_x, -500, 500))
                    # pitch = int(np.clip(-output_y, -500, 500))  # Инвертируем Y для правильного направления
                    # throttle = int(np.clip(1500 + output_z, 1000, 2000))  # Базовое значение 1500 для висения
                    # board.send_RAW_RC([1500 + roll, 1500 + pitch, throttle, 1500, 1000, 1000, 1000, 1000])


                screen.addstr(8, 50, "Start track: {}".format(str(dict_["init_tracker"])), curses.A_BOLD)
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
                            CMDS['aux3'] = 1500
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
                            if dict_["controller_init_tracker"] == False:
                                height = filtered_distance + 40.0  
                                CMDS['throttle'] = board.RC['channels'][3]
                            # зафиксировать обьект
                            dict_["controller_init_tracker"] = True
                            CMDS['aux2'] = 1500

                        else:
                            
                            # открепить обьект
                            dict_["controller_init_tracker"] = False
                            
#                        if autopilot == False:    
#                            CMDS['throttle'] = board.RC['channels'][3]
#                            CMDS['yaw'] = board.RC['channels'][2]
#                            CMDS['pitch'] = board.RC['channels'][1]
#                            CMDS['roll'] = board.RC['channels'][0]  
#                            CMDS['aux2'] = 1500
                        screen.addstr(20, 0, "RC Channels Values: {}".format(board.RC['channels']))
                        screen.addstr(21, 0, f"RC Channels Client: {[CMDS[ki] for ki in CMDS_ORDER]}")
                        screen.clrtoeol()

                    elif next_msg == 'MSP_FEATURE_CONFIG':
                        screen.addstr(22, 0, "C: {}".format(board.FEATURE_CONFIG['features'][3])) 
                        screen.addstr(23, 0, "C: {}".format(board.FEATURE_CONFIG['features'][14])) 
                        screen.addstr(24, 0, f"Высота датчика: {filtered_distance}, желаемая высота: {height}") 
                        screen.clrtoeol()
                    screen.addstr(17, 50, "GUI cycleTime: {0:2.2f}ms (average {1:2.2f}Hz)".format((last_cycleTime)*1000,
                                  (sum(average_cycle)/len(average_cycle))))
                    screen.clrtoeol()
                                                                            
                    screen.addstr(3, 0, cursor_msg)
                    screen.addstr(4, 0, cursor_msg1)
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
        # pid_pitch.stop()
        # pid_yaw.stop()
        # pid_throttle.stop()
        # pid_roll.stop()
        

class TargetPredictor:
    def __init__(self, history_size=10):
        self.history = deque(maxlen=history_size)
        self.last_time = time.time()

    def update(self, position):
        current_time = time.time()
        dt = current_time - self.last_time
        self.history.append((position, dt))
        self.last_time = current_time

    def predict(self, time_ahead):
        if len(self.history) < 2:
            return self.history[-1][0] if self.history else None
    
        velocities = []
        for i in range(1, len(self.history)):
            dx = self.history[i][0] - self.history[i-1][0]
            dt = self.history[i][1]
            velocities.append(dx / dt if dt > 0 else 0)

        avg_velocity = sum(velocities) / len(velocities)
        last_position = self.history[-1][0]
        return last_position + avg_velocity * time_ahead


class AdaptiveTargetPredictor:
    def __init__(self, history_size=20, min_samples=5):
        self.history = deque(maxlen=history_size)
        self.last_time = time.time()
        self.min_samples = min_samples
        self.model = 'linear'
        self.coeffs = None

    def update(self, position):
        current_time = time.time()
        self.history.append((np.array(position), current_time))
        
        if len(self.history) >= self.min_samples:
            self._fit_model()

    def _fit_model(self):
        times = np.array([t for _, t in self.history]) - self.history[0][1]
        positions = np.array([p for p, _ in self.history])

        # Убедимся, что positions - это 2D массив
        if positions.ndim == 1:
            positions = positions.reshape(-1, 1)

        if self.model == 'linear':
            self.coeffs = [np.polyfit(times, positions[:, i], 1) for i in range(positions.shape[1])]
        elif self.model == 'quadratic':
            self.coeffs = [np.polyfit(times, positions[:, i], 2) for i in range(positions.shape[1])]
        elif self.model == 'exponential':
            def exp_func(x, a, b, c):
                return a * np.exp(b * x) + c
            try:
                self.coeffs = [curve_fit(exp_func, times, positions[:, i])[0] for i in range(positions.shape[1])]
            except:
                # If exponential fitting fails, fallback to linear
                self.model = 'linear'
                self.coeffs = [np.polyfit(times, positions[:, i], 1) for i in range(positions.shape[1])]

        # Adaptive model selection based on fit error
        errors = {
            'linear': np.mean([np.mean((positions[:, i] - np.polyval(np.polyfit(times, positions[:, i], 1), times))**2) for i in range(positions.shape[1])]),
            'quadratic': np.mean([np.mean((positions[:, i] - np.polyval(np.polyfit(times, positions[:, i], 2), times))**2) for i in range(positions.shape[1])]),
        }
        if self.model == 'exponential':
            errors['exponential'] = np.mean([np.mean((positions[:, i] - (self.coeffs[i][0] * np.exp(self.coeffs[i][1] * times) + self.coeffs[i][2]))**2) for i in range(positions.shape[1])])

        self.model = min(errors, key=errors.get)

    def predict(self, time_ahead):
        if len(self.history) < self.min_samples:
            return self.history[-1][0] if self.history else None

        current_time = time.time()
        prediction_time = current_time + time_ahead - self.history[0][1]

        if self.model == 'linear':
            return np.array([np.polyval(coeff, prediction_time) for coeff in self.coeffs])
        elif self.model == 'quadratic':
            return np.array([np.polyval(coeff, prediction_time) for coeff in self.coeffs])
        elif self.model == 'exponential':
            return np.array([coeff[0] * np.exp(coeff[1] * prediction_time) + coeff[2] for coeff in self.coeffs])

    def get_confidence(self):
        if len(self.history) < self.min_samples:
            return 0

        times = np.array([t for _, t in self.history]) - self.history[0][1]
        positions = np.array([p for p, _ in self.history])
        
        if positions.ndim == 1:
            positions = positions.reshape(-1, 1)

        if self.model == 'linear':
            predicted = np.array([np.polyval(coeff, times) for coeff in self.coeffs]).T
        elif self.model == 'quadratic':
            predicted = np.array([np.polyval(coeff, times) for coeff in self.coeffs]).T
        elif self.model == 'exponential':
            predicted = np.array([coeff[0] * np.exp(coeff[1] * times) + coeff[2] for coeff in self.coeffs]).T

        mse = np.mean((positions - predicted)**2)
        max_error = np.max(np.abs(positions - predicted))
        
        confidence = 1 / (1 + mse)  # Normalize confidence between 0 and 1
        return confidence

        
def image_task(dict_):
    #sys.stdout = open('output.txt', 'w')
    # Создание сокета
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    host_name = socket.gethostname()
    host_ip = socket.gethostbyname(host_name)
#    host_ip = '10.42.0.1'
    host_ip = '192.168.1.2'
    logger.info(f'Хост IP: {host_ip}')
    #print('Хост IP:', host_ip)
    port = 9999
    socket_address = (host_ip, port)
    # Привязка сокета
    server_socket.bind(socket_address)

    # Ожидание подключения клиента
    server_socket.listen(5)
    
#    k_scale = 1.0 #  yolo+sort/csrt/kcf
    k_scale = 0.7
    cap = cv2.VideoCapture(0)
    payload_size = struct.calcsize("Q")
    data = b""
    init_tracker = False
    client_socket = False
    size_box = 50
    pressed_activate_key_track = 0
    
    # предсказание цели 
    # Версия 1
#    predictor_0 = TargetPredictor()
#    predictor_1 = TargetPredictor()
        
    # Версия 2
    predictor = AdaptiveTargetPredictor()

    #lib_start.init_yolo()
    logger.info("Ожидание подключения клиента...")
    while True:
        client_socket, addr = server_socket.accept()
        #print('Получено соединение от:', addr, client_socket)
        logger.info(f'Получено соединение от: {addr}')
        try:
           while(cap.isOpened()):
#                try:
                (status, frame) = cap.read()
                if status:
                    frame = cv2.resize(frame, (int(frame.shape[1]*k_scale), int(frame.shape[0]*k_scale)))
                    _img, obj_center, img_center = lib_start.process_img_server(frame, dict_["init_tracker"])
#                    _img, obj_center, img_center = lib_start.process_img_server_NanoTrack(frame, dict_["init_tracker"])  

                    area_OIU = [img_center[0]-size_box, img_center[1]-size_box, img_center[0]+size_box, img_center[1]+size_box]
                    area_OIU = [int(d) for d in area_OIU]
                    bbox_OIU = [area_OIU[0], area_OIU[1], area_OIU[2]-area_OIU[0], area_OIU[3]-area_OIU[1]]
                    
                    # Сжатие кадра в формат JPEG
                    _, img = cv2.imencode('.jpg', _img, encode_param)
                    
                    dict_["y_current"] = img_center[0]
                    dict_["z_current"] = img_center[1]
                    
                    # Версия 1
                    # Координаты центара обьекта
#                    predictor_0.update(obj_center[0])
#                    predicted_position_0 = predictor_0.predict(0.1)  # Предсказание на 100 мс вперед
#                    
#                    predictor_1.update(obj_center[1])
#                    predicted_position_1 = predictor_1.predict(0.1)  # Предсказание на 100 мс вперед
#                    
#                    dict_["y_target"] = predicted_position_0
#                    dict_["z_target"] = predicted_position_1

#                    # Версия 2
#                    # Обновление позиции объекта адаптивный способ
                    predictor.update([obj_center[0], obj_center[1]])
                    # Предсказание позиции через 0.5 секунды
                    future_position = predictor.predict(0.5)
                    # Получение уверенности в предсказании
                    confidence = predictor.get_confidence()
                    #print([obj_center[0], obj_center[1]], f"Predicted position: {future_position}, Confidence: {confidence}")
                    dict_["y_target"] = future_position[0]
                    dict_["z_target"] = future_position[1]
                    

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
                            lib_start.init_tracker(_img, bbox)
#                            lib_start.NanoTracker.init(_img, bbox_OIU)

                            lib_start.state = 0
                            init_tracker = True
                            
                    # включение выключение слежения с пульта        
                    if dict_["controller_init_tracker"]:
                        pressed_activate_key_track += 1
                        if pressed_activate_key_track == 1:
                            lib_start.init_tracker(_img, bbox_OIU)
#                            lib_start.NanoTracker.init(_img, bbox_OIU)
                            init_tracker = True
                    else:
                        if pressed_activate_key_track > 2:
                            pressed_activate_key_track = 0
                            init_tracker = False
                    lib_start.init_switch = init_switch
                    dict_["init_tracker"] = init_tracker
        except ConnectionResetError:
            logger.warning("Клиент отключился")
            client_socket.close()
        except Exception as e:
            logger.error(f"Error in image task: {str(e)}")    
    # Close the server socket
    server_socket.close()

    
num = Value('d', 0.0)    
if __name__ == '__main__':
    
    with Manager() as manager:
        dict_ = manager.dict()
        dict_["init_tracker"] = False
        dict_["controller_init_tracker"] = False
        dict_["filtered_distance"] = 0
        
        # VL53L0X
        sensor_proc = Process(target=sensor_process, args=(dict_,))
        sensor_proc.start()
        
        # run the thread
        thread1 = Process(target=run_curses, args=(dict_, ), daemon=True)              
        thread1.start()  
                
        thread2 = Process(target=image_task, args=(dict_,), daemon=True)
        thread2.start() 
        
        # wait for the thread to finish
        logger.info('Waiting for the threads...')
        try:
            sensor_proc.join()
            thread1.join()
            thread2.join()
        except KeyboardInterrupt:
            logger.info("Main process interrupted. Shutting down...")
        finally:
            sensor_proc.terminate()
            thread1.terminate()
            thread2.terminate()
            logger.info("All processes terminated.")        
        

  

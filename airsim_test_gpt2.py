import os
import sys
import cv2
import time
import math
import airsim
import tracker_lib
import numpy as np
from numpy.linalg import norm

from pyquaternion import Quaternion

from threading import Thread
from multiprocessing import Process, Value, Array, Manager
from deap import base, creator, tools, algorithms

"""
Этот код создает экземпляр PID-регулятора с определенными коэффициентами Kp, Ki и Kd. Затем он подключается к симулятору AirSim и запускает цикл управления. В цикле он получает текущую высоту, вычисляет ошибку относительно целевой высоты, обновляет PID-регулятор и применяет получившийся управляющий сигнал к функции moveByRC

Вперёд
    pitch = 1.0,
    throttle = 1.0,
    yaw=0.0,
    roll=0.0
    
Назад
    pitch = -1.0,
    throttle = 1.0,
    yaw=0.0,
    roll=0.0        
    
Вправо
    pitch = 0.0,
    throttle = 1.0,
    yaw=0.0,
    roll=1.0

 Влево
    pitch = 0.0,
    throttle = 1.0,
    yaw=0.0,
    roll=-1.0

 Вверх 
    pitch = 0.0,
    throttle = 2.0,
    yaw=0.0,
    roll=0.0  

 Вокруг себя когда дрон в воздухе
    pitch = 0.0,
    throttle = 0.0,
    yaw=1.0,
    roll=0.0  

 Подьём при throttle = 0.6 в airsim 
 Массив [0.0, 0.0, 0.0, 0.0] 

"""

class KalmanFilterPID:
    def __init__(self, Kp, Ki, Kd, dt):
        # PID gains
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # Sampling time
        self.dt = dt

        # Kalman filter variables
        self.x = np.zeros(2)  # State vector [position, velocity]
        self.P = np.eye(2)  # Error covariance matrix
        self.Q = np.eye(2)  # Process noise covariance matrix
        self.R = np.eye(1)  # Measurement noise covariance matrix

        # PID variables
        self.error_sum = 0.0
        self.prev_error = 0.0

    def update(self, setpoint, measurement):
        # Prediction step
        self.x[0] += self.dt * self.x[1]  # Predicted position
        self.P[0, 0] += self.dt * (self.P[1, 1] + self.P[0, 1] + self.P[1, 0] + self.Q[0, 0])  # Predicted error covariance

        # Kalman gain calculation
        K = self.P[0, 0] / (self.P[0, 0] + self.R[0, 0])

        # Update step
        self.x[0] += K * (measurement - self.x[0])  # Updated position
        self.x[1] = self.x[1] + K * (measurement - self.x[0]) / self.dt  # Updated velocity
        self.P[0, 0] -= K * self.P[0, 0]  # Updated error covariance

        # PID control calculation
        error = setpoint - self.x[0]
        self.error_sum += error * self.dt
        error_rate = (error - self.prev_error) / self.dt

        output = self.Kp * error + self.Ki * self.error_sum + self.Kd * error_rate

        # Update previous error
        self.prev_error = error

        return output

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


def track_point(target_x, target_y):
    # Подключение к airsim
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)

    # Настройка PID регулятора
    pid_controller = PIDController()

    while True:
        # Получение текущего положения дрона
        current_x, current_y = get_current_position(client)

        # Вычисление ошибки между текущим и целевым положением
        error_x = target_x - current_x
        error_y = target_y - current_y

        # Вычисление управляющего сигнала с использованием PID регулятора
        control_signal = pid_controller.calculate(error_x, error_y)

        # Применение управляющего сигнала к дрону
        client.moveByVelocity(control_signal[0], control_signal[1], control_signal[2], 1)

def get_current_position(client):
    # Получение текущего положения дрона из airsim
    position = client.getPosition()
    current_x = position.x_val
    current_y = position.y_val

    return current_x, current_y

def test_pid_up(tracker_arg, dict_):
    # Connect to the AirSim simulator
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.reset()

    # Set the target height for the PID controller (in meters)
    target_height = -10

    # Set the PID controller gains

    # 25 сек
    #Kp = 0.06
    #Ki = 0.0001 
    #Kd = 7.0


    ## 10 сек
    #Kp = 0.06
    #Ki = 0.0003 
    #Kd = 9.0

    Kp = 0.045136137394194487
    Ki = 0.00022393195314520642 
    Kd = 6.404490165264038

    # Create a PID controller object Pitch
    pid_controller = PIDController(Kp, Ki, Kd) # throttle
    
    # Take off to a safe height
    client.moveByRC(rcdata = airsim.RCData(pitch = 0.0,
                        throttle = 0.6,
                        yaw=0.0,
                        roll=0.0,
                        is_initialized = True,
                        is_valid = True)) 
    time.sleep(1)


    start_time = time.time()
    f_kalman = KalmanFilterPID(Kp, Ki, Kd, 0.1)

    # Start the main control loop
    while True:
        # Get the current height from the AirSim simulator
        current_height = client.getMultirotorState().kinematics_estimated.position.z_val
        dict_["current_height"] = current_height
    #    pid_output = f_kalman.update(target_height, current_height)
        # Calculate the PID output
        pid_output = pid_controller.update(current_height, target_height)

        # Adjust throttle based on PID output
        client.moveByRC(rcdata = airsim.RCData(pitch = 0.0,
                                               throttle = -pid_output,
                                               yaw=0.0,
                                               roll=0.0,
                                               is_initialized = True,
                                               is_valid = True))
        
        time.sleep(0.01)
#        if current_height < 0.04:
#            tracker_arg.value += 1
#            dict_["init_switch"] = True
#            print (-pid_output, current_height, "error:", pid_controller.prev_error, "time:", time.time()-start_time)


def main(tracker_arg, dict_):
    client = airsim.MultirotorClient()
    client.confirmConnection()
    #client.reset()
    # Set PID controller gains
    Kp = 0.045136137394194487
    Ki = 0.00022393195314520642 
    Kd = 6.404490165264038

    # Set target position on the screen
    target_position = (client.getMultirotorState().kinematics_estimated.position.x_val+10.0, 
                       client.getMultirotorState().kinematics_estimated.position.y_val-10.0)
    
    # Initialize PID controller for x and y axes
    pid_x = PIDController(Kp, Ki, Kd)
    pid_y = PIDController(Kp, Ki, Kd)
    
    # Create a PID controller object Pitch
    #pid_controller = PIDControllerRC(target_height, Kp, Ki, Kd) # throttle
    while True:
        if -(dict_["current_height"]) > 7.0:
            # Get quadcopter position
            quadcopter_position = client.getMultirotorState().kinematics_estimated.position

            # Get quadcopter velocity
            quadcopter_velocity = client.getMultirotorState().kinematics_estimated.linear_velocity

            # Calculate error for x and y axes
            error_x = target_position[0] - quadcopter_position.x_val
            error_y = target_position[1] - quadcopter_position.y_val

            # Calculate control signals using PID controller
            control_signal_x = pid_x.update(quadcopter_position.x_val, target_position[0])
            control_signal_y = pid_y.update(quadcopter_position.y_val, target_position[1])

            # Apply control signals to quadcopter
            #client.moveByVelocityAsync(control_signal_x, control_signal_y, quadcopter_velocity.z_val, 0.1).join()
            
            # Adjust throttle based on PID output
            client.moveByRC(rcdata = airsim.RCData(throttle = control_signal_y,
                                                   roll=control_signal_x,
                                                   is_initialized = True,
                                                   is_valid = True))

            print (control_signal_x, control_signal_y, f"error X {error_x} error Y {error_y}")
            print ("------------>", target_position, quadcopter_position.x_val, quadcopter_position.y_val)
            
            time.sleep(0.01)  # Delay for stability
        
        

if __name__ == "__main__":
    num = Value('d', 0.0)
    with Manager() as manager:
        dict_ = manager.dict()
        dict_["init_switch"] = False
        dict_["current_height"] = 0
        # run the thread
        thread = Process(target=test_pid_up, args=(num, dict_), daemon=True)
        thread.start()
     
        thread2 = Process(target=main, args=(num, dict_), daemon=True)
        thread2.start()
        # wait for the thread to finish
        print('Waiting for the thread...')
        thread.join()   
        thread2.join()


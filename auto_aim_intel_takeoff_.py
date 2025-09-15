#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Автоматическое переключение на английскую раскладку при запуске
import sys, os, platform
def set_english_layout():
    system = platform.system()
    try:
        if system == "Linux":
            # Для Linux (X11)
            os.system("setxkbmap us")
        elif system == "Darwin":
            # Для macOS
            os.system("osascript -e 'tell application \"System Events\" to key code {0}'")
        elif system == "Windows":
            # Для Windows (требуется установка pywin32)
            try:
                import win32api
                import win32con
                win32api.LoadKeyboardLayout("00000409", win32con.KLF_ACTIVATE)
            except ImportError:
                print("Для переключения раскладки на Windows установите pywin32: pip install pywin32")
    except Exception as e:
        print(f"Ошибка при переключении раскладки: {str(e)}")

# Переключаем раскладку при запуске
set_english_layout()

import uinput
import time
import math
import threading
import signal
import pygame
import numpy as np
import cv2
import json
from multiprocessing import Process, Manager, Queue, Lock
from scipy.spatial import distance
from collections import deque
import concurrent
from concurrent.futures import ThreadPoolExecutor
from scipy.optimize import curve_fit
from threading import Thread
import pygame.locals as pl

# Глобальные переменные для обработки мыши
drawing = False
start_x, start_y = -1, -1
end_x, end_y = -1, -1
current_bbox = None

os.environ["XDG_SESSION_TYPE"] = "xcb"
os.environ["QT_QPA_PLATFORM"] = "xcb"

class TrackerLib(object):
    def __init__(self):
        self.state = 0
        self.init_switch = False
        self.bbox = [0, 0, 0, 0]
        self.last_bbox = [0, 0, 0, 0]
        self.Error_track = False
        self.dst = 0
        self.obj_center = [0,0]
        self.lost_object_counter = 0
        self.recent_positions = deque(maxlen=10)
        self.max_lost_frames = 30
        self.trackers = {}
        
        # Фильтр Калмана
        self.kalman = cv2.KalmanFilter(4, 2)
        self.kalman_initialized = False        
        # Проверяем доступность DaSiamRPN перед добавлением
        try:
            _ = cv2.TrackerDaSiamRPN()  # Тест создания трекера
            self.tracker_weights = {'csrt': 0.4, 'kcf': 0.3, 'dasiamrpn': 0.3}
        except (AttributeError, cv2.error):
            print("DaSiamRPN недоступен, используются только CSRT и KCF")
            self.tracker_weights = {'csrt': 0.6, 'kcf': 0.4}

    def init_kalman(self, center_x, center_y):
        """Инициализация фильтра Калмана с заданным центром"""
        # Матрица перехода (предполагаем постоянную скорость)
        self.kalman.transitionMatrix = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float32)
        
        # Матрица измерений (измеряем только координаты)
        self.kalman.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float32)
        
        # Ковариация процесса (настраиваемый параметр)
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
        # Ковариация измерений (настраиваемый параметр)
        self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.5
        # Начальная ковариационная матрица
        self.kalman.errorCovPost = np.eye(4, dtype=np.float32) * 1
        
        # Начальное состояние [x, y, vx, vy]
        self.kalman.statePost = np.array([
            [center_x], 
            [center_y], 
            [0], 
            [0]
        ], dtype=np.float32)
        
        self.kalman_initialized = True

    def update_kalman(self, measurement):
        """Обновление фильтра Калмана с новым измерением"""
        # Этап предсказания
        prediction = self.kalman.predict()
        
        # Этап коррекции
        measured = np.array([[np.float32(measurement[0])], 
                             [np.float32(measurement[1])]])
        corrected = self.kalman.correct(measured)
        
        # Возвращаем откорректированные координаты
        return (corrected[0], corrected[1])


    def get_center(self, img, x, y, w, h):
        xcentr = int(x+(w/2))
        ycentr = int(y+(h/2))
        cv2.circle(img, (xcentr, ycentr), radius=0, color=(0, 0, 255), thickness=5)
        return (xcentr, ycentr)    

    def draw_box(self, img, bbox, color_border_box=(255, 0, 255)):
        x, y, w, h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
        cv2.rectangle(img, (x, y), ((x+w), (y+h)), color_border_box, 3, 1)
        return self.get_center(img, x, y, w, h)

    def increase_brightness(self, img, value=10):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        v = np.clip(v.astype(np.int32) + value, 0, 255).astype(np.uint8)
        final_hsv = cv2.merge((h, s, v))
        return cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
        
    def init_tracker(self, img, bbox):
        self.state = 0
        self.trackers = {}  # Сбрасываем существующие трекеры
    
        # Инициализируем фильтр Калмана
        center_x = bbox[0] + bbox[2]/2
        center_y = bbox[1] + bbox[3]/2
        self.init_kalman(center_x, center_y)    
        
        model_path = "dasiamrpn_model.onnx"
        kernel_path = "dasiamrpn_kernel_r1.onnx"
        
        # Создаем только доступные трекеры
        valid_trackers = []
        for name, weight in self.tracker_weights.items():
            try:
                if name == 'csrt':
                    tracker = cv2.TrackerCSRT_create()
                elif name == 'kcf':
                    tracker = cv2.TrackerKCF_create()
                elif name == 'dasiamrpn':
                    tracker = cv2.TrackerDaSiamRPN_create()
                
                tracker.init(img, bbox)
                self.trackers[name] = tracker
                valid_trackers.append(name)
            except Exception as e:
                print(f"Ошибка инициализации {name}: {str(e)}")
        
        # Обновляем веса для работающих трекеров
        if valid_trackers:
            total_weight = sum(self.tracker_weights[n] for n in valid_trackers)
            self.tracker_weights = {n: self.tracker_weights[n]/total_weight 
                                    for n in valid_trackers}
            self.init_switch = True
        else:
            print("Не удалось инициализировать ни один трекер")
            self.init_switch = False

    def image_process(self, img, bbox, img_center, color_border_box=(255, 0, 255)):
        self.obj_center = self.draw_box(img, bbox, color_border_box)
        distance = np.linalg.norm(np.array(self.obj_center) - np.array(img_center))
        cv2.line(img, img_center, self.obj_center, (255, 0, 0), 4)
        cv2.putText(img, f"{int(distance)}", (bbox[0], bbox[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        self.recent_positions.append(self.obj_center)      

    def aim_visual(self, img, size_box=50, corner_size=20, line_thickness=3, color=(1, 152, 117), full=False):
        height, width = img.shape[:2]
        center_y, center_x = height // 2, width // 2
        top = center_y - size_box
        bottom = center_y + size_box
        left = center_x - size_box
        right = center_x + size_box

        if full:
            cv2.rectangle(img, (left, top), (right, bottom), color, line_thickness)
            cv2.rectangle(img, (left, top), (right, bottom), (255,0,0), 1)
        else:
            corners = [ (left, top), (right, top), (right, bottom), (left, bottom) ]

            for x, y in corners:
                if x == left:
                    cv2.line(img, (x, y), (x + corner_size, y), color, line_thickness)
                else:
                    cv2.line(img, (x, y), (x - corner_size, y), color, line_thickness)

                if y == top:
                    cv2.line(img, (x, y), (x, y + corner_size), color, line_thickness)
                else:
                    cv2.line(img, (x, y), (x, y - corner_size), color, line_thickness)

        return img

    def process_img_server(self, img, init_tracker):
        img_center = self.get_center(img, 0, 0, img.shape[1], img.shape[0])
        if self.init_switch or init_tracker:
            bbox_results = {}
            with ThreadPoolExecutor() as executor:
                future_to_tracker = {executor.submit(tracker.update, img): name for name, tracker in self.trackers.items()}
                for future in concurrent.futures.as_completed(future_to_tracker):
                    tracker_name = future_to_tracker[future]
                    success, bbox = future.result()
                    
                    if success:
                        bbox_results[tracker_name] = bbox

            if bbox_results:
                weighted_bbox = np.average([bbox for bbox in bbox_results.values()], 
                                           axis=0, 
                                           weights=[self.tracker_weights[name] for name in bbox_results.keys()])
                bbox = [int(x) for x in weighted_bbox]
                self.dst = distance.euclidean(self.last_bbox, bbox)
                self.last_bbox = bbox
                
                # Обновляем фильтр Калмана
                center_x, center_y = bbox[0] + bbox[2]/2, bbox[1] + bbox[3]/2
                if self.kalman_initialized:
                    kalman_center = self.update_kalman((center_x, center_y))
                    # Обновляем центр для отрисовки
                    center_x, center_y = kalman_center[0][0], kalman_center[1][0]
                    # Обновляем координаты bbox
                    bbox[0] = int(center_x - bbox[2]/2)
                    bbox[1] = int(center_y - bbox[3]/2)                
                
                self.image_process(img, bbox, img_center)
                self.lost_object_counter = 0
            else:
                self.lost_object_counter += 1
                
                # Используем предсказание Калмана при потере объекта
                if self.kalman_initialized and self.lost_object_counter < self.max_lost_frames:
                    prediction = self.kalman.predict()
                    pred_x = prediction[0][0]
                    pred_y = prediction[1][0]
                    
                    # Отрисовываем предсказанное положение
                    cv2.circle(img, (int(pred_x), int(pred_y)), 10, (0, 255, 255), 2)
                    cv2.putText(img, "PREDICTION", (int(pred_x)-30, int(pred_y)-15), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                
                if self.lost_object_counter >= self.max_lost_frames:
                    self.reinitialize_trackers(img)

        return img, self.obj_center, img_center

    def reinitialize_trackers(self, img):
        
        if self.recent_positions:
            last_known_position = self.recent_positions[-1]
            search_area = self.expand_search_area(last_known_position, img.shape)
            roi = img[search_area[1]:search_area[3], search_area[0]:search_area[2]]
            
            TS = 0
            for o in self.last_bbox:
                if o < 0:
                    TS += 1
            #print ("----", self.last_bbox, TS)
            if TS == 0:
                result = cv2.matchTemplate(roi, img[self.last_bbox[1]:self.last_bbox[1]+self.last_bbox[3], 
                                                self.last_bbox[0]:self.last_bbox[0]+self.last_bbox[2]], 
                                           cv2.TM_CCOEFF_NORMED)
                _, _, _, max_loc = cv2.minMaxLoc(result)
                
                new_bbox = (search_area[0] + max_loc[0], search_area[1] + max_loc[1], 
                            self.last_bbox[2], self.last_bbox[3])
                
                self.init_tracker(img, new_bbox)
                self.lost_object_counter = 0

    def expand_search_area(self, center, img_shape, factor=1.5):
        x, y = center
        w, h = self.last_bbox[2:]
        x1 = max(0, int(x - w*factor/2))
        y1 = max(0, int(y - h*factor/2))
        x2 = min(img_shape[1], int(x + w*factor/2))
        y2 = min(img_shape[0], int(y + h*factor/2))
        return (x1, y1, x2, y2)

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
                self.model = 'linear'
                self.coeffs = [np.polyfit(times, positions[:, i], 1) for i in range(positions.shape[1])]

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
        confidence = 1 / (1 + mse)
        return confidence

##############################
####  PID регуляторы начало
##############################


class PIDController:
    def __init__(self, kp, ki, kd, integral_limit=100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.integral_limit = integral_limit

    def update(self, current_value, target_value, dt):
        error = target_value - current_value
        self.integral += error * dt
        
        # Ограничение интегральной составляющей
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error 
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return output


######################################################
### PID регуляторы конец
######################################################

class FPVJoystickEmulator:
    def __init__(self, screen_width, screen_height):
        print("Инициализация эмулятора джойстика FPV...")

        # Явная инициализация модуля шрифтов
        if not pygame.font.get_init():
            pygame.font.init()
        
        # Создаем шрифты
        try:
            self.font = pygame.font.SysFont('Arial', 24)
            self.large_font = pygame.font.SysFont('Arial', 36)
            self.small_font = pygame.font.SysFont('Arial', 18)
        except:
            self.font = pygame.font.Font(None, 24)
            self.large_font = pygame.font.Font(None, 36)
            self.small_font = pygame.font.Font(None, 18)
 
        self.device = uinput.Device([
            uinput.ABS_X + (0, 255, 0, 0),
            uinput.ABS_Y + (0, 255, 0, 0),
            uinput.ABS_RX + (0, 255, 0, 0),
            uinput.ABS_RY + (0, 255, 0, 0),
            uinput.ABS_THROTTLE + (0, 255, 0, 0),
            uinput.BTN_JOYSTICK,
            uinput.BTN_TRIGGER,
            uinput.BTN_THUMB,
            uinput.BTN_THUMB2,
            uinput.BTN_TOP,
            uinput.BTN_TOP2,
            uinput.BTN_PINKIE,
            uinput.BTN_BASE,
            uinput.BTN_BASE2,
        ], name="FPV Joystick Emulator")
        print("Виртуальный джойстик создан")
        
        # Начальные значения
        self.x = 128
        self.y = 128
        self.rx = 128
        self.ry = 128
        self.throttle = 64
        
        # Размеры экрана
        self.screen_width = screen_width
        self.screen_height = screen_height
        
        # Состояния кнопок
        self.button_states = {
            'BTN_JOYSTICK': False,
            'BTN_TRIGGER': False,
            'BTN_THUMB': False,
            'BTN_THUMB2': False,
            'BTN_TOP': False,
            'BTN_TOP2': False,
            'BTN_PINKIE': False,
            'BTN_BASE': False,
        }
        
#        # Поля для ввода PID
#        self.pid_inputs = {
#            # X -------->
#            #'kp_x': "0.25", 'ki_x': "0.001", 'kd_x': "0.0009", # нейронастройки
#            'kp_x': "0.12", 'ki_x': "0.001", 'kd_x': "0.0009", # нейронастройки           
#            
#            # RX ------->
#            #'kp_rx': "0.18", 'ki_rx': "0.001", 'kd_rx': "0.001", # нейронастройки
#            'kp_rx': "0.05", 'ki_rx': "0.002", 'kd_rx': "0.001", # нейронастройки            
#            
#            # Y -------->
#            # PID с ограничением мощности
#            'kp_y': "0.3", 'ki_y': "0.001", 'kd_y': "0.001" # нейронастройки
#            
#        }

        # Поля для ввода PID
        self.pid_inputs = {
            # X -------->
            #'kp_x': "0.25", 'ki_x': "0.001", 'kd_x': "0.0009", # нейронастройки
            'kp_x': "0.25", 'ki_x': "0.01", 'kd_x': "0.002", # нейронастройки           
            
            # RX ------->
            #'kp_rx': "0.18", 'ki_rx': "0.001", 'kd_rx': "0.001", # нейронастройки
            'kp_rx': "0.25", 'ki_rx': "0.01", 'kd_rx': "0.002", # нейронастройки            
            
            # Y -------->
            # PID с ограничением мощности
            'kp_y': "0.35", 'ki_y': "0.001", 'kd_y': "0.001" # нейронастройки
            
        }
        
        self.active_input = None
        self.input_boxes = {}
        self.apply_button_rect = None
        
        # Инициализация PID контроллеров
        self.pid_x = PIDController(
            float(self.pid_inputs['kp_x']), 
            float(self.pid_inputs['ki_x']), 
            float(self.pid_inputs['kd_x'])
        )

        self.pid_rx = PIDController(
            float(self.pid_inputs['kp_rx']), 
            float(self.pid_inputs['ki_rx']), 
            float(self.pid_inputs['kd_rx'])
        )        
        self.pid_y = PIDController(
            kp=float(self.pid_inputs['kp_y']), 
            ki=float(self.pid_inputs['ki_y']), 
            kd=float(self.pid_inputs['kd_y'])
        )
        #----------------------------------->
        
        # Время последнего обновления
        self.last_update = time.time()
        
        # Позиции элементов
        self.stick_center = (self.screen_width - 300, 300)
        self.stick_radius = 50
        self.r_stick_center = (self.screen_width - 100, 300)
        
        self.button_positions = {
            'BTN_JOYSTICK': (self.screen_width - 750, 50),
            'BTN_TRIGGER': (self.screen_width - 750, 100),
            'BTN_THUMB': (self.screen_width - 750, 150),
            'BTN_THUMB2': (self.screen_width - 750, 200),
            'BTN_TOP': (self.screen_width - 750, 250),
            'BTN_TOP2': (self.screen_width - 750, 300),
            'BTN_PINKIE': (self.screen_width - 750, 350),
            'BTN_BASE': (self.screen_width - 750, 400),
        }
        
        for o in self.button_positions.keys():
            self.button_positions[o] = list(self.button_positions[o])
            self.button_positions[o][1] += 50
            self.button_positions[o][0] += 10
        
        # Текущие значения для отображения
        self.display_values = {
            'error_x': 0,
            'error_y': 0,
            'pid_x': 0,
            'pid_y': 0,
            'step': 1,  # Добавлено отображение шага
            'fps': 0
        }
        
        # Флаг интерактивного режима
        self.interactive_mode = False
        
        # Для плавного изменения в интерактивном режиме
        self.interactive_speed = 1.5
        self.interactive_accel = 0.1
        self.current_speed = {
            'x': 0, 'y': 0, 'rx': 0, 'ry': 0, 'throttle': 0
        }
        
        # Для отображения активных кнопок
        self.active_buttons = set()

        # Для хранения текущих позиций (без возврата в центр)
        self.hold_values = {
            'x': 128,     # Левый стик X
            'y': 0,       # Левый стик Y
            'throttle': 64  # Газ
        }
        
        # Параметры плавности для осей без возврата
        self.hold_interpolation_factor = 0.2
        self.hold_step_size = 1.5  # Скорость изменения позиции
        
        # Улучшенные параметры для плавного управления
        self.interactive_config = {
            'max_speed': 2.0,           # Максимальная скорость изменения
            'acceleration': 0.15,        # Ускорение при нажатии
            'deceleration': 0.25,        # Замедление при отпускании (больше для быстрой остановки)
            'dead_zone': 0.05,          # Мертвая зона для остановки
            'smooth_factor': 0.8,       # Фактор сглаживания (0-1)
            'response_curve': 1.2       # Кривая отклика (1.0 = линейная, >1.0 = экспоненциальная)
        }
        
        # Целевые скорости (к чему стремимся)
        self.target_speed = {
            'x': 0, 'y': 0, 'rx': 0, 'ry': 0, 'throttle': 0
        }
        
        # Текущие скорости (фактические)
        self.current_speed = {
            'x': 0, 'y': 0, 'rx': 0, 'ry': 0, 'throttle': 0
        }
        
        # Для экспоненциального сглаживания
        self.smoothed_values = {
            'x': 128, 'y': 128, 'rx': 128, 'ry': 128, 'throttle': 128
        }

        # Параметры плавности для осей без возврата
        self.hold_interpolation_factor = 0.2
        self.hold_step_size = 0.4

        # Настройки PID
        self.pid_step = 0.01  # Шаг изменения коэффициентов PID
        self.pid_axis = 'x'   # Текущая ось для настройки PID ('x' или 'y')
        self.pid_param = 'kp' # Текущий параметр для настройки ('kp', 'ki', 'kd')
        self.pid_changed = False  # Флаг изменения PID для анимации
        self.pid_change_time = 0  # Время последнего изменения PID
        self.pid_animation_duration = 1.0  # Длительность анимации в секундах

    def update_pid_controllers(self):
        """Обновление PID контроллеров с новыми значениями"""
        self.pid_x = PIDController(
            float(self.pid_inputs['kp_x']), 
            float(self.pid_inputs['ki_x']), 
            float(self.pid_inputs['kd_x'])
        )
        self.pid_rx = PIDController(
            float(self.pid_inputs['kp_rx']), 
            float(self.pid_inputs['ki_rx']), 
            float(self.pid_inputs['kd_rx'])
        )        
        self.pid_y = PIDController(
            float(self.pid_inputs['kp_y']), 
            float(self.pid_inputs['ki_y']), 
            float(self.pid_inputs['kd_y'])
        )
        
    def update_visualization(self, screen, camera_surface):
        """Обновление графического интерфейса"""
        # Отрисовка камеры
        if camera_surface:
            screen.blit(camera_surface, (10, 10))
        
        # Отрисовка разделительной линии
        pygame.draw.line(screen, (100, 100, 100), 
                        (self.screen_width // 2, 0), 
                        (self.screen_width // 2, self.screen_height), 
                        2)
        
        # Заголовок
        title = self.large_font.render("FPV Joystick Control", True, (255, 255, 0))
        screen.blit(title, (self.screen_width // 2 + 20, 20))
        
        # Индикатор режима
        mode_text = self.font.render(
            f"Режим: {'ИНТЕРАКТИВНЫЙ' if self.interactive_mode else 'ТРЕКИНГ'}",
            True, 
            (0, 255, 0) if not self.interactive_mode else (255, 0, 0)
        )
        screen.blit(mode_text, (self.screen_width - self.screen_width/5, 60))
        
        # Левый стик (X/Y)
        x_pos = self.stick_center[0] + (self.x - 128) * 0.5
        y_pos = self.stick_center[1] - (self.y - 128) * 0.5  # Инвертировать направление
        
        pygame.draw.circle(screen, (100, 100, 100), [self.stick_center[0]-40, self.stick_center[1]+20], self.stick_radius, 2)
        pygame.draw.circle(screen, (0, 255, 0), (int(x_pos)-40, int(y_pos)+20), 10)
        
        # Подписи стиков
        stick_label = self.font.render("Left Stick (X/Y)", True, (0, 255, 0))
        screen.blit(stick_label, (self.stick_center[0] - 140, self.stick_center[1] - 80))
        
        # Правый стик (RX/RY)
        rx_pos = self.r_stick_center[0] + (self.rx - 128) * 0.5
        ry_pos = self.r_stick_center[1] + (self.ry - 128) * 0.5
        pygame.draw.circle(screen, (100, 100, 100), [self.r_stick_center[0]-40, self.r_stick_center[1]+20], self.stick_radius, 2)
        pygame.draw.circle(screen, (255, 0, 0), (int(rx_pos)-40, int(ry_pos)+20), 10)
        
        r_stick_label = self.font.render("Right Stick (RX/RY)", True, (255, 0, 0))
        screen.blit(r_stick_label, (self.r_stick_center[0] - 130, self.r_stick_center[1] - 80))
        
        # Газ
        throttle_height = self.throttle / 255 * 200
        pygame.draw.rect(screen, (200, 200, 0), 
                        (self.screen_width - 555, 400 - throttle_height, 40, throttle_height))
        
        throttle_label = self.font.render("ARM", True, (200, 200, 0))
        screen.blit(throttle_label, (self.screen_width - 560, 420))
        
        # Кнопки
        for btn, pos in self.button_positions.items():
            color = (0, 255, 0) if self.button_states[btn] else (100, 100, 100)
            pygame.draw.circle(screen, color, pos, 15)
            text = self.font.render(btn.split('_')[-1], True, (255, 255, 255))
            screen.blit(text, (pos[0] - 10, pos[1] - 10))
            
        # Информационная панель
        pygame.draw.rect(screen, (40, 40, 60), 
                        (self.screen_width // 2 + 10, self.screen_height - 200, 
                         self.screen_width // 2 - 20, 190), 0)
        
        info_title = self.font.render("Tracking Information", True, (200, 200, 255))
        screen.blit(info_title, (self.screen_width // 2 + 20, self.screen_height - 190))
        
        # Отображение информации
        y_pos = self.screen_height - 160
        for key, value in self.display_values.items():
            if key != "fps":
                text = self.font.render(f"{key}: {value:.2f}", True, (200, 200, 255))
                screen.blit(text, (self.screen_width // 2 + 30, y_pos))
                
            else:
                # Отображение FPS
                fps_text = self.font.render(f"FPS: {self.display_values['fps']:.1f}", True, (0, 255, 255))
                screen.blit(fps_text, (self.screen_width // 2 + 30, y_pos))
            y_pos += 25
        # Отображение активных кнопок в интерактивном режиме
        if self.interactive_mode and self.active_buttons:
            active_text = self.small_font.render("Активные кнопки: " + ", ".join(self.active_buttons), True, (0, 255, 255))
            screen.blit(active_text, (20, self.screen_height - 30))
        #---------------------------->
        
        # Панель настроек PID
        self._draw_pid_settings(screen)
        
    def emit_events(self):
        """Отправка событий джойстика"""
        self.device.emit(uinput.ABS_X, self.x)
        self.device.emit(uinput.ABS_Y, self.y)
        self.device.emit(uinput.ABS_RX, self.rx)
        self.device.emit(uinput.ABS_RY, self.ry)
        self.device.emit(uinput.ABS_THROTTLE, self.throttle)
        
        for btn, state in self.button_states.items():
            btn_code = getattr(uinput, btn)
            self.device.emit(btn_code, 1 if state else 0)

#    def update_position(self, error_x, error_y, fps):
#        """Обновление положения джойстика на основе ошибки трекинга"""
#        current_time = time.time()
#        dt = current_time - self.last_update
#        self.last_update = current_time
#        
#        # ПИД-регуляторы
#        pid_x = self.pid_x.update(0, error_x, dt)  
#        pid_rx = self.pid_rx.update(0, error_x, dt)      
#        pid_y = self.pid_y.update(0, error_y, dt)
#        
#        # Ограничение -100..100
#        pid_x = max(-100, min(100, pid_x))
#        pid_rx = max(-100, min(100, pid_rx))
#        pid_y = max(-100, min(100, pid_y))  
#        
#        # Преобразование в диапазон 
#        SCALE = 127 / 100.0  # 127 потому что 128 ±127 даст 1 и 255 (границы)

#        self.y = int(128 - pid_y * SCALE)  # Инвертировали ось Y
#        self.x = int(128 + pid_x * SCALE)
#        
#        # Ограничение диапазона для self.y (20% - 80% от полного диапазона)
#        MIN_Y = int(255 * 0.2)  # 20% = 51
#        MAX_Y = int(255 * 0.75)  # 80% = 204
#        self.y = max(MIN_Y, min(MAX_Y, self.y))
#        
#        # Обновление значений для отображения
#        self.display_values['error_x'] = error_x
#        self.display_values['error_y'] = error_y
#        self.display_values['pid_x'] = pid_x
#        self.display_values['pid_y'] = pid_y
#        self.display_values['fps'] = fps
#        
#        # Отправка команд
#        self.emit_events()

#    def update_position(self, error_x, error_y, fps):
#        """Обновление положения с адаптивной фильтрацией"""
#        current_time = time.time()
#        dt = current_time - self.last_update
#        self.last_update = current_time
#        
#        # ПИД-регуляторы
#        pid_x = self.pid_x.update(0, error_x, dt) 
#         
#        pid_y = self.pid_y.update(0, error_y, dt)
#        
#        pid_rx = self.pid_rx.update(0, error_x, dt)
#        
#        # Ограничение -100..100
#        pid_x = max(-100, min(100, pid_x))
#        pid_y = max(-100, min(100, pid_y))
#        pid_rx = max(-100, min(100, pid_rx))
#        # Расчет сырого значения Y
#        raw_y = 128 - pid_y * (127 / 100.0)
#        
#        # Адаптивный фильтр (чем больше изменение, тем меньше фильтрация)
#        if not hasattr(self, 'filtered_y'):
#            self.filtered_y = raw_y
#            self.last_error_y = error_y
#        
#        # Вычисление скорости изменения ошибки
#        error_change = abs(error_y - self.last_error_y)
#        self.last_error_y = error_y
#        
#        # Адаптивный коэффициент фильтрации
#        adaptive_alpha = 0.7 - min(0.6, error_change / 50.0)  # 0.1-0.7 в зависимости от скорости изменения
#        adaptive_alpha = max(0.1, min(0.7, adaptive_alpha))
#        
#        # Применение фильтра
#        self.filtered_y = adaptive_alpha * raw_y + (1 - adaptive_alpha) * self.filtered_y
#        
#        # Ограничение диапазона
#        MIN_Y = int(255 * 0.2)
#        MAX_Y = int(255 * 0.8)
#        self.y = int(max(MIN_Y, min(MAX_Y, self.filtered_y)))
#        
#        self.x = int(128 + pid_x * (127 / 100.0))
#        
#        self.rx = int(128 + pid_rx * (127 / 100.0))
#        
#        # Обновление значений для отображения
#        self.display_values['error_x'] = error_x
#        self.display_values['error_y'] = error_y
#        self.display_values['pid_x'] = pid_x
#        self.display_values['pid_y'] = pid_y
#        self.display_values['fps'] = fps
#        
#        # Отправка команд
#        self.emit_events()

    # С фильтром для всех осехй
    def update_position(self, error_x, error_y, img_width, fps):
        """Обновление положения джойстика на основе ошибки трекинга с евро-фильтрацией всех осей"""
        
        current_time = time.time()
        dt = current_time - self.last_update
        self.last_update = current_time
        
        # ПИД-регуляторы
        pid_x = self.pid_x.update(0, error_x, dt)  
        pid_y = self.pid_y.update(0, error_y, dt)
        pid_rx = self.pid_rx.update(0, error_x, dt)
        
        # Ограничения применения управления в процентном соотношении
        if img_width != 0:
            # Вычисляем отклонение от центра в процентах от половины ширины
            deviation_percent = abs(error_x) / (img_width / 2) * 100
        else:
            deviation_percent = 0
        
#        PRC = 3 
#        # Управление RX начинается только при отклонении более 30%
#        if deviation_percent > PRC:
#            # Масштабируем ошибку для RX, начиная с 30% отклонения
#            rx_error_scale = min(1.0, (deviation_percent - PRC) / (100-PRC))  # От 0 до 1 при 30-100%
#            pid_rx = self.pid_rx.update(0, error_x * rx_error_scale, dt)
#        else:
#            # Если отклонение меньше 30%, не применяем управление RX
#            pid_rx = 0
#            # Сбрасываем интегральную составляющую PID для RX
#            self.pid_rx.integral = 0
        
        # Ограничение -100..100
        pid_x = max(-100, min(100, pid_x))
        pid_rx = max(-100, min(100, pid_rx))
        pid_y = max(-100, min(100, pid_y))  
        
        # Преобразование в диапазон 
        SCALE = 127 / 100.0

        # Расчет сырых значений
        raw_x = 128 + pid_x * SCALE
        raw_y = 128 - pid_y * SCALE  # Инвертировали ось Y
        raw_rx = 128 + pid_rx * SCALE
        
        # Инициализация евро-фильтров при первом вызове
        if not hasattr(self, 'euro_filters_initialized'):
            self.euro_filters_initialized = True
            self.euro_filters = {
                'x': {'prev_raw': raw_x, 'prev_filtered': raw_x, 'prev_time': current_time},
                'y': {'prev_raw': raw_y, 'prev_filtered': raw_y, 'prev_time': current_time},
                'rx': {'prev_raw': raw_rx, 'prev_filtered': raw_rx, 'prev_time': current_time},
            }
            self.euro_filter_min_cutoff = 1.0  # Минимальная частота среза (Гц)
            self.euro_filter_beta = 0.1        # Коэффициент скорости
        
        # Применение евро-фильтра ко всем осям
        filtered_x = self.one_euro_filter('x', raw_x, current_time)
        filtered_y = self.one_euro_filter('y', raw_y, current_time)
        filtered_rx = self.one_euro_filter('rx', raw_rx, current_time)
        
        # Ограничение диапазона для Y (20% - 80% от полного диапазона)
        MIN_Y = int(255 * 0.2)  # 20% = 51
        MAX_Y = int(255 * 0.8)  # 80% = 204
        filtered_y = max(MIN_Y, min(MAX_Y, filtered_y))
        
        # Присваиваем отфильтрованные значения
        self.x = int(filtered_x)
        self.y = int(filtered_y)
        self.rx = int(filtered_rx)
        #print (self.rx)
        
        # Обновление значений для отображения
        self.display_values['error_x'] = error_x
        self.display_values['error_y'] = error_y
        self.display_values['pid_x'] = pid_x
        self.display_values['pid_y'] = pid_y
        self.display_values['deviation_percent'] = deviation_percent  # Добавляем отображение отклонения
        self.display_values['fps'] = fps
        
        # Отправка команд
        self.emit_events()


#    def one_euro_filter(self, axis, value, timestamp):
#        """Реализация евро-фильтра для конкретной оси"""
#        # Получаем состояние фильтра для оси
#        state = self.euro_filters[axis]
#        
#        # Вычисление дельты времени
#        dt = timestamp - state['prev_time']
#        state['prev_time'] = timestamp
#        
#        # Предотвращение деления на ноль
#        if dt <= 0:
#            return state['prev_filtered']
#        
#        # Вычисление производной (скорости изменения)
#        dx = (value - state['prev_raw']) / dt
#        
#        # Адаптивная частота среза на основе скорости изменения
#        cutoff = self.euro_filter_min_cutoff + self.euro_filter_beta * abs(dx)
#        
#        # Коэффициент сглаживания на основе частоты среза
#        alpha = 1.0 / (1.0 + (1.0 / (cutoff * dt)))
#        
#        # Применение фильтра низких частот
#        filtered_value = alpha * value + (1.0 - alpha) * state['prev_filtered']
#        
#        # Обновление предыдущих значений
#        state['prev_raw'] = value
#        state['prev_filtered'] = filtered_value
#        
#        return filtered_value

    # Добавляем метод для настройки параметров фильтра
    def set_euro_filter_params(self, min_cutoff=None, beta=None):
        """Установка параметров евро-фильтра для всех осей"""
        if min_cutoff is not None:
            self.euro_filter_min_cutoff = max(0.1, min_cutoff)
        if beta is not None:
            self.euro_filter_beta = max(0.01, beta)

    # Добавляем метод для сброса фильтров
    def reset_euro_filters(self):
        """Сброс состояния всех евро-фильтров"""
        if hasattr(self, 'euro_filters_initialized'):
            del self.euro_filters_initialized
            del self.euro_filters

    # Добавляем метод для установки отдельных параметров для каждой оси
    def set_axis_filter_params(self, axis, min_cutoff=None, beta=None):
        """Установка отдельных параметров фильтра для конкретной оси"""
        if not hasattr(self, 'euro_filters_initialized'):
            return
        
        if min_cutoff is not None:
            # Сохраняем индивидуальные параметры для оси
            if not hasattr(self, 'axis_filter_params'):
                self.axis_filter_params = {}
            if axis not in self.axis_filter_params:
                self.axis_filter_params[axis] = {}
            self.axis_filter_params[axis]['min_cutoff'] = max(0.1, min_cutoff)
        
        if beta is not None:
            if not hasattr(self, 'axis_filter_params'):
                self.axis_filter_params = {}
            if axis not in self.axis_filter_params:
                self.axis_filter_params[axis] = {}
            self.axis_filter_params[axis]['beta'] = max(0.01, beta)

    # Модифицируем one_euro_filter для поддержки индивидуальных параметров осей
    def one_euro_filter(self, axis, value, timestamp):
        """Реализация евро-фильтра с поддержкой индивидуальных параметров для осей"""
        # Получаем состояние фильтра для оси
        state = self.euro_filters[axis]
        
        # Используем индивидуальные параметры для оси, если они заданы
        min_cutoff = self.euro_filter_min_cutoff
        beta = self.euro_filter_beta
        
        if hasattr(self, 'axis_filter_params') and axis in self.axis_filter_params:
            axis_params = self.axis_filter_params[axis]
            if 'min_cutoff' in axis_params:
                min_cutoff = axis_params['min_cutoff']
            if 'beta' in axis_params:
                beta = axis_params['beta']
        
        # Вычисление дельты времени
        dt = timestamp - state['prev_time']
        state['prev_time'] = timestamp
        
        # Предотвращение деления на ноль
        if dt <= 0:
            return state['prev_filtered']
        
        # Вычисление производной (скорости изменения)
        dx = (value - state['prev_raw']) / dt
        
        # Адаптивная частота среза на основе скорости изменения
        cutoff = min_cutoff + beta * abs(dx)
        
        # Коэффициент сглаживания на основе частоты среза
        alpha = 1.0 / (1.0 + (1.0 / (cutoff * dt)))
        
        # Применение фильтра низких частот
        filtered_value = alpha * value + (1.0 - alpha) * state['prev_filtered']
        
        # Обновление предыдущих значений
        state['prev_raw'] = value
        state['prev_filtered'] = filtered_value
        
        return filtered_value
    # конец


    def draw_animated_selection(self, surface, x, y, width, height, animation_counter):
        """Рисует анимированную рамку выделения с эффектом пульсации"""
        # Размер угловых элементов
        corner_size = 20
        pulse_factor = 0.5 + abs(math.sin(animation_counter * 2)) * 0.5
        
        # Цвет (пульсирующий)
        pulse = abs(math.sin(animation_counter)) * 155 + 100
        color = (0, int(pulse), 0)
        
        # Толщина линии (пульсирует)
        line_thickness = max(2, int(3 * pulse_factor))
        
        # Угловые элементы
        corners = [
            (x, y),  # Левый верхний
            (x + width, y),  # Правый верхний
            (x + width, y + height),  # Правый нижний
            (x, y + height)  # Левый нижний
        ]
        
        # Рисуем углы
        for i, (cx, cy) in enumerate(corners):
            if i == 0:  # Левый верхний
                pygame.draw.line(surface, color, (cx, cy), (cx + corner_size, cy), line_thickness)
                pygame.draw.line(surface, color, (cx, cy), (cx, cy + corner_size), line_thickness)
            elif i == 1:  # Правый верхний
                pygame.draw.line(surface, color, (cx, cy), (cx - corner_size, cy), line_thickness)
                pygame.draw.line(surface, color, (cx, cy), (cx, cy + corner_size), line_thickness)
            elif i == 2:  # Правый нижний
                pygame.draw.line(surface, color, (cx, cy), (cx - corner_size, cy), line_thickness)
                pygame.draw.line(surface, color, (cx, cy), (cx, cy - corner_size), line_thickness)
            elif i == 3:  # Левый нижний
                pygame.draw.line(surface, color, (cx, cy), (cx + corner_size, cy), line_thickness)
                pygame.draw.line(surface, color, (cx, cy), (cx, cy - corner_size), line_thickness)
        
        # Анимированные точки по углам
        dot_size = 4 * pulse_factor
        for cx, cy in corners:
            pygame.draw.circle(surface, color, (int(cx), int(cy)), dot_size)
        
        # Текст "Выделение" в центре
        if width > 100 and height > 30:
            text = self.font.render("Выделение", True, color)
            text_rect = text.get_rect(center=(x + width/2, y + height/2))
            surface.blit(text, text_rect)

    def apply_response_curve(self, value, curve_factor):
        """Применение кривой отклика для более естественного управления"""
        sign = 1 if value >= 0 else -1
        normalized = abs(value)
        curved = pow(normalized, curve_factor)
        return sign * curved

    def smooth_interpolation(self, current, target, factor):
        """Экспоненциальное сглаживание для плавных переходов"""
        return current + (target - current) * factor

    def interactive_mode_function(self):
        """Улучшенное интерактивное управление с плавностью"""
        keys = pygame.key.get_pressed()
        self.active_buttons = set()
        config = self.interactive_config
        
        dt = time.time() - getattr(self, '_last_interactive_update', time.time())
        self._last_interactive_update = time.time()
        dt = min(dt, 0.05)  # Ограничиваем dt для стабильности
        
        # === ОБНОВЛЕНИЕ ЦЕЛЕВЫХ СКОРОСТЕЙ ===
        # Левый стик Y (W/S - тангаж)
        # Корректная передача данных
        if keys[pygame.K_w]:
            self.hold_values['y'] = max(0, self.hold_values['y'] - self.hold_step_size)
            self.active_buttons.add("W")
        elif keys[pygame.K_s]:
            self.hold_values['y'] = min(255, self.hold_values['y'] + self.hold_step_size)
            self.active_buttons.add("S")
        
        # Левый стик X (A/D)
        if keys[pygame.K_a]:
            self.target_speed['x'] = -config['max_speed']
            self.active_buttons.add("A")
        elif keys[pygame.K_d]:
            self.target_speed['x'] = config['max_speed']
            self.active_buttons.add("D")
        else:
            self.target_speed['x'] = 0
        
        # Правый стик Y (I/K)
        if keys[pygame.K_i]:
            self.target_speed['ry'] = -config['max_speed']
            self.active_buttons.add("I")
        elif keys[pygame.K_k]:
            self.target_speed['ry'] = config['max_speed']
            self.active_buttons.add("K")
        else:
            self.target_speed['ry'] = 0
        
        # Правый стик X (J/L)
        if keys[pygame.K_j]:
            self.target_speed['rx'] = -config['max_speed']
            self.active_buttons.add("J")
        elif keys[pygame.K_l]:
            self.target_speed['rx'] = config['max_speed']
            self.active_buttons.add("L")
        else:
            self.target_speed['rx'] = 0
        
        # Газ (Q/E)
        if keys[pygame.K_q]:
            self.hold_values['throttle'] = max(0, self.hold_values['throttle'] - self.hold_step_size)
            self.active_buttons.add("Q")
        elif keys[pygame.K_e]:
            self.hold_values['throttle'] = min(255, self.hold_values['throttle'] + self.hold_step_size)
            self.active_buttons.add("E")

        # Управление PID с клавиатуры
        self._handle_pid_keyboard(keys)
        
        # === ПЛАВНОЕ ИЗМЕНЕНИЕ ТЕКУЩИХ СКОРОСТЕЙ ===
        
        for axis in self.current_speed.keys():
            target = self.target_speed[axis]
            current = self.current_speed[axis]
            
            # Определяем ускорение или замедление
            if abs(target) > abs(current) or (target * current < 0):
                # Ускоряемся или меняем направление
                accel = config['acceleration']
            else:
                # Замедляемся
                accel = config['deceleration']
            
            # Вычисляем изменение скорости
            if abs(target - current) < config['dead_zone']:
                # В мертвой зоне - останавливаемся
                self.current_speed[axis] = target
            else:
                # Плавно движемся к целевой скорости
                direction = 1 if target > current else -1
                speed_change = accel * direction * dt * 60  # Нормализуем по FPS
                new_speed = current + speed_change
                
                # Не превышаем целевую скорость
                if direction > 0:
                    self.current_speed[axis] = min(new_speed, target)
                else:
                    self.current_speed[axis] = max(new_speed, target)
            
            # Применяем кривую отклика
            curved_speed = self.apply_response_curve(
                self.current_speed[axis], 
                config['response_curve']
            )
            
            # Обновляем значения джойстика с экспоненциальным сглаживанием
            # Плавная интерполяция к целевым позициям
            if axis in ['y', 'throttle']:
                self.smoothed_values[axis] = self.smooth_interpolation(
                    self.smoothed_values[axis],
                    self.hold_values[axis],
                    self.hold_interpolation_factor
                )
                setattr(self, axis, int(self.smoothed_values[axis]))            
            
            if axis == 'x':
                target_value = 128 + curved_speed * 50
                self.smoothed_values['x'] = self.smooth_interpolation(
                    self.smoothed_values['x'], target_value, config['smooth_factor']
                )
                self.x = max(0, min(255, int(self.smoothed_values['x'])))
                
            elif axis == 'rx':
                target_value = 128 + curved_speed * 50
                self.smoothed_values['rx'] = self.smooth_interpolation(
                    self.smoothed_values['rx'], target_value, config['smooth_factor']
                )
                self.rx = max(0, min(255, int(self.smoothed_values['rx'])))
                
            elif axis == 'ry':
                target_value = 128 + curved_speed * 50
                self.smoothed_values['ry'] = self.smooth_interpolation(
                    self.smoothed_values['ry'], target_value, config['smooth_factor']
                )
                self.ry = max(0, min(255, int(self.smoothed_values['ry'])))
                
        
        # Сброс в центр (с плавностью)
        if keys[pygame.K_r]:
            # Сброс осей с удержанием позиции
            for axis in ['x', 'y', 'throttle']:
                if axis == 'throttle':
                    self.hold_values[axis] = 64
                    self.smoothed_values[axis] = 64
                    setattr(self, axis, 64)
                else:
                    self.hold_values[axis] = 128
                    self.smoothed_values[axis] = 128
                    setattr(self, axis, 128)

            # Сброс осей с возвратом в центр
            for axis in ['rx', 'ry']:
                self.target_speed[axis] = 0
                self.current_speed[axis] = 0
                self.smoothed_values[axis] = 128
                setattr(self, axis, 128)
            
            self.active_buttons.add("R")

        
        
        # Управление кнопками (без изменений)
        self.button_states['BTN_JOYSTICK'] = keys[pygame.K_1]
        if keys[pygame.K_1]: self.active_buttons.add("1")
        
        self.button_states['BTN_TRIGGER'] = keys[pygame.K_2]
        if keys[pygame.K_2]: self.active_buttons.add("2")
        
        self.button_states['BTN_THUMB'] = keys[pygame.K_3]
        if keys[pygame.K_3]: self.active_buttons.add("3")
        
        self.button_states['BTN_THUMB2'] = keys[pygame.K_4]
        if keys[pygame.K_4]: self.active_buttons.add("4")
        
        self.button_states['BTN_TOP'] = keys[pygame.K_5]
        if keys[pygame.K_5]: self.active_buttons.add("5")
        
        self.button_states['BTN_TOP2'] = keys[pygame.K_6]
        if keys[pygame.K_6]: self.active_buttons.add("6")
        
        self.button_states['BTN_PINKIE'] = keys[pygame.K_7]
        if keys[pygame.K_7]: self.active_buttons.add("7")
        
        self.button_states['BTN_BASE'] = keys[pygame.K_8]
        if keys[pygame.K_8]: self.active_buttons.add("8")
        
        # Отправка команд
        self.emit_events()
        
    def update_display_values_interactive(self):
        """Обновление отображаемых значений для интерактивного режима"""
        self.display_values.update({
            'target_x': self.target_speed['x'],
            'target_y': self.target_speed['y'],
            'speed_x': self.current_speed['x'],
            'speed_y': self.current_speed['y'],
            'smooth_x': (self.smoothed_values['x'] - 128) / 50,
            'smooth_y': (self.smoothed_values['y'] - 128) / 50,
        })        

    def _draw_pid_settings(self, screen):
        """Отрисовка панели настроек PID"""
        # Фон панели
        panel_rect = pygame.Rect(self.screen_width // 3 + 10, self.screen_height - 240, 
                                self.screen_width // 3, 240)
        panel_rect.x = self.screen_width-panel_rect.width
                        
        pygame.draw.rect(screen, (40, 50, 60), panel_rect, 0)
        pygame.draw.rect(screen, (80, 80, 100), panel_rect, 2)
        
        title = self.font.render("PID Settings", True, (200, 200, 255))
        screen.blit(title, (panel_rect.x + 300, panel_rect.y + 10))
        
        # Поля ввода
        y_pos = panel_rect.y + 20
        labels = [
            ("Kp X:", 'kp_x'), ("Ki X:", 'ki_x'), ("Kd X:", 'kd_x'),
            ("Kp Y:", 'kp_y'), ("Ki Y:", 'ki_y'), ("Kd Y:", 'kd_y')
        ]
        
        self.input_boxes = {}
        for i, (label, key) in enumerate(labels):
            # Метка
            text = self.font.render(label, True, (200, 200, 255))
            screen.blit(text, (panel_rect.x + 20, y_pos))
            
            # Поле ввода
            input_rect = pygame.Rect(panel_rect.x + 120, y_pos - 5, 100, 30)
            pygame.draw.rect(screen, (60, 60, 80), input_rect, 0)
            pygame.draw.rect(screen, (100, 100, 140) if self.active_input == key else (80, 80, 120), input_rect, 2)
            
            # Текст в поле
            input_text = self.font.render(self.pid_inputs[key], True, (255, 255, 255))
            screen.blit(input_text, (input_rect.x + 5, input_rect.y + 5))
            
            self.input_boxes[key] = input_rect
            y_pos += 40 if i == 2 else 30  # Больший отступ после X параметров
        
        # Кнопка "Применить"
        apply_rect = pygame.Rect(panel_rect.x + panel_rect.width - 220, panel_rect.y + panel_rect.height - 40, 100, 30)
        apply_rect.width = 200
        pygame.draw.rect(screen, (0, 150, 0), apply_rect, 0)
        pygame.draw.rect(screen, (0, 200, 0), apply_rect, 2)
        apply_text = self.font.render("Применить", True, (255, 255, 255))
        screen.blit(apply_text, (apply_rect.x + (apply_rect.width-apply_text.get_width())//2, apply_rect.y))
        self.apply_button_rect = apply_rect
        
    def _handle_pid_keyboard(self, keys):
        """Обработка клавиш для управления PID"""
        # Выбор оси
        if keys[pygame.K_x]:
            self.pid_axis = 'x'
            self.active_buttons.add("X")
        elif keys[pygame.K_y]:
            self.pid_axis = 'y'
            self.active_buttons.add("Y")
        
        # Выбор параметра
        if keys[pygame.K_p]:
            self.pid_param = 'kp'
            self.active_buttons.add("P")
        elif keys[pygame.K_i]:
            self.pid_param = 'ki'
            self.active_buttons.add("I")
        elif keys[pygame.K_d]:
            self.pid_param = 'kd'
            self.active_buttons.add("D")
        
        # Изменение значения
        changed = False
        if keys[pygame.K_PLUS] or keys[pygame.K_EQUALS]:
            key = f'{self.pid_param}_{self.pid_axis}'
            current_val = float(self.pid_inputs[key])
            self.pid_inputs[key] = str(round(current_val + self.pid_step, 3))
            changed = True
            self.active_buttons.add("+")
        elif keys[pygame.K_MINUS]:
            key = f'{self.pid_param}_{self.pid_axis}'
            current_val = float(self.pid_inputs[key])
            new_val = current_val - self.pid_step
            if new_val < 0: new_val = 0
            self.pid_inputs[key] = str(round(new_val, 3))
            changed = True
            self.active_buttons.add("-")
        
        # Применение изменений
        if changed:
            self.update_pid_controllers()
            self.pid_changed = True
            self.pid_change_time = time.time()
        
    def handle_pid_input(self, event):
        """Обработка ввода для полей PID"""
        if event.type == pygame.MOUSEBUTTONDOWN:
            # Проверка клика по полям ввода
            for key, rect in self.input_boxes.items():
                if rect.collidepoint(event.pos):
                    self.active_input = key
                    return True
                    
            # Проверка клика по кнопке "Применить"
            if self.apply_button_rect and self.apply_button_rect.collidepoint(event.pos):
                self.update_pid_controllers()
                return True
                
            # Сброс активного поля при клике вне области
            self.active_input = None
            return False
            
        elif event.type == pygame.KEYDOWN and self.active_input:
            if event.key == pygame.K_RETURN:
                self.update_pid_controllers()
                self.active_input = None
                return True
            elif event.key == pygame.K_BACKSPACE:
                self.pid_inputs[self.active_input] = self.pid_inputs[self.active_input][:-1]
                return True
            elif event.key == pygame.K_ESCAPE:
                self.active_input = None
                return True
            elif event.unicode.isdigit() or event.unicode == '.':
                self.pid_inputs[self.active_input] += event.unicode
                return True
                
        return False

def draw_pid_visualization(img, joystick, img_center, obj_center):
    """Визуализация работы PID на изображении с камеры"""
    # Рисуем вектор ошибки
    cv2.arrowedLine(img, img_center, obj_center, (0, 255, 255), 2)
    
    # Вычисляем и рисуем компоненты PID
    error_x = obj_center[0] - img_center[0]
    error_y = obj_center[1] - img_center[1]
    
    # P-компонент (пропорциональный)
    p_x = joystick.pid_x.kp * error_x
    p_y = joystick.pid_y.kp * error_y
    
    # I-компонент (интегральный)
    i_x = joystick.pid_x.ki * joystick.pid_x.integral
    i_y = joystick.pid_y.ki * joystick.pid_y.integral
    
    # D-компонент (дифференциальный)
    d_x = joystick.pid_x.kd * ((error_x - joystick.pid_x.prev_error) / 0.1 if 0.1 > 0 else 0)
    d_y = joystick.pid_y.kd * ((error_y - joystick.pid_y.prev_error) / 0.1 if 0.1 > 0 else 0)
    
    # Рисуем компоненты PID
    pid_origin = (img_center[0], img_center[1] + 50)
    
    # P-компонент (красный)
    cv2.arrowedLine(img, pid_origin, 
                   (int(pid_origin[0] + p_x), int(pid_origin[1] + p_y)), 
                   (0, 0, 255), 2)
    cv2.putText(img, "P", (int(pid_origin[0] + p_x), int(pid_origin[1] + p_y)), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    
    # I-компонент (зеленый)
    cv2.arrowedLine(img, pid_origin, 
                   (int(pid_origin[0] + i_x), int(pid_origin[1] + i_y)), 
                   (0, 255, 0), 2)
    cv2.putText(img, "I", (int(pid_origin[0] + i_x), int(pid_origin[1] + i_y)), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    
    # D-компонент (синий)
    cv2.arrowedLine(img, pid_origin, 
                   (int(pid_origin[0] + d_x), int(pid_origin[1] + d_y)), 
                   (255, 0, 0), 2)
    cv2.putText(img, "D", (int(pid_origin[0] + d_x), int(pid_origin[1] + d_y)), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
    
    # Суммарный PID (белый)
    total_x = p_x + i_x + d_x
    total_y = p_y + i_y + d_y
    cv2.arrowedLine(img, pid_origin, 
                   (int(pid_origin[0] + total_x), int(pid_origin[1] + total_y)), 
                   (255, 255, 255), 2)
    cv2.putText(img, "PID", (int(pid_origin[0] + total_x), int(pid_origin[1] + total_y)), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Отображаем значения PID
    cv2.putText(img, f"P: ({p_x:.1f}, {p_y:.1f})", (10, img.shape[0] - 60), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    cv2.putText(img, f"I: ({i_x:.1f}, {i_y:.1f})", (10, img.shape[0] - 40), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    cv2.putText(img, f"D: ({d_x:.1f}, {d_y:.1f})", (10, img.shape[0] - 20), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
    
    return img

def draw_joystick_position(img, joystick, img_center):
    """Визуализация положения левого стика на изображении"""
    # Масштабируем положение стика для отображения на изображении
    scale_x = img.shape[1] / 255.0
    scale_y = img.shape[0] / 255.0
    
    # Вычисляем положение стика на изображении (инвертируем ось Y)
    stick_x = int(joystick.x * scale_x)
    stick_rx = int(joystick.rx * scale_x)
    
    stick_y = int(img.shape[0] - joystick.y * scale_y)  # Инвертирование оси Y
    
    # Рисуем перекрестие в позиции стика
    cv2.drawMarker(img, (stick_x, stick_y), (0, 255, 0), 
                  cv2.MARKER_CROSS, 20, 2)
    
    # Рисуем круг вокруг позиции стика
    cv2.circle(img, (stick_x, stick_y), 30, (0, 255, 0), 2)
    
    # Соединяем центр изображения с позицией стика
    cv2.line(img, img_center, (stick_x, stick_y), (0, 255, 0), 2)
    
    # Соединяем центр изображения с позицией стика
    cv2.line(img, img_center, (stick_rx, stick_y), (0, 100, 0), 2)

    
    # Подписываем позицию
    cv2.putText(img, f"Stick: ({joystick.x}, {joystick.y})", 
               (stick_x + 40, stick_y), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    
    return img
        
def image_task(frame_queue, bbox_queue, dict_):
    k_scale = 1.2 #0.6
    cap = cv2.VideoCapture(0)
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
    size_box = 50
    pressed_activate_key_track = 0
    predictor = AdaptiveTargetPredictor()
    lib_start = TrackerLib()
    current_tracker_bbox = None
    
    # Счетчики FPS для процесса обработки
    proc_frame_count = 0
    proc_start_time = time.time()
    proc_fps = 0

    while True:
        (status, frame) = cap.read()
        if status:
            frame = cv2.resize(frame, (int(frame.shape[1]*k_scale), int(frame.shape[0]*k_scale)))
            
            # Расчет FPS процесса обработки
            proc_frame_count += 1
            if proc_frame_count >= 10:
                end_time = time.time()
                proc_fps = proc_frame_count / (end_time - proc_start_time)
                proc_start_time = time.time()
                proc_frame_count = 0
            
            # Получаем новый bbox из очереди
            if not bbox_queue.empty():
                current_tracker_bbox = bbox_queue.get()
                lib_start.init_tracker(frame, current_tracker_bbox)
                dict_["init_tracker"] = True
            
            # Обработка изображения
            _img, obj_center, img_center = lib_start.process_img_server(frame, dict_["init_tracker"])
            
            area_OIU = [img_center[0]-size_box, img_center[1]-size_box, 
                        img_center[0]+size_box, img_center[1]+size_box]
            area_OIU = [int(d) for d in area_OIU]
            bbox_OIU = [area_OIU[0], area_OIU[1], 
                        area_OIU[2]-area_OIU[0], area_OIU[3]-area_OIU[1]]
            
            # Обновление и предсказание позиции
            predictor.update([obj_center[0], obj_center[1]])
            future_position = predictor.predict(0.5)
            
            if future_position is not None:
                dict_["y_target"] = future_position[0]
                dict_["z_target"] = future_position[1]
            
            dict_["y_current"] = img_center[0]
            dict_["z_current"] = img_center[1]
            
            # Вычисляем ошибку позиционирования
            dict_["error_x"] = obj_center[0] - img_center[0]
            dict_["error_y"] = obj_center[1] - img_center[1]
            
            # Добавляем FPS процесса обработки на изображение
            cv2.putText(_img, f"PROC FPS: {proc_fps:.1f}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            # Отправка кадра в главный процесс
            _, img_encoded = cv2.imencode('.jpg', _img, encode_param)
            if not frame_queue.full():
                frame_queue.put((img_encoded.tobytes(), _img.shape[1], _img.shape[0], proc_fps))

def main():
    # Глобальные переменные для обработки мыши
    drawing = False
    start_x, start_y = -1, -1
    end_x, end_y = -1, -1
    current_bbox = None
    
    with Manager() as manager:
        dict_ = manager.dict()
        dict_["init_tracker"] = False
        dict_["controller_init_tracker"] = False
        dict_["filtered_distance"] = 0
        dict_["width"] = 0
        dict_["error_x"] = 0
        dict_["error_y"] = 0
        
        # Очереди для передачи данных между процессами
        frame_queue = Queue(maxsize=3)
        bbox_queue = Queue()
        
        # Запуск процесса обработки изображений
        image_proc = Process(target=image_task, args=(frame_queue, bbox_queue, dict_), daemon=True)
        image_proc.start()
        
        # Инициализация Pygame
        screen_width = 1600
        screen_height = 720
        screen = pygame.display.set_mode((screen_width, screen_height))
        pygame.display.set_caption("FPV Drone Control System")
        
        # Инициализация эмулятора джойстика
        joystick = FPVJoystickEmulator(screen_width, screen_height)
        # Для более плавного управления тангажом (ось Y)
        joystick.set_axis_filter_params('y', min_cutoff=0.4, beta=0.02)

        # Для более отзывчивого управления рысканьем (ось X)
        joystick.set_axis_filter_params('x', min_cutoff=1.5, beta=0.2)

        # Для плавного управления креном (ось RX)
        joystick.set_axis_filter_params('rx', min_cutoff=0.07, beta=0.008)

        
        # Счетчики FPS для главного процесса (отображения)
        disp_frame_count = 0
        disp_start_time = time.time()
        disp_fps = 0

        # Переменные для анимации рамки
        selection_animation_counter = 0
        selection_animation_speed = 0.2
        selection_color = (0, 255, 0)
        last_selection_time = 0
        selection_animation_active = False
        
        # Основной цикл управления
        running = True
        camera_surface = None
        
        while running:
            # Обработка событий Pygame
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1:  # Левая кнопка мыши
                        drawing = True
                        start_x, start_y = event.pos
                        end_x, end_y = event.pos
                elif event.type == pygame.MOUSEMOTION:
                    if drawing:
                        end_x, end_y = event.pos
                elif event.type == pygame.MOUSEBUTTONUP:
                    if event.button == 1 and drawing:
                        drawing = False
                        end_x, end_y = event.pos
                        
                        # Вычисляем координаты прямоугольника
                        x1 = min(start_x, end_x)
                        y1 = min(start_y, end_y)
                        x2 = max(start_x, end_x)
                        y2 = max(start_y, end_y)
                        
                        # Учитываем только область изображения (левая половина)
                        if x1 < screen_width // 2 and x2 < screen_width // 2:
                            current_bbox = (x1, y1, x2 - x1, y2 - y1)
                elif event.type == pygame.KEYDOWN:
                    # Обработка независимо от раскладки
                    if event.key == pl.K_a:  # Активация трекера
                        dict_["controller_init_tracker"] = True
                    elif event.key == pl.K_d:  # Деактивация трекера
                        dict_["controller_init_tracker"] = False
                    elif event.key == pl.K_TAB:  # Переключение режима управления
                        joystick.interactive_mode = not joystick.interactive_mode
                        # Сброс скоростей при переключении режима
                        for key in joystick.current_speed:
                            joystick.current_speed[key] = 0
                    elif event.key == pl.K_ESCAPE:  # Выход
                        running = False
                
                # Обработка ввода PID
                if joystick.handle_pid_input(event):
                    pass  # Уже обработано
            
            # Получение кадра из очереди
            if not frame_queue.empty():
                frame_data, width, height, proc_fps = frame_queue.get()
                dict_["width"] = width
                # Декодирование изображения
                nparr = np.frombuffer(frame_data, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                
                # Добавляем визуализацию PID и положения стика
                if not joystick.interactive_mode:
                    img_center = (frame.shape[1] // 2, frame.shape[0] // 2)
                    obj_center = (img_center[0] + dict_["error_x"], img_center[1] + dict_["error_y"])
                    
                    # Визуализация работы PID
                    #frame = draw_pid_visualization(frame, joystick, img_center, obj_center)
                    
                    # Визуализация положения левого стика
                    frame = draw_joystick_position(frame, joystick, img_center)
                
                # Создание поверхности Pygame из изображения
                camera_surface = pygame.image.frombuffer(frame.tobytes(), (width, height), "BGR")
            
            # Отправляем новый bbox в дочерний процесс
            if current_bbox is not None:
                if max(current_bbox)+10<frame.shape[0]:
                    bbox_queue.put(current_bbox)
                    current_bbox = None            
            
            # Расчет FPS отображения
            disp_frame_count += 1
            current_time = time.time()
            if current_time - disp_start_time >= 1.0:
                disp_fps = disp_frame_count / (current_time - disp_start_time)
                disp_frame_count = 0
                disp_start_time = current_time
            
            # Очистка экрана
            screen.fill((30, 30, 40))
            
            # Отрисовка камеры
            if camera_surface:
                screen.blit(camera_surface, (10, 10))
                
                # Рисуем текущий прямоугольник (если рисуем)
                if drawing:
                    x = min(start_x, end_x)
                    y = min(start_y, end_y)
                    width = abs(end_x - start_x)
                    height = abs(end_y - start_y)
                    
                    # Используем анимированную функцию рисования
                    joystick.draw_animated_selection(
                        camera_surface, 
                        x, y, width, height,
                        selection_animation_counter
                    )            
            
            # Обновление положения джойстика
            if joystick.interactive_mode:
                # Интерактивный режим управления с клавиатуры
                joystick.interactive_mode_function()
            else:
                # Автоматический режим трекинга
                joystick.update_position(dict_["error_x"], dict_["error_y"], dict_["width"], disp_fps)
            
            # Отрисовка интерфейса джойстика
            joystick.update_visualization(screen, camera_surface)
            
            # Обновление экрана
            pygame.display.flip()
            
            # Обновление счетчика анимации
            selection_animation_counter += selection_animation_speed
        
        # Завершение процессов
        image_proc.terminate()
        pygame.quit()

if __name__ == "__main__":
    main()

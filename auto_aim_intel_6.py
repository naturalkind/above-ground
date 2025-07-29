#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import uinput
import time
import math
import threading
import signal
import sys, os
import pygame
import numpy as np
import cv2
from multiprocessing import Process, Manager, Queue
from scipy.spatial import distance
from collections import deque
import concurrent
from concurrent.futures import ThreadPoolExecutor
from scipy.optimize import curve_fit

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
        self.tracker_weights = {'csrt': 0.6, 'kcf': 0.4}

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
        self.trackers['csrt'] = cv2.TrackerCSRT_create()
        self.trackers['csrt'].init(img, bbox)
        self.trackers['kcf'] = cv2.TrackerKCF_create()
        self.trackers['kcf'].init(img, bbox)
        self.init_switch = True

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
                self.image_process(img, bbox, img_center)
                self.lost_object_counter = 0
            else:
                self.lost_object_counter += 1
                if self.lost_object_counter >= self.max_lost_frames:
                    self.reinitialize_trackers(img)

        return img, self.obj_center, img_center

    def reinitialize_trackers(self, img):
        if self.recent_positions:
            last_known_position = self.recent_positions[-1]
            search_area = self.expand_search_area(last_known_position, img.shape)
            roi = img[search_area[1]:search_area[3], search_area[0]:search_area[2]]
            
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
        
        # ПИД-регуляторы
#        self.pid_x = PIDController(kp=0.5, ki=0.01, kd=0.05)
#        self.pid_y = PIDController(kp=0.5, ki=0.01, kd=0.05)

        self.pid_x = PIDController(kp=0.0005, ki=0.00001, kd=0.00005)
        self.pid_y = PIDController(kp=0.0005, ki=0.00001, kd=0.00005)


        
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
            'fps': 0,
            'step': 1  # Добавлено отображение шага
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
            'y': 128,     # Левый стик Y
            'throttle': 64  # Газ
        }
        
        # Параметры плавности для осей без возврата
        self.hold_interpolation_factor = 0.2
        self.hold_step_size = 1.5  # Скорость изменения позиции
#######################
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
#        self.hold_step_size = 1.5  # Скорость изменения позиции
        self.hold_step_size = 0.4
        
##############


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
        #y_pos = self.stick_center[1] + (self.y - 128) * 0.5
        
#        self.stick_center[iu] 
#        print ([u+100 for u in list(self.stick_center)])
#        temp_size = []
#        for iu, u in enumerate(self.stick_center):
#            
#            if iu == 0:
#                s = self.stick_center[iu]
#                self.stick_center = list(self.stick_center)
#                self.stick_center[iu] -= 10
#                print (iu, u, self.stick_center[iu])
#            elif iu == 1:
#                temp_size.appe     
#        
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
            text = self.font.render(f"{key}: {value:.2f}", True, (200, 200, 255))
            screen.blit(text, (self.screen_width // 2 + 30, y_pos))
            y_pos += 25
            
        # Отображение FPS
        fps_text = self.font.render(f"FPS: {self.display_values['fps']:.1f}", True, (0, 255, 255))
        screen.blit(fps_text, (self.screen_width - 150, self.screen_height - 30))
        
        # Отображение активных кнопок в интерактивном режиме
        if self.interactive_mode and self.active_buttons:
            active_text = self.small_font.render("Активные кнопки: " + ", ".join(self.active_buttons), True, (0, 255, 255))
            screen.blit(active_text, (20, self.screen_height - 30))

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

    def update_position(self, error_x, error_y, fps):
        """Обновление положения джойстика на основе ошибки трекинга"""
        current_time = time.time()
        dt = current_time - self.last_update
        self.last_update = current_time
        
        # ПИД-регуляторы
        pid_x = self.pid_x.update(0, error_x, dt)
#        pid_y = self.pid_y.update(0, error_y, dt)
        pid_y = self.pid_y.update(0, -error_y, dt)  # Инвертирована ошибка Y
        
        # Ограничение значений
        pid_x = max(-100, min(100, pid_x))
        pid_y = max(-100, min(100, pid_y))
        
        # Преобразование в значения джойстика
        self.rx = int(128 + pid_x)
#        self.ry = int(128 + pid_y)
#        self.ry = int(128 - pid_y)  # Инвертировать направление
        self.y = int(128 - pid_y)  # Инвертировать направление
        
        # Обновление значений для отображения
        self.display_values['error_x'] = error_x
        self.display_values['error_y'] = error_y
        self.display_values['pid_x'] = pid_x
        self.display_values['pid_y'] = pid_y
        self.display_values['fps'] = fps
        
        # Отправка команд
        self.emit_events()

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
            self.hold_values['y'] = min(255, self.hold_values['y'] + self.hold_step_size)
            self.active_buttons.add("S")
        elif keys[pygame.K_s]:
            self.hold_values['y'] = max(0, self.hold_values['y'] - self.hold_step_size)
            self.active_buttons.add("W")
            
            
         # коректное отображение           
#        if keys[pygame.K_w]:
#            self.hold_values['y'] = max(0, self.hold_values['y'] - self.hold_step_size)
#            self.active_buttons.add("W")
#        elif keys[pygame.K_s]:
#            self.hold_values['y'] = min(255, self.hold_values['y'] + self.hold_step_size)
#            self.active_buttons.add("S")

#        print (self.hold_values['y'])

        
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
        
#        # Газ (Q/E)
#        if keys[pygame.K_q]:
#            self.target_speed['throttle'] = -config['max_speed']
#            self.active_buttons.add("Q")
#        elif keys[pygame.K_e]:
#            self.target_speed['throttle'] = config['max_speed']
#            self.active_buttons.add("E")
#        else:
#            self.target_speed['throttle'] = 0

        # Газ (Q/E)
        print ("THR--", self.hold_step_size)
        if keys[pygame.K_q]:
            self.hold_values['throttle'] = max(0, self.hold_values['throttle'] - self.hold_step_size)
            self.active_buttons.add("Q")
        elif keys[pygame.K_e]:
            self.hold_values['throttle'] = min(255, self.hold_values['throttle'] + self.hold_step_size)
            self.active_buttons.add("E")

        
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
#                
                
                
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
#        if keys[pygame.K_r]:
#            for axis in ['x', 'y', 'rx', 'ry', 'throttle']:
#                self.target_speed[axis] = 0
#                self.current_speed[axis] = 0
#                self.smoothed_values[axis] = 128
#            self.x = self.y = self.rx = self.ry = self.throttle = 128
#            self.active_buttons.add("R")
        if keys[pygame.K_r]:
            # Сброс осей с удержанием позиции
            for axis in ['x', 'y', 'throttle']:
                if axis != 'throttle':
                    self.hold_values[axis] = 128
                    self.smoothed_values[axis] = 128
                    setattr(self, axis, 128)
                else:
                    self.hold_values[axis] = 64
                    self.smoothed_values[axis] = 64
                    setattr(self, axis, 64)
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
                    if event.key == pygame.K_a:  # Активация трекера
                        dict_["controller_init_tracker"] = True
                    elif event.key == pygame.K_d:  # Деактивация трекера
                        dict_["controller_init_tracker"] = False
                    elif event.key == pygame.K_TAB:  # Переключение режима управления
                        joystick.interactive_mode = not joystick.interactive_mode
                        # Сброс скоростей при переключении режима
                        for key in joystick.current_speed:
                            joystick.current_speed[key] = 0
                    elif event.key == pygame.K_ESCAPE:  # Выход
                        running = False
                
#            # Отправляем новый bbox в дочерний процесс
#            if current_bbox is not None:
#                bbox_queue.put(current_bbox)
#                current_bbox = None
            
            # Получение кадра из очереди
            if not frame_queue.empty():
                frame_data, width, height, proc_fps = frame_queue.get()
                
                # Декодирование изображения
                nparr = np.frombuffer(frame_data, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                
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
                joystick.update_position(dict_["error_x"], dict_["error_y"], disp_fps)
            
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
    
    
#Добавлен интерактивный режим управления:
#    Реализован метод interactive_mode() в классе FPVJoystickEmulator
#    Добавлен флаг interactive_mode для переключения между режимами
#    Переключение режимов по клавише TAB
#Управление в интерактивном режиме:
#    WASD: Управление левым стиком (X/Y)
#    IJKL: Управление правым стиком (RX/RY)
#    QE: Управление газом (Throttle)
#    R: Сброс всех осей в центр
#    1-8: Активация кнопок джойстика    

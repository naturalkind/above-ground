#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Комбинированный код: ELRS трансмиттер + FPV трекинг с эмуляцией джойстика
# Архитектура: Трекер (CV) → Калман (позиция) → PID → Евро-фильтр → Джойстик

import sys
import os
import platform
import pygame
import uinput
import numpy as np
import cv2
import json
import time
import math
import threading
import serial
import configparser
from enum import IntEnum
import serial.tools.list_ports
from scipy.spatial import distance
from collections import deque
import concurrent.futures
from concurrent.futures import ThreadPoolExecutor
from scipy.optimize import curve_fit
import csv
from datetime import datetime
import pygame.locals as pl
import struct
from queue import Queue as ThreadQueue, Empty

# ============================================================================
# Автоматическое переключение на английскую раскладку
# ============================================================================
def set_english_layout():
    system = platform.system()
    try:
        if system == "Linux":
            os.system("setxkbmap us")
        elif system == "Darwin":
            os.system("osascript -e 'tell application \"System Events\" to key code {0}'")
        elif system == "Windows":
            try:
                import win32api
                import win32con
                win32api.LoadKeyboardLayout("00000409", win32con.KLF_ACTIVATE)
            except ImportError:
                print("Для переключения раскладки на Windows установите pywin32")
    except Exception as e:
        print(f"Ошибка при переключении раскладки: {str(e)}")

set_english_layout()

# ============================================================================
# Константы и классы CRSF (из ELRS кода)
# ============================================================================
CRSF_SYNC_BYTE = 0xC8
CRSF_INVERTED_SYNC_BYTE = 0xEA
CRSF_MAX_PACKET_SIZE = 64

class CRSFPacketType(IntEnum):
    GPS = 0x02
    BATTERY_SENSOR = 0x08
    LINK_STATISTICS = 0x14
    ATTITUDE = 0x1E
    FLIGHT_MODE = 0x21
    DEVICE_PING = 0x28
    DEVICE_INFO = 0x29
    REQUEST_SETTINGS = 0x2A
    CHANNELS_INFO = 0x2F
    RC_CHANNELS_PACKED = 0x16
    VARIO = 0x07
    BARO_ALTITUDE = 0x09

def crc8_dvb_s2(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if (crc & 0x80):
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

def crsf_validate_frame(frame):
    length = frame[1]
    if length != len(frame) - 2:
        return False
    crc = crc8_dvb_s2(frame[2:-1])
    return crc == frame[-1]

def packCrsfToBytes(channels):
    if len(channels) != 16:
        raise ValueError('CRSF must have 16 channels')
    result = bytearray()
    bit_buffer = 0
    bits_in_buffer = 0
    for ch in channels:
        bit_buffer |= ch << bits_in_buffer
        bits_in_buffer += 11
        while bits_in_buffer >= 8:
            result.append(bit_buffer & 0xFF)
            bit_buffer >>= 8
            bits_in_buffer -= 8
    if bits_in_buffer > 0:
        result.append(bit_buffer & 0xFF)
    return result

def channelsCrsfToChannelsPacket(channels):
    payload = bytearray([CRSFPacketType.RC_CHANNELS_PACKED])
    payload += packCrsfToBytes(channels)
    length = len(payload) + 1
    packet = bytearray([CRSF_SYNC_BYTE, length]) + payload
    crc = crc8_dvb_s2(packet[2:])
    packet.append(crc)
    return packet

def map_axis(value):
    return int(1500 + value * 500)

def map_button(value):
    return 2000 if value else 1000

def map_to_crsf(value):
    return int((value - 1000) * 2047 / 1000)

# ==================== ДОБАВЛЕНО: Функции для телеметрии ====================
class CRSFTelemetryParser:
    def __init__(self):
        self.rx_buffer = bytearray()
        self.last_telemetry_time = time.time()
        
        # Данные телеметрии
        self.telemetry = {
            'voltage': "0.00",
            'current': "0.0",
            'fuel': "0%",
            'battery_format': "Unknown",
            'last_update': 0,
            
            'link_stats': {
                'uplink_rssi_avg': 0,
                'uplink_lq': 0,
                'downlink_lq': 0,
                'downlink_rssi': 0,
                'snr': 0,
                'last_update': 0
            },
            'attitude': {
                'pitch': 0.0,
                'roll': 0.0,
                'yaw': 0.0,
                'last_update': 0
            },
            'flight_mode': {
                'mode': "N/A",
                'last_update': 0
            }
        }
    
    def process_data(self, data):
        """Обработка входящих данных и извлечение пакетов"""
        self.rx_buffer.extend(data)
        
        pos = 0
        packets_found = []
        while pos < len(self.rx_buffer):
            # Ищем sync байт
            if self.rx_buffer[pos] in [CRSF_SYNC_BYTE, CRSF_INVERTED_SYNC_BYTE]:
                if len(self.rx_buffer) - pos < 2:
                    break
                
                frame_size = self.rx_buffer[pos + 1]
                total_packet_size = frame_size + 2
                
                if frame_size < 3 or frame_size > CRSF_MAX_PACKET_SIZE:
                    pos += 1
                    continue
                
                if len(self.rx_buffer) - pos < total_packet_size:
                    break
                
                packet = bytes(self.rx_buffer[pos:pos + total_packet_size])
                
                # Проверяем CRC
                if len(packet) >= 4:
                    crc_calculated = crc8_dvb_s2(packet[2:-1])
                    crc_received = packet[-1]
                    
                    if crc_calculated == crc_received:
                        self.parse_packet(packet, "Serial")
                        packets_found.append(packet)
                
                pos += total_packet_size
            else:
                pos += 1
        
        # Удаляем обработанные данные из буфера
        if pos > 0:
            del self.rx_buffer[:pos]
        
        return packets_found
    
    def parse_packet(self, packet, source="Unknown"):
        """Разбор пакета телеметрии"""
        if len(packet) < 4:
            return
        
        packet_type = packet[2]
        payload = packet[3:-1] if len(packet) > 4 else b''
        current_time = time.time()
        
        # Обновляем время последней телеметрии
        self.last_telemetry_time = current_time
        
        try:
            if packet_type == CRSFPacketType.BATTERY_SENSOR:
                self._parse_battery_sensor(payload, source)
                    
            elif packet_type == CRSFPacketType.LINK_STATISTICS and len(payload) >= 10:
                # Link statistics (0x14)
                uplink_rssi_1 = struct.unpack('b', payload[0:1])[0]  # RSSI антенна 1
                uplink_rssi_2 = struct.unpack('b', payload[1:2])[0]  # RSSI антенна 2
                uplink_lq = payload[2]  # Link Quality (uplink)
                downlink_lq = payload[3]  # Link Quality (downlink)
                downlink_rssi = struct.unpack('b', payload[8:9])[0]  # RSSI приемника
                snr = struct.unpack('b', payload[9:10])[0]  # SNR
                
                self.telemetry['link_stats'] = {
                    'uplink_rssi_avg': (uplink_rssi_1 + uplink_rssi_2) / 2,
                    'uplink_lq': uplink_lq,
                    'downlink_lq': downlink_lq,
                    'downlink_rssi': downlink_rssi,
                    'snr': snr,
                    'last_update': current_time
                }
                
            elif packet_type == CRSFPacketType.ATTITUDE and len(payload) >= 6:
                # Attitude (0x1E) - pitch, roll, yaw
                # Формат: int16_t pitch, roll, yaw (0.01 градус)
                pitch = struct.unpack('<h', payload[0:2])[0] * 0.01  # Градусы
                roll = struct.unpack('<h', payload[2:4])[0] * 0.01   # Градусы
                yaw = struct.unpack('<h', payload[4:6])[0] * 0.01    # Градусы
                
                self.telemetry['attitude'] = {
                    'pitch': pitch,
                    'roll': roll,
                    'yaw': yaw,
                    'last_update': current_time
                }
                
            elif packet_type == CRSFPacketType.FLIGHT_MODE and len(payload) > 0:
                # Flight mode (0x21)
                try:
                    flight_mode = payload.decode('ascii', errors='ignore').rstrip('\x00')
                    self.telemetry['flight_mode'] = {
                        'mode': flight_mode,
                        'last_update': current_time
                    }
                except:
                    pass
                    
        except Exception as e:
            print(f"Error parsing telemetry packet: {e}")
    
    def _parse_battery_sensor(self, payload, source):
        """Parse Battery Sensor packet (0x08) - ваша улучшенная версия"""
        if len(payload) < 8:
            return
        
        voltages = []
        
        # Big-endian interpretation (based on your data)
        try:
            # Big-endian interpretation (based on your data)
            voltage_raw = struct.unpack('>H', payload[0:2])[0]
            voltage = voltage_raw * 0.1
            voltages.append(("BE Standard", voltage, voltage_raw))
        except:
            pass
        
        # Little-endian interpretation (alternative)
        try:
            voltage_raw = struct.unpack('<H', payload[0:2])[0]
            voltage = voltage_raw * 0.1
            voltages.append(("LE Alternative", voltage, voltage_raw))
        except:
            pass
        
        # Find plausible voltage
        plausible_voltages = []
        for fmt, voltage, raw in voltages:
            if 3.0 <= voltage <= 26.0:  # Reasonable drone battery range
                plausible_voltages.append((fmt, voltage, raw))
        
        if plausible_voltages:
            fmt, voltage, raw = plausible_voltages[0]
            self.telemetry['voltage'] = f"{voltage:.2f}"
            self.telemetry['battery_format'] = f"{source}: {fmt}"
            self.telemetry['last_update'] = time.time()
            
            # Try to parse current
            if len(payload) >= 4:
                try:
                    # Big-endian для тока
                    current_raw = struct.unpack('>H', payload[2:4])[0]
                    current = current_raw * 0.1
                    if 0 <= current <= 200:  # Разумный диапазон тока
                        self.telemetry['current'] = f"{current:.1f}"
                    else:
                        self.telemetry['current'] = "0.0"
                except:
                    self.telemetry['current'] = "0.0"
            
            # Try to parse percentage (fuel)
            if len(payload) >= 8:
                try:
                    # Байт 7: емкость в процентах
                    percent = payload[7]
                    if 0 <= percent <= 100:
                        self.telemetry['fuel'] = f"{percent}%"
                    else:
                        # Альтернативно: биты 24-31 (bytes 4-7) могут содержать емкость в мАч
                        fuel_bytes = payload[4:8]
                        fuel_mah = struct.unpack('<I', fuel_bytes)[0]
                        if 0 <= fuel_mah <= 10000:  # Разумный диапазон
                            self.telemetry['fuel'] = f"{fuel_mah}mAh"
                        else:
                            self.telemetry['fuel'] = "N/A"
                except:
                    self.telemetry['fuel'] = "N/A"
            else:
                self.telemetry['fuel'] = "N/A"
        else:
            # Если не нашли правдоподобное напряжение
            self.telemetry['voltage'] = "N/A"
            self.telemetry['current'] = "N/A"
            self.telemetry['fuel'] = "N/A"
            self.telemetry['battery_format'] = f"{source}: Unknown format"
    
    def get_telemetry_summary(self):
        """Получить сводку телеметрии для отображения"""
        current_time = time.time()
        summary = []
        
        # Проверяем актуальность данных (не старше 3 секунд)
        if current_time - self.telemetry['last_update'] < 3:
            summary.append(f"Voltage: {self.telemetry['voltage']} V")
            summary.append(f"Current: {self.telemetry['current']} A")
            summary.append(f"Remaining: {self.telemetry['fuel']}")
        
        if current_time - self.telemetry['link_stats']['last_update'] < 3:
            link = self.telemetry['link_stats']
            summary.append(f"RSSI: {link['downlink_rssi']}dB LQ: {link['downlink_lq']}%")
        
        if current_time - self.telemetry['attitude']['last_update'] < 3:
            att = self.telemetry['attitude']
            summary.append(f"Pitch: {att['pitch']:+6.1f}° Roll: {att['roll']:+6.1f}°")
        
        if current_time - self.telemetry['flight_mode']['last_update'] < 3:
            mode = self.telemetry['flight_mode']
            summary.append(f"Mode: {mode['mode']}")
        
        return summary
    
    def get_formatted_telemetry(self):
        """Получить форматированные данные телеметрии"""
        current_time = time.time()
        formatted = {
            'battery': {
                'voltage': self.telemetry['voltage'],
                'current': self.telemetry['current'],
                'fuel': self.telemetry['fuel'],
                'last_update': self.telemetry['last_update']
            },
            'link_stats': None,
            'attitude': None,
            'flight_mode': None,
            'has_telemetry': False
        }
        
        # Проверяем актуальность данных
        if current_time - self.telemetry['last_update'] < 3:
            formatted['has_telemetry'] = True
        
        if current_time - self.telemetry['link_stats']['last_update'] < 3:
            formatted['link_stats'] = self.telemetry['link_stats']
            formatted['has_telemetry'] = True
        
        if current_time - self.telemetry['attitude']['last_update'] < 3:
            formatted['attitude'] = self.telemetry['attitude']
            formatted['has_telemetry'] = True
        
        if current_time - self.telemetry['flight_mode']['last_update'] < 3:
            formatted['flight_mode'] = self.telemetry['flight_mode']
            formatted['has_telemetry'] = True
        
        return formatted

# ============================================================================
# Классы для трекинга (из FPV кода)
# ============================================================================
class TrackerLib:
    def __init__(self):
        self.state = 0
        self.init_switch = False
        self.bbox = [0, 0, 0, 0]
        self.last_bbox = [0, 0, 0, 0]
        self.Error_track = False
        self.dst = 0
        self.obj_center = [0, 0]
        self.lost_object_counter = 0
        self.recent_positions = deque(maxlen=10)
        self.max_lost_frames = 30
        self.trackers = {}
        
        # Работает медленно без ускарения!
#        try:
#            _ = cv2.TrackerDaSiamRPN()
#            self.tracker_weights = {'csrt': 0.4, 'kcf': 0.3, 'dasiamrpn': 0.3}
#        except (AttributeError, cv2.error):
#            print("DaSiamRPN недоступен, используются только CSRT и KCF")
#            self.tracker_weights = {'csrt': 0.6, 'kcf': 0.4}
            
        self.tracker_weights = {'csrt': 0.6, 'kcf': 0.4}            
            
        # Фильтр Калмана для предсказания позиции объекта
        self.kalman = cv2.KalmanFilter(4, 2)
        self.kalman_initialized = False

    def init_kalman(self, center_x, center_y):
        self.kalman.transitionMatrix = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float32)
        
        self.kalman.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float32)
        
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
        self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.5
        self.kalman.errorCovPost = np.eye(4, dtype=np.float32) * 1
        
        self.kalman.statePost = np.array([
            [center_x], 
            [center_y], 
            [0], 
            [0]
        ], dtype=np.float32)
        
        self.kalman_initialized = True

    def update_kalman(self, measurement):
        prediction = self.kalman.predict()
        measured = np.array([[np.float32(measurement[0])], 
                             [np.float32(measurement[1])]])
        corrected = self.kalman.correct(measured)
        return (corrected[0], corrected[1])

    def get_center(self, img, x, y, w, h):
        xcentr = int(x + (w / 2))
        ycentr = int(y + (h / 2))
        cv2.circle(img, (xcentr, ycentr), radius=0, color=(0, 0, 255), thickness=5)
        return (xcentr, ycentr)

    def draw_box(self, img, bbox, color_border_box=(255, 0, 255)):
        x, y, w, h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
        cv2.rectangle(img, (x, y), ((x + w), (y + h)), color_border_box, 3, 1)
        return self.get_center(img, x, y, w, h)

    def increase_brightness(self, img, value=10):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        v = np.clip(v.astype(np.int32) + value, 0, 255).astype(np.uint8)
        final_hsv = cv2.merge((h, s, v))
        return cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)

    def init_tracker(self, img, bbox):
        self.state = 0
        self.trackers = {}
        
        center_x = bbox[0] + bbox[2] / 2
        center_y = bbox[1] + bbox[3] / 2
        self.init_kalman(center_x, center_y)
        
        model_path = "dasiamrpn_model.onnx"
        kernel_path = "dasiamrpn_kernel_r1.onnx"
        
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
        
        if valid_trackers:
            total_weight = sum(self.tracker_weights[n] for n in valid_trackers)
            self.tracker_weights = {n: self.tracker_weights[n] / total_weight 
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
            cv2.rectangle(img, (left, top), (right, bottom), (255, 0, 0), 1)
        else:
            corners = [(left, top), (right, top), (right, bottom), (left, bottom)]
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
                future_to_tracker = {executor.submit(tracker.update, img): name 
                                     for name, tracker in self.trackers.items()}
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
                
                center_x, center_y = bbox[0] + bbox[2] / 2, bbox[1] + bbox[3] / 2
                if self.kalman_initialized:
                    kalman_center = self.update_kalman((center_x, center_y))
                    center_x, center_y = kalman_center[0][0], kalman_center[1][0]
                    bbox[0] = int(center_x - bbox[2] / 2)
                    bbox[1] = int(center_y - bbox[3] / 2)
                
                self.image_process(img, bbox, img_center)
                self.lost_object_counter = 0
            else:
                self.lost_object_counter += 1
                if self.kalman_initialized and self.lost_object_counter < self.max_lost_frames:
                    prediction = self.kalman.predict()
                    pred_x = prediction[0][0]
                    pred_y = prediction[1][0]
                    cv2.circle(img, (int(pred_x), int(pred_y)), 10, (0, 255, 255), 2)
                    cv2.putText(img, "PREDICTION", (int(pred_x) - 30, int(pred_y) - 15), 
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
            if TS == 0:
                result = cv2.matchTemplate(roi, img[self.last_bbox[1]:self.last_bbox[1] + self.last_bbox[3], 
                                                self.last_bbox[0]:self.last_bbox[0] + self.last_bbox[2]], 
                                           cv2.TM_CCOEFF_NORMED)
                _, _, _, max_loc = cv2.minMaxLoc(result)
                new_bbox = (search_area[0] + max_loc[0], search_area[1] + max_loc[1], 
                            self.last_bbox[2], self.last_bbox[3])
                self.init_tracker(img, new_bbox)
                self.lost_object_counter = 0

    def expand_search_area(self, center, img_shape, factor=1.5):
        x, y = center
        w, h = self.last_bbox[2:]
        x1 = max(0, int(x - w * factor / 2))
        y1 = max(0, int(y - h * factor / 2))
        x2 = min(img_shape[1], int(x + w * factor / 2))
        y2 = min(img_shape[0], int(y + h * factor / 2))
        return (x1, y1, x2, y2)

class AdaptiveTargetPredictor:
    def __init__(self, history_size=30, min_samples=10):
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
            'linear': np.mean([np.mean((positions[:, i] - np.polyval(np.polyfit(times, positions[:, i], 1), times)) ** 2) for i in range(positions.shape[1])]),
            'quadratic': np.mean([np.mean((positions[:, i] - np.polyval(np.polyfit(times, positions[:, i], 2), times)) ** 2) for i in range(positions.shape[1])]),
        }
        if self.model == 'exponential':
            errors['exponential'] = np.mean([np.mean((positions[:, i] - (self.coeffs[i][0] * np.exp(self.coeffs[i][1] * times) + self.coeffs[i][2])) ** 2) for i in range(positions.shape[1])])

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

        mse = np.mean((positions - predicted) ** 2)
        confidence = 1 / (1 + mse)
        return confidence

# ============================================================================
# PID контроллер
# ============================================================================
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
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return output

# ============================================================================
# Основной класс комбинированной системы
# ============================================================================

class CombinedFPVSystem:
    def __init__(self, screen_width=1800, screen_height=1000):
        # Инициализация Pygame
        pygame.init()
        
        # Экран
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.screen = pygame.display.set_mode((screen_width, screen_height))
        pygame.display.set_caption("ELRS Transmitter + FPV Tracking System v2.0 (Архитектура: Трекер → Калман → PID → Евро-фильтр → Джойстик)")
        
        # Цвета
        self.WHITE = (255, 255, 255)
        self.BLACK = (0, 0, 0)
        self.PINK = (255, 105, 180)
        self.GREEN = (0, 255, 0)
        self.GRAY = (200, 200, 200)
        self.LIGHT_BLUE = (173, 216, 230)
        self.DARK_GRAY = (100, 100, 100)
        self.YELLOW = (255, 255, 0)
        self.RED = (255, 0, 0)
        self.BLUE = (0, 0, 255)
        self.CYAN = (0, 255, 255)
        self.ORANGE = (255, 165, 0)
        
        # Шрифты
        pygame.font.init()
        try:
            self.font = pygame.font.SysFont('Arial', 24)
            self.large_font = pygame.font.SysFont('Arial', 36)
            self.small_font = pygame.font.SysFont('Arial', 18)
        except:
            self.font = pygame.font.Font(None, 24)
            self.large_font = pygame.font.Font(None, 36)
            self.small_font = pygame.font.Font(None, 18)
        
        # Режимы работы
        self.mode = "local"  # "local" или "elrs"
        self.interactive_mode = False
        
        # ============ ИСПРАВЛЕНИЕ: Добавлен недостающий атрибут ============
        self.tracking_active = False  # Флаг активности трекинга
        # ===================================================================
        
        # ============ ДОБАВЛЕНЫ ПЕРЕМЕННЫЕ ДЛЯ ВЫДЕЛЕНИЯ МЫШЬЮ ============
        self.drawing = False  # Флаг процесса рисования
        self.start_x, self.start_y = -1, -1  # Начальная точка выделения
        self.end_x, self.end_y = -1, -1  # Конечная точка выделения
        self.current_bbox = None  # Текущий выделенный прямоугольник
        self.camera_rect = None  # Добавить эту строку!
        self.selection_animation_counter = 0  # Счетчик для анимации
        self.selection_animation_speed = 0.2  # Скорость анимации
        # ===================================================================
        
        # Настройки ELRS
        self.selected_serial_port = "Not Connected"
        self.selected_baud_rate = 921600  # Стандартный 921600 для ELRS
        self.baud_rates = [921600, 115200, 57600, 9600, 19200, 38400, 400000, 1870000, 3750000, 5250000]
        self.serial_ports = [port.device for port in serial.tools.list_ports.comports()]
        self.ser = None
        self.ser_lock = threading.Lock()
        self.running = True
        
        # Каналы CRSF (16 каналов) - СТАНДАРТНЫЕ ЗНАЧЕНИЯ 1000-2000
        self.crsf_channels = [1500] * 16  # Значения в микросекундах (1000-2000)
        
        # Джойстик Pygame
        self.joystick = None
        self.joystick_index = None
        self.joystick_names = ["None"] + [pygame.joystick.Joystick(i).get_name() 
                                         for i in range(pygame.joystick.get_count())]
        
        # Эмулятор uinput (для локального режима)
        self.uinput_device = None
        if self.mode == "local":
            self.init_uinput()
        
        # Параметры PID для трекинга (взяты из FPVJoystickEmulator)
        self.pid_x = PIDController(0.25, 0.01, 0.002)      # Для YAW
        self.pid_y = PIDController(0.35, 0.001, 0.001)    # Для PITCH
        self.pid_rx = PIDController(0.25, 0.01, 0.002)    # Для ROLL
        
        # Время последнего обновления
        self.last_update = time.time()
        
        # Позиции элементов интерфейса
        self.stick_center = (self.screen_width - 300, 300)
        self.stick_radius = 50
        self.r_stick_center = (self.screen_width - 100, 300)
        
        # === УЛУЧШЕННАЯ СИСТЕМА ПЛАВНОСТИ ===
        self.interactive_config = {
            'max_speed': 2.0,
            'acceleration': 0.15,
            'deceleration': 0.25,
            'dead_zone': 0.05,
            'smooth_factor': 0.8,
            'response_curve': 1.2
        }        
        
        # Целевые скорости (к чему стремимся)
        self.target_speed = {'x': 0, 'y': 0, 'rx': 0, 'ry': 0}
        
        # Текущие скорости (фактические)
        self.current_speed = {'x': 0, 'y': 0, 'rx': 0, 'ry': 0}
        
        # Сглаженные значения для плавного управления
        self.smoothed_values = {'x': 128, 'y': 128, 'rx': 128, 'ry': 128}
        
        # Значения джойстика (0-255)
        self.x = 128      # Левый стик X (YAW/A)
        self.y = 0        # Левый стик Y (THROTTLE/T) - начальное значение 0
        self.rx = 128     # Правый стик X (ROLL/R)
        self.ry = 128     # Правый стик Y (PITCH/E)
        
        # Для газа (улучшенная логика из CombinedFPVSystem)
        self._throttle_step = 0.5
        self._throttle_target = 0
        self._throttle_current = 0
        
        # Для AUX каналов - СТАНДАРТНЫЕ ЗНАЧЕНИЯ 1000-2000
        self.aux1_state = 1000  # DISARM по умолчанию (1000 = OFF, 2000 = ON)
        self.aux2_state = 1000  # ANGLE по умолчанию
        self.zero_pressed = False  # Для обработки кнопки 0
        
        # Кнопки uinput
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
        
        # Состояния кнопок для отображения
        self.active_buttons = set()
        
        # Текущие значения для отображения
        self.display_values = {
            'error_x': 0,
            'error_y': 0,
            'pid_x': 0,
            'pid_y': 0,
            'fps': 0,
            'confidence': 0,
            'tracking_status': "OK",
            'mode': "LOCAL",
            'serial_status': "Disconnected",
            'arm_state': "DISARMED",
            'angle_state': "OFF",
            'throttle_pct': 0,
            'hover_throttle': 120,
            'pid_scale': 0.4,
            'euro_filter': "ON",
            'filter_type': "АВТО (Евро)"
        }
        
        self.battery_analyzer = {
            'enabled': True,
            'voltage_history': deque(maxlen=100),
            'current_history': deque(maxlen=100),
            'detected_cells': 0,
            'battery_type': 'Unknown',
            'min_voltage_per_cell': 3.0,
            'max_voltage_per_cell': 4.35,
            'nominal_voltage_per_cell': 3.7,
            'charge_state': 'Unknown',
            'capacity_estimated': 0,
            'auto_detection_complete': False,
        }        
        ####################################
        self.quadcopter_profiles = {
            'tinywhoop': {
                'weight_class': 'Micro',
                'throttle_curve': 'aggressive',
                'pid_scale': 0.3,
                'max_throttle': 180,
                'hover_throttle': 80,
                'response_speed': 'fast',
                'battery_cells': 1,
            },
            '3inch': {
                'weight_class': 'Small',
                'throttle_curve': 'moderate',
                'pid_scale': 0.5,
                'max_throttle': 200,
                'hover_throttle': 100,
                'response_speed': 'medium',
                'battery_cells': 2,
            },
            '5inch': {
                'weight_class': 'Medium',
                'throttle_curve': 'smooth',
                'pid_scale': 0.7,
                'max_throttle': 220,
                'hover_throttle': 120,
                'response_speed': 'medium',
                'battery_cells': 4,
            },
            '7inch': {
                'weight_class': 'Large',
                'throttle_curve': 'very_smooth',
                'pid_scale': 1.0,
                'max_throttle': 240,
                'hover_throttle': 140,
                'response_speed': 'slow',
                'battery_cells': 6,
            },
            'x_class': {
                'weight_class': 'X-Class',
                'throttle_curve': 'ultra_smooth',
                'pid_scale': 1.5,
                'max_throttle': 255,
                'hover_throttle': 160,
                'response_speed': 'very_slow',
                'battery_cells': 8,
            }
        }
        
        self.current_profile = '3inch'
        self.profile_auto_detected = True        
        
        self.adaptive_control = {
            'learning_enabled': True,
            'throttle_efficiency': 1.0,
            'power_efficiency': 1.0,
            'adaptation_rate': 0.01,
            'throttle_hover_history': deque(maxlen=50),
            'voltage_drop_history': deque(maxlen=50),
            'performance_metrics': {
                'response_time': 0.0,
                'stability_score': 0.0,
                'power_usage': 0.0,
                'efficiency_score': 0.0,
            }
        }
        
        # ==================== ЕВРО-ФИЛЬТР ДЛЯ АВТОМАТИЧЕСКОГО УПРАВЛЕНИЯ ====================
        self.euro_filter_enabled = True  # Включить евро-фильтр для автоматического трекинга
        self.euro_filters = {}  # Состояния фильтров для каждой оси
        self.euro_filter_params = {
            'min_cutoff': 1.0,      # Минимальная частота среза (Гц)
            'beta': 0.1,           # Параметр скорости (чем больше, тем меньше задержка при быстрых движениях)
            'd_cutoff': 1.0        # Частота среза для производной (Гц)
        }
        
        # ==================== ДОБАВЛЕНО: ТЕЛЕМЕТРИЯ ====================
        self.telemetry_parser = CRSFTelemetryParser()
        self.telemetry_active = False
        self.last_rc_send_time = 0
        self.rc_send_interval = 0.02  # 50Hz для ELRS
        
        # ==================== ИСПРАВЛЕНИЕ ОШИБКИ МНОГОПРОЦЕССНОСТИ ====================
        # Используем потоки вместо процессов для избежания проблем с сериализацией
        self.tracking_data = {
            "init_tracker": False,
            "error_x": 0,
            "error_y": 0,
            "confidence": 1.0,
            "fps": 0
        }
        self.tracking_data_lock = threading.Lock()
        
        # Очереди для обмена данными между потоками
        self.frame_queue = ThreadQueue(maxsize=3)
        self.bbox_queue = ThreadQueue()
        self.tracking_results_queue = ThreadQueue(maxsize=10)
        
        # Флаг для остановки трекинга
        self.tracking_stop_event = threading.Event()
        
        # Трекер
        self.tracker_lib = TrackerLib()
        self.predictor = AdaptiveTargetPredictor(history_size=30, min_samples=10)
        self.current_tracker_bbox = None
        self.tracking_thread = None
        
        # Для FPS расчета
        self.proc_frame_count = 0
        self.proc_start_time = time.time()
        self.proc_fps = 0
        
        # Камера
        self.camera = None
        self.camera_surface = None
        
        # Загрузка конфигурации
        self.load_config()
        
        # Запуск потоков
        self.start_serial_thread()
        self.start_tracking_thread()
        
        # GUI элементы
        self.create_gui_elements()
        
        print("=" * 80)
        print("CombinedFPVSystem инициализирован с архитектурой:")
        print("  Трекер (CV) → Калман (позиция) → PID → Евро-фильтр → Джойстик")
        print("=" * 80)
        print("Евро-фильтр включен только для автоматического управления (трекинг)")
        print("Интерактивное управление (клавиатура) работает без фильтра для максимальной отзывчивости")
        print("=" * 80)
    
    # ==================== ФУНКЦИИ ЕВРО-ФИЛЬТРА ====================
    
    def one_euro_filter(self, axis, value, timestamp):
        """One Euro Filter для сглаживания значений джойстика при автоматическом управлении"""
        if axis not in self.euro_filters:
            # Инициализация состояния фильтра для новой оси
            self.euro_filters[axis] = {
                'prev_raw': value,
                'prev_filtered': value,
                'prev_time': timestamp,
                'prev_derivative': 0.0
            }
            return value
        
        state = self.euro_filters[axis]
        
        # Время с последнего обновления
        dt = timestamp - state['prev_time']
        if dt <= 0:
            return state['prev_filtered']
        
        # Вычисляем производную (скорость изменения)
        derivative = (value - state['prev_raw']) / dt
        
        # Сглаживаем производную
        alpha_d = 1.0 / (1.0 + 1.0 / (self.euro_filter_params['d_cutoff'] * dt))
        smoothed_derivative = alpha_d * derivative + (1.0 - alpha_d) * state['prev_derivative']
        
        # Адаптивная частота среза (зависит от скорости изменения)
        cutoff = self.euro_filter_params['min_cutoff'] + self.euro_filter_params['beta'] * abs(smoothed_derivative)
        
        # Коэффициент сглаживания (зависит от частоты среза и времени)
        alpha = 1.0 / (1.0 + 1.0 / (cutoff * dt))
        
        # Сглаживание значения
        filtered_value = alpha * value + (1.0 - alpha) * state['prev_filtered']
        
        # Обновление состояния фильтра
        state['prev_raw'] = value
        state['prev_filtered'] = filtered_value
        state['prev_time'] = timestamp
        state['prev_derivative'] = smoothed_derivative
        
        return filtered_value
    
    def reset_euro_filters(self):
        """Сброс состояния евро-фильтров"""
        self.euro_filters = {}
        print("Евро-фильтры сброшены")
    
    def smooth_interpolation(self, current, target, factor):
        """Экспоненциальное сглаживание для плавных переходов"""
        return current + (target - current) * factor
    
    def apply_response_curve(self, value, curve_factor):
        """Применение кривой отклика для более естественного управления"""
        if abs(value) < 0.001:
            return 0
        sign = 1 if value >= 0 else -1
        normalized = abs(value)
        curved = pow(normalized, curve_factor)
        return sign * curved
    
    def limit_position_to_circle(self, x_pos, y_pos, center_x, center_y, radius):
        """Ограничивает позицию стика внутри круга (из CombinedFPVSystem)"""
        dx = x_pos - center_x
        dy = y_pos - center_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance > radius:
            scale = radius / distance
            dx *= scale
            dy *= scale
        
        return int(center_x + dx), int(center_y + dy)
    
    # ==================== ДОБАВЛЕНЫ ФУНКЦИИ ДЛЯ ВЫДЕЛЕНИЯ МЫШЬЮ ====================
    
    def draw_animated_selection(self, surface, x, y, width, height, animation_counter):
        """Рисует анимированную рамку выделения с эффектом пульсации (из FPV кода)"""
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
    
    def handle_mouse_selection(self, event):
        """Обработка выделения объекта мышью (исправленная версия)"""
        
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # Левая кнопка мыши
                pos = event.pos
                
                # Проверяем, что клик был в области камеры
                if hasattr(self, 'camera_rect') and self.camera_rect:
                    if self.camera_rect.collidepoint(pos):
                        self.drawing = True
                        # Преобразуем экранные координаты в координаты камеры
                        self.start_x = pos[0] - self.camera_rect.x
                        self.start_y = pos[1] - self.camera_rect.y
                        self.end_x = self.start_x
                        self.end_y = self.start_y
                        print(f"Начало выделения: ({self.start_x}, {self.start_y})")
                        return True
                        
        elif event.type == pygame.MOUSEMOTION:
            if self.drawing and hasattr(self, 'camera_rect'):
                pos = event.pos
                # Ограничиваем координаты областью камеры
                if self.camera_rect.collidepoint(pos):
                    self.end_x = pos[0] - self.camera_rect.x
                    self.end_y = pos[1] - self.camera_rect.y
                return True
                    
        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1 and self.drawing:
                self.drawing = False
                
                # Вычисляем координаты прямоугольника
                x1 = min(self.start_x, self.end_x)
                y1 = min(self.start_y, self.end_y)
                x2 = max(self.start_x, self.end_x)
                y2 = max(self.start_y, self.end_y)
                
                # Проверяем размер прямоугольника
                width = x2 - x1
                height = y2 - y1
                
                if width > 10 and height > 10:  # Минимальный размер
                    # Масштабируем координаты для трекинга (k_scale = 1.2)
                    k_scale = 1.2
                    scaled_x1 = int(x1 * k_scale)
                    scaled_y1 = int(y1 * k_scale)
                    scaled_width = int(width * k_scale)
                    scaled_height = int(height * k_scale)
                    
                    # Создаем bbox для трекинга
                    bbox = (scaled_x1, scaled_y1, scaled_width, scaled_height)
                    
                    # Отправляем в очередь трекинга
                    try:
                        if not self.bbox_queue.full():
                            self.bbox_queue.put(bbox, block=False)
                            print(f"Объект выделен для трекинга: {bbox}")
                            
                            # Активируем трекинг
                            with self.tracking_data_lock:
                                self.tracking_data["init_tracker"] = True
                            self.tracking_active = True
                        else:
                            print("Очередь bbox переполнена!")
                    except Exception as e:
                        print(f"Ошибка отправки bbox: {e}")
                else:
                    print("Прямоугольник слишком мал для трекинга")
                
                # Сбрасываем координаты
                self.start_x, self.start_y = -1, -1
                self.end_x, self.end_y = -1, -1
                return True
        
        return False


    
    # ==================== СИСТЕМНЫЕ ФУНКЦИИ ====================
    
    def init_uinput(self):
        """Инициализация виртуального джойстика для локального режима"""
        try:
            self.uinput_device = uinput.Device([
                uinput.ABS_X + (0, 255, 0, 0),
                uinput.ABS_Y + (0, 255, 0, 0),
                uinput.ABS_RX + (0, 255, 0, 0),
                uinput.ABS_RY + (0, 255, 0, 0),
                uinput.BTN_JOYSTICK,
                uinput.BTN_TRIGGER,
                uinput.BTN_THUMB,
                uinput.BTN_THUMB2,
                uinput.BTN_TOP,
                uinput.BTN_TOP2,
                uinput.BTN_PINKIE,
                uinput.BTN_BASE,
            ], name="FPV Tracking System")
            print("Виртуальный джойстик создан")
        except Exception as e:
            print(f"Ошибка создания виртуального джойстика: {e}")
    
    def load_config(self):
        """Загрузка конфигурации из файла"""
        config = configparser.ConfigParser()
        config_file = 'controller_map.txt'
        
        if os.path.exists(config_file):
            config.read(config_file)
            
            if 'General' in config:
                general = config['General']
                self.mode = general.get('mode', 'local')
                self.selected_serial_port = general.get('serial_port', 'Not Connected')
                self.selected_baud_rate = general.getint('baud_rate', 921600)
                self.joystick_index = general.getint('joystick_index', None)
                
                # Загрузка параметров евро-фильтра
                self.euro_filter_enabled = general.getboolean('euro_filter_enabled', True)
                self.euro_filter_params['min_cutoff'] = general.getfloat('euro_min_cutoff', 1.0)
                self.euro_filter_params['beta'] = general.getfloat('euro_beta', 0.1)
        
        # Инициализация джойстика Pygame
        if self.joystick_index is not None and self.joystick_index >= 0:
            if self.joystick_index < pygame.joystick.get_count():
                self.joystick = pygame.joystick.Joystick(self.joystick_index)
                self.joystick.init()
    
    def save_config(self):
        """Сохранение конфигурации в файл"""
        config = configparser.ConfigParser()
        config['General'] = {
            'mode': self.mode,
            'serial_port': self.selected_serial_port,
            'baud_rate': str(self.selected_baud_rate),
            'joystick_index': str(self.joystick_index) if self.joystick_index is not None else '0',
            'euro_filter_enabled': str(self.euro_filter_enabled),
            'euro_min_cutoff': str(self.euro_filter_params['min_cutoff']),
            'euro_beta': str(self.euro_filter_params['beta'])
        }
        
        with open('controller_map.txt', 'w') as configfile:
            config.write(configfile)
        print("Конфигурация сохранена")
    
    def start_serial_thread(self):
        """Запуск потока для работы с последовательным портом"""
        if self.selected_serial_port != "Not Connected":
            try:
                self.ser = serial.Serial(
                    port=self.selected_serial_port,
                    baudrate=self.selected_baud_rate,
                    timeout=0.1,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    rtscts=False,
                    dsrdtr=False
                )
                
                # Очищаем буферы
                time.sleep(0.5)
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                time.sleep(0.1)
                
                self.display_values['serial_status'] = "Connected"
                print(f"Serial port {self.selected_serial_port} opened at {self.selected_baud_rate} baud")
                
            except Exception as e:
                print(f"Failed to open serial port: {e}")
                self.ser = None
                self.display_values['serial_status'] = "Error"
        
        # Запуск потока для отправки данных
        self.serial_write_thread = threading.Thread(target=self.serial_write_func, daemon=True)
        self.serial_write_thread.start()
        
        # Запуск потока для чтения телеметрии
        self.serial_read_thread = threading.Thread(target=self.serial_read_func, daemon=True)
        self.serial_read_thread.start()
    
    def serial_write_func(self):
        """Поток для отправки CRSF пакетов - ВАЖНО: 50Hz для ELRS"""
        while self.running:
            if self.ser and self.ser.is_open and self.mode == "elrs":
                current_time = time.time()
                
                # Отправляем RC пакеты с интервалом 20ms (50Hz)
                if current_time - self.last_rc_send_time >= self.rc_send_interval:
                    with self.ser_lock:
                        channels = self.get_crsf_channels()
                        packet = channelsCrsfToChannelsPacket(channels)
                        try:
                            self.ser.write(packet)
                            self.last_rc_send_time = current_time
                        except Exception as e:
                            print(f"Serial write error: {e}")
                    
                    time.sleep(0.005)  # Короткая пауза
                else:
                    time.sleep(0.001)  # Короткая пауза для избежания busy waiting
            else:
                time.sleep(0.1)
    
    def serial_read_func(self):
        """Поток для чтения телеметрии CRSF"""
        while self.running:
            if self.ser and self.ser.is_open:
                try:
                    # Читаем доступные данные
                    available = self.ser.in_waiting
                    if available > 0:
                        data = self.ser.read(available)
                        if data:
                            # Обрабатываем телеметрию
                            self.telemetry_parser.process_data(data)
                            self.telemetry_active = True
                    else:
                        time.sleep(0.001)
                        
                except Exception as e:
                    print(f"Serial read error: {e}")
                    time.sleep(0.1)
            else:
                time.sleep(0.1)
    
    def start_tracking_thread(self):
        """Запуск потока трекинга - ИСПРАВЛЕННАЯ ВЕРСИЯ (поток вместо процесса)"""
        # Очистка очередей перед запуском
        while not self.frame_queue.empty():
            try:
                self.frame_queue.get_nowait()
            except:
                pass
                
        while not self.bbox_queue.empty():
            try:
                self.bbox_queue.get_nowait()
            except:
                pass
        
        # Запуск потока трекинга
        self.tracking_thread = threading.Thread(
            target=self.tracking_task,
            daemon=True
        )
        self.tracking_thread.start()
        print("Tracking thread started")
    
    def tracking_task(self):
        """Задача трекинга - ИСПРАВЛЕННАЯ ВЕРСИЯ (в отдельном потоке)"""
        k_scale = 1.2
        
        # Инициализация камеры
        self.camera = cv2.VideoCapture(0)
        if not self.camera.isOpened():
            print("Error: Could not open camera")
            return
            
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
        self.proc_frame_count = 0
        self.proc_start_time = time.time()

        while not self.tracking_stop_event.is_set() and self.running:
            try:
                (status, frame) = self.camera.read()
                if not status:
                    time.sleep(0.01)
                    continue
                    
                frame = cv2.resize(frame, (int(frame.shape[1] * k_scale), int(frame.shape[0] * k_scale)))
                
                # Расчет FPS
                self.proc_frame_count += 1
                if self.proc_frame_count >= 10:
                    end_time = time.time()
                    self.proc_fps = self.proc_frame_count / (end_time - self.proc_start_time)
                    self.proc_start_time = time.time()
                    self.proc_frame_count = 0
                
                # Получение нового bbox
                if not self.bbox_queue.empty():
                    try:
                        self.current_tracker_bbox = self.bbox_queue.get_nowait()
                        self.tracker_lib.init_tracker(frame, self.current_tracker_bbox)
                        with self.tracking_data_lock:
                            self.tracking_data["init_tracker"] = True
                        print(f"Трекер инициализирован с bbox: {self.current_tracker_bbox}")
                    except:
                        pass
                
                # Обработка изображения
                init_tracker = self.tracking_data.get("init_tracker", False)
                _img, obj_center, img_center = self.tracker_lib.process_img_server(frame, init_tracker)
                
                # Обновление и предсказание позиции
                self.predictor.update([obj_center[0], obj_center[1]])
                future_position = self.predictor.predict(0.5)
                
                # Расчет уверенности трекинга
                confidence = self.predictor.get_confidence()
                
                # Расчет ошибки
                error_x = obj_center[0] - img_center[0]
                error_y = obj_center[1] - img_center[1]
                
                if future_position is not None:
                    predicted_error_y = future_position[1] - img_center[1]
                    error_y = 0.7 * error_y + 0.3 * predicted_error_y
                
                # Обновление данных трекинга
                with self.tracking_data_lock:
                    self.tracking_data.update({
                        'error_x': error_x,
                        'error_y': error_y,
                        'confidence': confidence,
                        'fps': self.proc_fps
                    })
                
                # Добавление информации на изображение
                cv2.putText(_img, f"FPS: {self.proc_fps:.1f}", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                cv2.putText(_img, f"Confidence: {confidence:.2f}", (10, 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, 
                            (0, 255, 0) if confidence > 0.7 else (0, 255, 255) if confidence > 0.4 else (0, 0, 255), 2)
                
                # Кодирование и отправка кадра
                _, img_encoded = cv2.imencode('.jpg', _img, encode_param)
                
                # Не блокируемся, если очередь полна
                try:
                    if not self.frame_queue.full():
                        self.frame_queue.put((img_encoded.tobytes(), _img.shape[1], _img.shape[0]), block=False)
                except:
                    pass
                    
                time.sleep(0.01)  # Небольшая пауза для снижения нагрузки
                
            except Exception as e:
                print(f"Tracking error: {e}")
                time.sleep(0.1)
        
        # Очистка при завершении
        if self.camera:
            self.camera.release()
        cv2.destroyAllWindows()
    
    # ==================== УПРАВЛЕНИЕ ====================
    
    def get_crsf_channels(self):
        """Преобразование текущих значений джойстика в каналы CRSF"""
        channels = [1500] * 16
        
        # Канал 1: YAW (A) - ось X левого стика
        channels[3] = int(1485 + (self.x - 128) * 500 / 127)
        
        # Канал 2: PITCH (E) - ось Y правого стика
        channels[1] = int(1485 + (self.ry - 128) * 500 / 127)
        
        # Канал 3: THROTTLE (T) - ось Y левого стика
        channels[2] = int(1000 + self.y * 1000 / 255)
        
        # Канал 4: ROLL (R) - ось X правого стика
        channels[0] = int(1485 + (self.rx - 128) * 500 / 127)
        
        # Канал 5: AUX1 - ARM
        channels[4] = self.aux1_state
        
        # Канал 6: AUX2 - ANGLE
        channels[5] = self.aux2_state
        
        # Ограничение значений
        for i in range(16):
            channels[i] = max(1000, min(2000, channels[i]))
        
        # Преобразование в формат CRSF (0-2047)
        crsf_channels = [map_to_crsf(ch) for ch in channels]
        
        return crsf_channels

    # ==================== АДАПТИВНЫЙ PID ДЛЯ THROTTLE ====================
    
    def update_adaptive_throttle_pid(self, error_y, current_voltage, dt):
        """Обновление адаптивного PID для throttle с учетом напряжения"""
        profile = self.quadcopter_profiles[self.current_profile]
        
        # 1. Базовый PID для вертикальной позиции
        pid_output = self.pid_y.update(0, error_y, dt)
        
        # 2. Коррекция на основе напряжения батареи
        if self.battery_analyzer['auto_detection_complete']:
            cells = self.battery_analyzer['detected_cells']
            nominal_voltage = self.battery_analyzer['nominal_voltage_per_cell'] * cells
            target_voltage = nominal_voltage * 0.95
            
            voltage_error = target_voltage - current_voltage
            
            voltage_kp = 0.1
            
            if current_voltage < nominal_voltage * 0.85:
                voltage_kp = 0.2
            elif current_voltage < nominal_voltage * 0.9:
                voltage_kp = 0.15
            
            voltage_correction = voltage_error * voltage_kp
            pid_output += voltage_correction
        
        # 3. Адаптация к профилю квадрокоптера
        pid_output *= profile['pid_scale']
        
        # 4. Ограничение
        max_output = profile['max_throttle'] - profile['hover_throttle']
        pid_output = max(-max_output, min(max_output, pid_output))
        
        # 5. Добавление базового газа для висения
        throttle_output = profile['hover_throttle'] + pid_output
        
        # 6. Учет эффективности
        throttle_output *= self.adaptive_control['throttle_efficiency']
        
        # Ограничение общего диапазона
        throttle_output = max(0, min(255, throttle_output))
        
        return throttle_output
    
    def update_adaptive_learning(self, throttle, voltage, current, dt):
        """Адаптивное обучение для оптимизации управления"""
        if not self.adaptive_control['learning_enabled']:
            return
        
        self.adaptive_control['throttle_hover_history'].append(throttle)
        
        if len(self.battery_analyzer['voltage_history']) >= 2:
            voltages = list(self.battery_analyzer['voltage_history'])
            voltage_drop = voltages[-2] - voltages[-1] if len(voltages) >= 2 else 0
            self.adaptive_control['voltage_drop_history'].append(voltage_drop)
        
        if len(self.adaptive_control['throttle_hover_history']) >= 10:
            avg_throttle = np.mean(list(self.adaptive_control['throttle_hover_history']))
            
            profile = self.quadcopter_profiles[self.current_profile]
            ideal_hover = profile['hover_throttle']
            
            if avg_throttle > 0:
                efficiency = ideal_hover / avg_throttle
                old_efficiency = self.adaptive_control['throttle_efficiency']
                new_efficiency = old_efficiency + (efficiency - old_efficiency) * self.adaptive_control['adaptation_rate']
                self.adaptive_control['throttle_efficiency'] = max(0.5, min(1.5, new_efficiency))
        
        if current > 0 and voltage > 0:
            power = current * voltage
            self.adaptive_control['performance_metrics']['power_usage'] = power
            
            if throttle > 0:
                power_per_throttle = power / throttle
                efficiency_score = 100 / (1 + power_per_throttle)
                self.adaptive_control['performance_metrics']['efficiency_score'] = efficiency_score

    # ==================== МЕТОДЫ АВТООПРЕДЕЛЕНИЯ БАТАРЕИ ====================
    
    def analyze_battery_type(self, voltage, current):
        """Анализ и автоопределение типа батареи и количества ячеек"""
        if not self.battery_analyzer['enabled']:
            return
        
        # Сохраняем историю
        self.battery_analyzer['voltage_history'].append(voltage)
        self.battery_analyzer['current_history'].append(current)
        
        if len(self.battery_analyzer['voltage_history']) < 20:
            return
        
        # Определяем максимальное напряжение
        max_voltage = max(self.battery_analyzer['voltage_history'])
        avg_current = np.mean(list(self.battery_analyzer['current_history'])) if self.battery_analyzer['current_history'] else 0
        
        # Определяем количество ячеек
        cells = 0
        battery_type = 'Unknown'
        
        # Литий-полимерные (LiPo) - 4.2V на ячейку
        if max_voltage > 4.0:
            if max_voltage >= 16.0:
                cells = round(max_voltage / 4.2)
                battery_type = 'LiPo'
            elif max_voltage >= 12.0:
                cells = 3
                battery_type = 'LiPo'
            elif max_voltage >= 8.0:
                cells = 2
                battery_type = 'LiPo'
            elif max_voltage >= 4.0:
                cells = 1
                battery_type = 'LiPo'
        
        # Литий-ионные (Li-Ion)
        elif max_voltage > 3.0:
            voltages = list(self.battery_analyzer['voltage_history'])
            if len(voltages) > 10:
                discharge_rate = (voltages[0] - voltages[-1]) / len(voltages)
                if discharge_rate < 0.01:
                    battery_type = 'Li-Ion'
        
        # Литий-железо-фосфатные (LiFePO4) - 3.65V на ячейку
        if max_voltage <= 4.0 and max_voltage >= 3.0:
            if cells == 0:
                cells = round(max_voltage / 3.65)
                if cells > 0:
                    battery_type = 'LiFePO4'
        
        # Обновляем информацию
        if cells > 0:
            self.battery_analyzer['detected_cells'] = cells
            self.battery_analyzer['battery_type'] = battery_type
            
            # Определяем параметры в зависимости от типа
            if battery_type == 'LiPo':
                self.battery_analyzer['max_voltage_per_cell'] = 4.2
                self.battery_analyzer['min_voltage_per_cell'] = 3.3
                self.battery_analyzer['nominal_voltage_per_cell'] = 3.7
            elif battery_type == 'LiHV':
                self.battery_analyzer['max_voltage_per_cell'] = 4.35
                self.battery_analyzer['min_voltage_per_cell'] = 3.5
                self.battery_analyzer['nominal_voltage_per_cell'] = 3.8
            elif battery_type == 'Li-Ion':
                self.battery_analyzer['max_voltage_per_cell'] = 4.2
                self.battery_analyzer['min_voltage_per_cell'] = 3.0
                self.battery_analyzer['nominal_voltage_per_cell'] = 3.6
            elif battery_type == 'LiFePO4':
                self.battery_analyzer['max_voltage_per_cell'] = 3.65
                self.battery_analyzer['min_voltage_per_cell'] = 2.5
                self.battery_analyzer['nominal_voltage_per_cell'] = 3.2
            
            # Оценочная емкость
            if avg_current > 0:
                time_samples = len(self.battery_analyzer['current_history'])
                avg_current_a = avg_current
                estimated_capacity = avg_current_a * 5 * 60
                self.battery_analyzer['capacity_estimated'] = int(estimated_capacity * 1000)
            
            # Определение состояния заряда
            nominal_voltage = self.battery_analyzer['nominal_voltage_per_cell'] * cells
            current_voltage = voltage
            
            if current_voltage >= nominal_voltage * 1.1:
                charge_state = 'Fully Charged'
            elif current_voltage >= nominal_voltage:
                charge_state = 'High'
            elif current_voltage >= nominal_voltage * 0.9:
                charge_state = 'Medium'
            elif current_voltage >= nominal_voltage * 0.8:
                charge_state = 'Low'
            else:
                charge_state = 'Critical'
            
            self.battery_analyzer['charge_state'] = charge_state
            self.battery_analyzer['auto_detection_complete'] = True
    
    def auto_detect_quadcopter_profile(self):
        """Автоматическое определение профиля квадрокоптера"""
        if not self.battery_analyzer['auto_detection_complete']:
            return '5inch'
        
        cells = self.battery_analyzer['detected_cells']
        
        # Определяем профиль по количеству ячеек
        if cells == 1:
            profile = 'tinywhoop'
        elif cells == 2:
            profile = '3inch'
        elif cells == 3 or cells == 4:
            avg_current = np.mean(list(self.battery_analyzer['current_history'])) if self.battery_analyzer['current_history'] else 0
            if avg_current > 30:
                profile = '5inch'
            else:
                profile = '3inch'
        elif cells == 5 or cells == 6:
            profile = '7inch'
        else:
            profile = 'x_class'
        
        self.current_profile = profile
        self.profile_auto_detected = True
        return profile




    def update_joystick_values(self):
        """Обновление значений джойстика на основе ошибки трекинга"""
        # Получаем данные телеметрии
        try:
            voltage = float(self.telemetry_parser.telemetry['voltage'])
            current = float(self.telemetry_parser.telemetry['current'])
        except:
            voltage = 0
            current = 0
            
        # Анализ батареи
        if voltage > 0:
            self.analyze_battery_type(voltage, current)
        
        # Автоопределение профиля
        if self.battery_analyzer['auto_detection_complete'] and not self.profile_auto_detected:
            self.auto_detect_quadcopter_profile()            
        
        # Получаем данные трекинга
        with self.tracking_data_lock:
            error_x = self.tracking_data.get("error_x", 0)
            error_y = self.tracking_data.get("error_y", 0)
            fps = self.tracking_data.get("fps", 0)
            confidence = self.tracking_data.get("confidence", 0)
        
        # ============ ИСПРАВЛЕНИЕ: Используем self.tracking_active ============
        if not self.interactive_mode and self.tracking_active:
            current_time = time.time()
            dt = current_time - self.last_update
            self.last_update = current_time
            
            # ПИД-регуляторы (архитектура: Калман -> PID)
            pid_x = self.pid_x.update(0, error_x, dt)  # Для YAW
            pid_y = self.pid_y.update(0, -error_y, dt) # ВЕРТИКАЛЬНОЕ ПОЛОЖЕНИЕ (error_y) → THROTTLE
            pid_rx = self.pid_rx.update(0, error_x, dt)  # Для ROLL
            
            # Ограничение
            pid_x = max(-20, min(20, pid_x))
            pid_y = max(-100, min(100, pid_y))
            pid_rx = max(-20, min(20, pid_rx))
            
            # Преобразование в диапазон 0-255
            SCALE = 127 / 100.0
            
            # Рассчитываем сырые значения
            raw_x = 128 + pid_x * SCALE
            raw_rx = 128 + pid_rx * SCALE
            
            
            # ФИКСИРОВАННЫЙ PITCH ВПЕРЕД
            # Вместо PID для расстояния - устанавливаем фиксированный наклон вперед
            fixed_pitch_percent = 0.24  
            SCALE = 127 / 100.0
            raw_pitch = 128 + int(fixed_pitch_percent * 100 * SCALE)
            
            # Ограничиваем PITCH для безопасности
            max_pitch_forward = 180  # Максимальный наклон вперед
            max_pitch_backward = 80  # Максимальный наклон назад
            self.ry = max(max_pitch_backward, min(max_pitch_forward, raw_pitch))            
            #################################
            
            base_throttle = self.display_values['hover_throttle']
            # Коррекция газа на основе напряжения
            if voltage > 0:
                throttle_output = self.update_adaptive_throttle_pid(pid_y, voltage, dt)
            else:
                # Без данных о напряжении
                throttle_output = base_throttle + pid_y * self.display_values['pid_scale']
            raw_throttle = max(50, min(220, throttle_output))
            
            
            # АРХИТЕКТУРА: PID -> Евро-фильтр (только для автоматического управления)
            if self.euro_filter_enabled and not self.interactive_mode:
                # Применяем евро-фильтр к выходам PID контроллера
                filtered_x = self.one_euro_filter('x_auto', raw_x, current_time)
                filtered_rx = self.one_euro_filter('rx_auto', raw_rx, current_time)
                #filtered_ry = self.one_euro_filter('ry_auto', raw_ry, current_time)
                filtered_throttle = self.one_euro_filter('throttle_auto', raw_throttle, current_time)
                
                
                # Ограничиваем значения после фильтрации
                self.x = max(0, min(255, int(round(filtered_x))))
                self.rx = max(0, min(255, int(round(filtered_rx))))
                #self.ry = max(0, min(255, int(round(filtered_ry))))
                self.y = max(0, min(255, int(round(filtered_throttle))))
                
                # Обновляем отображение типа фильтра
                self.display_values['filter_type'] = "АВТО (Евро)"
            else:
                # Простое экспоненциальное сглаживание (без евро-фильтра)
                smooth_factor = 0.7
                self.x = int(self.x + (raw_x - self.x) * smooth_factor)
                self.rx = int(self.rx + (raw_rx - self.rx) * smooth_factor)
                #self.ry = int(self.ry + (raw_ry - self.ry) * smooth_factor)
                self.y = int(self.y + (raw_throttle - self.y) * smooth_factor)
                
                # Ограничиваем значения
                self.x = max(0, min(255, self.x))
                self.rx = max(0, min(255, self.rx))
                #self.ry = max(0, min(255, self.ry))
                self.y = max(0, min(255, self.y))
                
                # Обновляем отображение типа фильтра
                self.display_values['filter_type'] = "АВТО (Простой)"
            
            # Обновление значений для отображения
            self.display_values['error_x'] = error_x
            self.display_values['error_y'] = error_y
            self.display_values['pid_x'] = pid_x
            self.display_values['pid_y'] = pid_y
            self.display_values['fps'] = fps
            self.display_values['confidence'] = confidence
            self.display_values['euro_filter'] = "ON" if self.euro_filter_enabled else "OFF"
            self.display_values['throttle_pct'] = int((self.y / 255) * 100)
            
            if confidence > 0.7:
                self.display_values['tracking_status'] = "OK"
            elif confidence > 0.4:
                self.display_values['tracking_status'] = "WARNING"
            else:
                self.display_values['tracking_status'] = "LOW"
        
        # Отправка команд
        if self.mode == "local" and self.uinput_device:
            self.emit_uinput_events()
    
    def emit_uinput_events(self):
        """Отправка событий uinput (для локального режима)"""
        if self.uinput_device:
            self.uinput_device.emit(uinput.ABS_X, self.x)
            self.uinput_device.emit(uinput.ABS_Y, self.y)
            self.uinput_device.emit(uinput.ABS_RX, self.rx)
            self.uinput_device.emit(uinput.ABS_RY, self.ry)
            
            for btn, state in self.button_states.items():
                btn_code = getattr(uinput, btn)
                self.uinput_device.emit(btn_code, 1 if state else 0)
    
    def handle_interactive_input(self):
        """УЛУЧШЕННАЯ обработка интерактивного ввода (без евро-фильтра)"""
        keys = pygame.key.get_pressed()
        self.active_buttons = set()
        
        current_time = time.time()
        dt = current_time - getattr(self, '_last_interactive_update', current_time)
        self._last_interactive_update = current_time
        dt = min(dt, 0.05)  # Ограничиваем dt для стабильности
        
        config = self.interactive_config
        
        # === 1. ГАЗ ===
        if keys[pygame.K_w]:
            self._throttle_target = min(255, self._throttle_target + self._throttle_step * dt * 60)
            self.active_buttons.add("W")
        elif keys[pygame.K_s]:
            self._throttle_target = max(0, self._throttle_target - self._throttle_step * dt * 60)
            self.active_buttons.add("S")
        
        # Плавное движение газа
        if abs(self._throttle_target - self._throttle_current) < 0.1:
            self._throttle_current = self._throttle_target
        else:
            direction = 1 if self._throttle_target > self._throttle_current else -1
            speed_change = self._throttle_step * direction * dt * 60
            if direction > 0:
                self._throttle_current = min(self._throttle_current + speed_change, self._throttle_target)
            else:
                self._throttle_current = max(self._throttle_current + speed_change, self._throttle_target)
        
        self.y = int(self._throttle_current)
        self.display_values['throttle_pct'] = int((self.y / 255) * 100)
        
        # === 2. ОСНОВНЫЕ ОСИ (ИНТЕРАКТИВНОЕ УПРАВЛЕНИЕ - БЕЗ ЕВРО-ФИЛЬТРА) ===
        target_x_speed = 0
        target_rx_speed = 0
        target_ry_speed = 0
        
        # ROLL (правый стик X) - J/L
        if keys[pygame.K_j]:
            target_rx_speed = -config['max_speed']
            self.active_buttons.add("J")
        elif keys[pygame.K_l]:
            target_rx_speed = config['max_speed']
            self.active_buttons.add("L")
        
        # PITCH (правый стик Y) - I/K
        if keys[pygame.K_i]:
            target_ry_speed = config['max_speed']
            self.active_buttons.add("I")
        elif keys[pygame.K_k]:
            target_ry_speed = -config['max_speed']
            self.active_buttons.add("K")
        
        # YAW (левый стик X) - A/D
        if keys[pygame.K_a]:
            target_x_speed = -config['max_speed']
            self.active_buttons.add("A")
        elif keys[pygame.K_d]:
            target_x_speed = config['max_speed']
            self.active_buttons.add("D")
        
        # === 3. ПЛАВНОЕ ИЗМЕНЕНИЕ С ПРИНУДИТЕЛЬНЫМ ВОЗВРАТОМ К ЦЕНТРУ ===
        axes_to_update = [
            ('x', target_x_speed, 128),
            ('rx', target_rx_speed, 128),
            ('ry', target_ry_speed, 128),
        ]
        
        for axis_name, target_speed, center_value in axes_to_update:
            current_speed = self.current_speed[axis_name]
            
            # Определяем, движемся ли мы к цели или к центру
            moving_to_target = abs(target_speed) > 0.001
            moving_to_center = abs(target_speed) < 0.001 and abs(current_speed) > 0.001
            
            # Разные параметры для ускорения и замедления
            if moving_to_target:
                acceleration = config['acceleration']
                final_target = target_speed
            else:
                acceleration = config['deceleration'] * 1.5
                final_target = 0
            
            # Плавное изменение скорости
            if abs(final_target - current_speed) < config['dead_zone']:
                new_speed = final_target
            else:
                direction = 1 if final_target > current_speed else -1
                speed_change = acceleration * direction * dt * 60
                new_speed = current_speed + speed_change
                
                if direction > 0:
                    new_speed = min(new_speed, final_target)
                else:
                    new_speed = max(new_speed, final_target)
            
            self.current_speed[axis_name] = new_speed
            
            # Принудительный возврат к центру
            if abs(new_speed) < 0.01:
                if axis_name == 'x':
                    self.x = center_value
                    self.smoothed_values['x'] = float(center_value)
                elif axis_name == 'rx':
                    self.rx = center_value
                    self.smoothed_values['rx'] = float(center_value)
                elif axis_name == 'ry':
                    self.ry = center_value
                    self.smoothed_values['ry'] = float(center_value)
                continue
            
            # Применяем кривую отклика
            curved_speed = self.apply_response_curve(new_speed, config['response_curve'])
            
            # Рассчитываем целевое значение
            speed_ratio = curved_speed / config['max_speed']
            raw_value = center_value + speed_ratio * 127
            
            # Экспоненциальное сглаживание (без евро-фильтра для интерактивного режима)
            smoothed_value = self.smooth_interpolation(
                self.smoothed_values[axis_name], 
                raw_value, 
                config['smooth_factor'] * (0.7 if moving_to_center else 1.0)
            )
            
            int_value = int(round(smoothed_value))
            
            if axis_name == 'x':
                self.x = max(0, min(255, int_value))
                self.smoothed_values['x'] = float(self.x)
            elif axis_name == 'rx':
                self.rx = max(0, min(255, int_value))
                self.smoothed_values['rx'] = float(self.rx)
            elif axis_name == 'ry':
                self.ry = max(0, min(255, int_value))
                self.smoothed_values['ry'] = float(self.ry)
        
        # Обновляем отображение типа фильтра для интерактивного режима
        self.display_values['filter_type'] = "РУЧНОЙ (Без фильтра)"
        
        # === 4. AUX КАНАЛЫ ===
        # ARM (Q/E) - СТАНДАРТНЫЕ ЗНАЧЕНИЯ 1000-2000
        if keys[pygame.K_q]:
            self.aux1_state = 1000  # DISARM
            self.display_values['arm_state'] = "DISARMED"
            self.active_buttons.add("Q")
        elif keys[pygame.K_e]:
            self.aux1_state = 2000  # ARM
            self.display_values['arm_state'] = "ARMED"
            self.active_buttons.add("E")
        
        # ANGLE (0)
        if keys[pygame.K_0] and not self.zero_pressed:
            self.zero_pressed = True
            self.aux2_state = 2000 if self.aux2_state == 1000 else 1000
            self.display_values['angle_state'] = "ON" if self.aux2_state == 2000 else "OFF"
            self.active_buttons.add("0")
        elif not keys[pygame.K_0]:
            self.zero_pressed = False
        
        # Обновление отображаемых состояний
        self.display_values['arm_state'] = "ARMED" if self.aux1_state == 2000 else "DISARMED"
        self.display_values['angle_state'] = "ON" if self.aux2_state == 2000 else "OFF"
        
        # === 5. СПЕЦИАЛЬНЫЕ КЛАВИШИ ===
        # Сброс всех осей кроме газа (R)
        if keys[pygame.K_r]:
            self.x = 128
            self.rx = 128
            self.ry = 128
            
            for axis in ['x', 'rx', 'ry']:
                self.current_speed[axis] = 0
                self.smoothed_values[axis] = 128.0
            
            self.active_buttons.add("R")
        
        # Сброс газа в нейтраль (X)
        if keys[pygame.K_x]:
            self._throttle_target = 0
            self.active_buttons.add("X")
        
        # Переключение режима трекинга (T)
        if keys[pygame.K_t] and not getattr(self, '_t_pressed', False):
            self._t_pressed = True
            self.tracking_active = not self.tracking_active
            self.active_buttons.add("T")
        elif not keys[pygame.K_t]:
            self._t_pressed = False
        
        # Переключение евро-фильтра (F)
        if keys[pygame.K_f] and not getattr(self, '_f_pressed', False):
            self._f_pressed = True
            self.euro_filter_enabled = not self.euro_filter_enabled
            if not self.euro_filter_enabled:
                self.reset_euro_filters()
            self.display_values['euro_filter'] = "ON" if self.euro_filter_enabled else "OFF"
            self.active_buttons.add("F")
            print(f"Euro filter {'enabled' if self.euro_filter_enabled else 'disabled'}")
        elif not keys[pygame.K_f]:
            self._f_pressed = False
        
        # Отправка команд
        if self.mode == "local" and self.uinput_device:
            self.emit_uinput_events()
    
    # ==================== GUI И ОТОБРАЖЕНИЕ ====================
    
    def create_gui_elements(self):
        """Создание GUI элементов"""
        # Кнопки режимов
        self.mode_local_rect = pygame.Rect(20, 20, 200, 40)
        self.mode_elrs_rect = pygame.Rect(240, 20, 200, 40)
        
        # Кнопка переключения интерактивного режима
        self.interactive_rect = pygame.Rect(460, 20, 300, 40)
        
        # Поля для настроек ELRS
        self.serial_rect = pygame.Rect(20, 80, 350, 30)
        self.baud_rect = pygame.Rect(20, 120, 350, 30)
        
        # Кнопки управления фильтрами
        self.euro_filter_toggle_rect = pygame.Rect(1100, 20, 200, 30)
        self.reset_filters_rect = pygame.Rect(1300, 20, 200, 30)
        self.save_config_rect = pygame.Rect(1480, 20, 200, 30)
        
        # Панель телеметрии
        self.telemetry_rect = pygame.Rect(20, 500, 350, 300)
        
        # Каналы (внизу экрана)
        self.channel_boxes = {}
        channel_box_width = 120
        channel_box_height = 80
        channel_x_offset = 10
        channel_y_offset_top = self.screen_height - (2 * channel_box_height) - 30
        channel_y_offset_bottom = self.screen_height - channel_box_height - 20

        for i in range(8):
            self.channel_boxes[f'channel_{i + 1}'] = pygame.Rect(
                channel_x_offset + i * (channel_box_width + 10),
                channel_y_offset_top,
                channel_box_width,
                channel_box_height
            )
        for i in range(8, 16):
            self.channel_boxes[f'channel_{i + 1}'] = pygame.Rect(
                channel_x_offset + (i - 8) * (channel_box_width + 10),
                channel_y_offset_bottom,
                channel_box_width,
                channel_box_height
            )
    
    def draw_interface(self):
        """Отрисовка интерфейса"""
        # Очистка экрана
        self.screen.fill((30, 30, 40))
        
        # Отрисовка камеры
        if self.camera_surface:
            camera_rect = self.camera_surface.get_rect()
            camera_rect.topleft = (400, 100)
            self.screen.blit(self.camera_surface, camera_rect)
            
            # Сохраняем rect камеры для обработки мыши
            self.camera_rect = camera_rect
            
            # Рамка вокруг камеры
            pygame.draw.rect(self.screen, (80, 80, 100), camera_rect, 2)
            
            # Отрисовка текущего прямоугольника выделения
            if self.drawing and self.start_x != -1 and self.end_x != -1:
                x = min(self.start_x, self.end_x) + camera_rect.x
                y = min(self.start_y, self.end_y) + camera_rect.y
                width = abs(self.end_x - self.start_x)
                height = abs(self.end_y - self.start_y)
                
                # Рисуем прямоугольник выделения
                pygame.draw.rect(self.screen, (0, 255, 0), 
                               (x, y, width, height), 2)
                
                # Анимированные углы
                self.draw_animated_selection(
                    self.screen, 
                    x, y, width, height,
                    self.selection_animation_counter
                )

        
        # === КНОПКИ РЕЖИМОВ ===
        pygame.draw.rect(self.screen, 
                        (0, 180, 0) if self.mode == "local" else (70, 70, 70),
                        self.mode_local_rect, border_radius=5)
        pygame.draw.rect(self.screen, 
                        (0, 200, 0) if self.mode == "local" else (100, 100, 100),
                        self.mode_local_rect, 2, border_radius=5)
        mode_local_text = self.font.render("LOCAL MODE", True, self.WHITE)
        self.screen.blit(mode_local_text, 
                        (self.mode_local_rect.x + (self.mode_local_rect.width - mode_local_text.get_width()) // 2,
                         self.mode_local_rect.y + 10))
        
        pygame.draw.rect(self.screen, 
                        (0, 180, 0) if self.mode == "elrs" else (70, 70, 70),
                        self.mode_elrs_rect, border_radius=5)
        pygame.draw.rect(self.screen, 
                        (0, 200, 0) if self.mode == "elrs" else (100, 100, 100),
                        self.mode_elrs_rect, 2, border_radius=5)
        mode_elrs_text = self.font.render("ELRS MODE", True, self.WHITE)
        self.screen.blit(mode_elrs_text, 
                        (self.mode_elrs_rect.x + (self.mode_elrs_rect.width - mode_elrs_text.get_width()) // 2,
                         self.mode_elrs_rect.y + 10))
        
        # === КНОПКА ИНТЕРАКТИВНОГО РЕЖИМА ===
        interactive_color = (0, 180, 0) if self.interactive_mode else (180, 0, 0)
        pygame.draw.rect(self.screen, interactive_color, self.interactive_rect, border_radius=5)
        pygame.draw.rect(self.screen, (0, 200, 0) if self.interactive_mode else (200, 0, 0), 
                        self.interactive_rect, 2, border_radius=5)
        interactive_text = self.font.render(
            f"INTERACTIVE: {'ON' if self.interactive_mode else 'OFF'}", 
            True, self.WHITE)
        self.screen.blit(interactive_text, 
                        (self.interactive_rect.x + (self.interactive_rect.width - interactive_text.get_width()) // 2,
                         self.interactive_rect.y + 10))
        
        # === НАСТРОЙКИ ELRS ===
        pygame.draw.rect(self.screen, self.LIGHT_BLUE, self.serial_rect, border_radius=3)
        pygame.draw.rect(self.screen, self.BLACK, self.serial_rect, 2, border_radius=3)
        serial_text = self.font.render(f"Serial: {self.selected_serial_port}", True, self.BLACK)
        self.screen.blit(serial_text, (self.serial_rect.x + 10, self.serial_rect.y + 5))
        
        pygame.draw.rect(self.screen, self.LIGHT_BLUE, self.baud_rect, border_radius=3)
        pygame.draw.rect(self.screen, self.BLACK, self.baud_rect, 2, border_radius=3)
        baud_text = self.font.render(f"Baud: {self.selected_baud_rate}", True, self.BLACK)
        self.screen.blit(baud_text, (self.baud_rect.x + 10, self.baud_rect.y + 5))
        
        # === ИНФОРМАЦИОННАЯ ПАНЕЛЬ ===
        info_rect = pygame.Rect(20, 200, 350, 300)
        pygame.draw.rect(self.screen, (40, 40, 60), info_rect, border_radius=8)
        pygame.draw.rect(self.screen, (80, 80, 100), info_rect, 2, border_radius=8)
        
        info_title = self.font.render("System Information", True, (200, 200, 255))
        self.screen.blit(info_title, (info_rect.x + (info_rect.width - info_title.get_width()) // 2, 
                                     info_rect.y + 10))
        
        y_pos = info_rect.y + 40
        info_items = [
            (f"FPS: {self.display_values['fps']:.1f}", (200, 200, 255)),
            (f"Error X: {self.display_values['error_x']:.2f}", (200, 200, 255)),
            (f"Error Y: {self.display_values['error_y']:.2f}", (200, 200, 255)),
            (f"PID X: {self.display_values['pid_x']:.2f}", (200, 200, 255)),
            (f"PID Y: {self.display_values['pid_y']:.2f}", (200, 200, 255)),
            (f"Confidence: {self.display_values['confidence']:.2f}", 
             (0, 255, 0) if self.display_values['confidence'] > 0.7 else 
             (255, 165, 0) if self.display_values['confidence'] > 0.4 else 
             (255, 0, 0)),
            (f"Tracking: {self.display_values['tracking_status']}", 
             (0, 255, 0) if self.display_values['tracking_status'] == "OK" else 
             (255, 165, 0) if self.display_values['tracking_status'] == "WARNING" else 
             (255, 0, 0)),
            (f"Mode: {self.display_values['mode']}", (200, 200, 255)),
            (f"Serial: {self.display_values['serial_status']}", 
             (0, 255, 0) if self.display_values['serial_status'] == "Connected" else 
             (255, 0, 0) if self.display_values['serial_status'] == "Error" else 
             (255, 165, 0)),
            (f"Throttle: {self.display_values['throttle_pct']}%", (200, 200, 0)),
            (f"Евро-фильтр: {self.display_values['euro_filter']}", 
             (0, 255, 0) if self.display_values['euro_filter'] == "ON" else (255, 0, 0)),
            (f"Тип фильтра: {self.display_values['filter_type']}", (200, 200, 255)),
            (f"Трекинг: {'ВКЛ' if self.tracking_active else 'ВЫКЛ'}", 
             (0, 255, 0) if self.tracking_active else (255, 0, 0)),
        ]
        
        for text, color in info_items:
            text_surface = self.small_font.render(text, True, color)
            self.screen.blit(text_surface, (info_rect.x + 20, y_pos))
            y_pos += 20
        
        # === КНОПКИ УПРАВЛЕНИЯ ФИЛЬТРАМИ ===
        # Кнопка включения/выключения евро-фильтра
        pygame.draw.rect(self.screen, 
                        (0, 180, 0) if self.euro_filter_enabled else (180, 0, 0),
                        self.euro_filter_toggle_rect, border_radius=5)
        pygame.draw.rect(self.screen, 
                        (0, 200, 0) if self.euro_filter_enabled else (200, 0, 0),
                        self.euro_filter_toggle_rect, 2, border_radius=5)
        euro_text = self.small_font.render(
            f"Euro Filter: {'ON' if self.euro_filter_enabled else 'OFF'}", 
            True, self.WHITE)
        self.screen.blit(euro_text, 
                        (self.euro_filter_toggle_rect.x + 10, 
                         self.euro_filter_toggle_rect.y + 8))
        
        # Кнопка сброса фильтров
        pygame.draw.rect(self.screen, (100, 100, 200), 
                        self.reset_filters_rect, border_radius=5)
        pygame.draw.rect(self.screen, (150, 150, 255), 
                        self.reset_filters_rect, 2, border_radius=5)
        reset_text = self.small_font.render("Reset Filters", True, self.WHITE)
        self.screen.blit(reset_text, 
                        (self.reset_filters_rect.x + 10, 
                         self.reset_filters_rect.y + 8))
        
        # Кнопка сохранения конфигурации
        pygame.draw.rect(self.screen, (100, 200, 100), 
                        self.save_config_rect, border_radius=5)
        pygame.draw.rect(self.screen, (150, 255, 150), 
                        self.save_config_rect, 2, border_radius=5)
        save_text = self.small_font.render("Save Config", True, self.WHITE)
        self.screen.blit(save_text, 
                        (self.save_config_rect.x + 10, 
                         self.save_config_rect.y + 8))
        
        # ==================== ПАНЕЛЬ ТЕЛЕМЕТРИИ ====================
        current_time = time.time()
        
        pygame.draw.rect(self.screen, (40, 40, 60), self.telemetry_rect, border_radius=8)
        pygame.draw.rect(self.screen, (80, 80, 100), self.telemetry_rect, 2, border_radius=8)
        
        telemetry_title = self.font.render("ELRS Telemetry", True, (200, 200, 255))
        self.screen.blit(telemetry_title, (self.telemetry_rect.x + (self.telemetry_rect.width - telemetry_title.get_width()) // 2, 
                                          self.telemetry_rect.y + 10))
        
        y_pos = self.telemetry_rect.y + 40
        
        # Проверяем, есть ли актуальные данные телеметрии
        has_recent_telemetry = current_time - self.telemetry_parser.telemetry['last_update'] < 3
        
        if has_recent_telemetry:
            # Battery info
            voltage_color = (0, 255, 0) if self.telemetry_parser.telemetry['voltage'] != "N/A" else (255, 165, 0)
            
            batt_text = self.small_font.render(f"Voltage: {self.telemetry_parser.telemetry['voltage']} V", 
                                              True, voltage_color)
            self.screen.blit(batt_text, (self.telemetry_rect.x + 10, y_pos))
            y_pos += 20
            
            current_text = self.small_font.render(f"Current: {self.telemetry_parser.telemetry['current']} A", 
                                                 True, voltage_color)
            self.screen.blit(current_text, (self.telemetry_rect.x + 10, y_pos))
            y_pos += 20
            
            fuel_text = self.small_font.render(f"Remaining: {self.telemetry_parser.telemetry['fuel']}", 
                                              True, voltage_color)
            self.screen.blit(fuel_text, (self.telemetry_rect.x + 10, y_pos))
            y_pos += 25
            
            # Link Stats (если есть)
            if current_time - self.telemetry_parser.telemetry['link_stats']['last_update'] < 3:
                link = self.telemetry_parser.telemetry['link_stats']
                rssi_color = (0, 255, 0) if link['downlink_rssi'] > -80 else \
                            (255, 165, 0) if link['downlink_rssi'] > -100 else \
                            (255, 0, 0)
                
                rssi_text = self.small_font.render(f"RSSI: {link['downlink_rssi']}dB LQ: {link['downlink_lq']}%", 
                                                  True, rssi_color)
                self.screen.blit(rssi_text, (self.telemetry_rect.x + 10, y_pos))
                y_pos += 20
                
                snr_text = self.small_font.render(f"SNR: {link['snr']}dB", True, rssi_color)
                self.screen.blit(snr_text, (self.telemetry_rect.x + 10, y_pos))
                y_pos += 25
            
            # Attitude (если есть)
            if current_time - self.telemetry_parser.telemetry['attitude']['last_update'] < 3:
                att = self.telemetry_parser.telemetry['attitude']
                attitude_text = self.small_font.render(f"Pitch: {att['pitch']:+6.1f}°", True, (200, 200, 255))
                self.screen.blit(attitude_text, (self.telemetry_rect.x + 10, y_pos))
                y_pos += 20
                
                attitude_text = self.small_font.render(f"Roll:  {att['roll']:+6.1f}°", True, (200, 200, 255))
                self.screen.blit(attitude_text, (self.telemetry_rect.x + 10, y_pos))
                y_pos += 20
                
                attitude_text = self.small_font.render(f"Yaw:   {att['yaw']:+6.1f}°", True, (200, 200, 255))
                self.screen.blit(attitude_text, (self.telemetry_rect.x + 10, y_pos))
                y_pos += 25
            
            # Flight Mode (если есть)
            if current_time - self.telemetry_parser.telemetry['flight_mode']['last_update'] < 3:
                mode = self.telemetry_parser.telemetry['flight_mode']
                mode_color = (0, 255, 0) if mode['mode'] in ["MANUAL", "STABILIZE"] else \
                            (255, 165, 0) if mode['mode'] in ["AUTO", "RTL"] else \
                            (255, 255, 0)
                mode_text = self.small_font.render(f"Mode: {mode['mode']}", True, mode_color)
                self.screen.blit(mode_text, (self.telemetry_rect.x + 10, y_pos))
        else:
            # Нет телеметрии
            no_data_text = self.small_font.render("No telemetry data", True, (150, 150, 150))
            self.screen.blit(no_data_text, (self.telemetry_rect.x + (self.telemetry_rect.width - no_data_text.get_width()) // 2, 
                                          self.telemetry_rect.y + 60))
            if self.mode == "elrs" and self.display_values['serial_status'] == "Connected":
                waiting_text = self.small_font.render("Waiting for ELRS handshake...", True, (255, 165, 0))
                self.screen.blit(waiting_text, (self.telemetry_rect.x + (self.telemetry_rect.width - waiting_text.get_width()) // 2, 
                                              self.telemetry_rect.y + 85))
        
        # === ARM/ANGLE ИНДИКАТОРЫ ===
        arm_rect = pygame.Rect(780, 20, 150, 40)
        angle_rect = pygame.Rect(950, 20, 150, 40)
        
        # ARM индикатор
        arm_color = (50, 255, 50) if self.display_values['arm_state'] == "ARMED" else (255, 50, 50)
        pygame.draw.rect(self.screen, arm_color, arm_rect, border_radius=5)
        pygame.draw.rect(self.screen, self.BLACK, arm_rect, 2, border_radius=5)
        arm_text = self.font.render(f"{self.display_values['arm_state']}", True, self.WHITE)
        self.screen.blit(arm_text, (arm_rect.x + (arm_rect.width - arm_text.get_width()) // 2,
                                   arm_rect.y + 10))
        
        # ANGLE индикатор
        angle_color = (50, 255, 50) if self.display_values['angle_state'] == "ON" else (100, 100, 100)
        pygame.draw.rect(self.screen, angle_color, angle_rect, border_radius=5)
        pygame.draw.rect(self.screen, self.BLACK, angle_rect, 2, border_radius=5)
        angle_text = self.font.render(f"ANGLE: {self.display_values['angle_state']}", True, self.WHITE)
        self.screen.blit(angle_text, (angle_rect.x + (angle_rect.width - angle_text.get_width()) // 2,
                                     angle_rect.y + 10))
        
        # === АКТИВНЫЕ КНОПКИ ===
        if self.active_buttons:
            active_text = self.small_font.render("Active: " + ", ".join(sorted(self.active_buttons)), 
                                                True, self.CYAN)
            self.screen.blit(active_text, (20, 570))
        
        # === КАНАЛЫ CRSF ===
        channels = self.get_crsf_channels()
        for key, rect in self.channel_boxes.items():
            channel_num = int(key.split('_')[-1]) - 1
            crsf_value = channels[channel_num]
            us_value = int(crsf_value * 1000 / 2000 + 1000)  # СТАНДАРТНАЯ ФОРМУЛА 1000-2000
            
            # Фон канала
            pygame.draw.rect(self.screen, (50, 50, 60), rect, border_radius=5)
            
            # Заполнение пропорционально значению
            fill_width = int((us_value - 1000) / 1000 * rect.width)
            fill_width = max(0, min(rect.width, fill_width))
            
            # Цвет заполнения
            if us_value < 1300:
                fill_color = (0, 200, 0)  # Зеленый для низких значений
            elif us_value > 1700:
                fill_color = (200, 0, 0)  # Красный для высоких значений
            else:
                fill_color = (200, 200, 0)  # Желтый для средних
            
            pygame.draw.rect(self.screen, fill_color, 
                           (rect.x, rect.y, fill_width, rect.height), border_radius=5)
            
            # Рамка
            pygame.draw.rect(self.screen, self.BLACK, rect, 2, border_radius=5)
            
            # Текст
            label = f"CH {channel_num + 1}"
            if channel_num == 2:  # Throttle
                label = f"THR\n{self.display_values['throttle_pct']}%"
            elif channel_num == 4:  # ARM
                label = f"ARM\n{'ON' if self.aux1_state > 1500 else 'OFF'}"
            elif channel_num == 5:  # ANGLE
                label = f"ANGLE\n{'ON' if self.aux2_state > 1500 else 'OFF'}"
            
            for idx, line in enumerate(label.split('\n')):
                text_surface = self.small_font.render(line, True, self.BLACK)
                self.screen.blit(text_surface, (rect.x + (rect.width - text_surface.get_width()) // 2,
                                              rect.y + 5 + idx * 18))
        
        # === СТИКИ ДЖОЙСТИКА ===
        # Левый стик (YAW и THROTTLE)
        left_stick_x = self.stick_center[0] + (self.x - 128) * 0.5
        left_stick_y = self.stick_center[1] - (self.y - 128) * 0.5  # Инвертируем Y
        
        # Ограничиваем позицию внутри круга
        left_x_pos, left_y_pos = self.limit_position_to_circle(
            left_stick_x, left_stick_y, 
            self.stick_center[0], self.stick_center[1],
            self.stick_radius
        )
        
        # Круг стика
        pygame.draw.circle(self.screen, (70, 70, 70), 
                          [self.stick_center[0], self.stick_center[1]], 
                          self.stick_radius + 2, 0)
        pygame.draw.circle(self.screen, (100, 100, 100), 
                          [self.stick_center[0], self.stick_center[1]], 
                          self.stick_radius, 2)
        
        # Центральные линии
        pygame.draw.line(self.screen, (60, 60, 60), 
                        (self.stick_center[0] - self.stick_radius, self.stick_center[1]),
                        (self.stick_center[0] + self.stick_radius, self.stick_center[1]), 1)
        pygame.draw.line(self.screen, (60, 60, 60), 
                        (self.stick_center[0], self.stick_center[1] - self.stick_radius),
                        (self.stick_center[0], self.stick_center[1] + self.stick_radius), 1)
        
        # Точка стика
        pygame.draw.circle(self.screen, (0, 255, 0), (left_x_pos, left_y_pos), 10)
        pygame.draw.circle(self.screen, (0, 200, 0), (left_x_pos, left_y_pos), 10, 2)
        
        # Подписи левого стика
        left_label = self.font.render(f"Left Stick", True, (0, 255, 0))
        left_values = self.small_font.render(f"X:{self.x} Y:{self.y}", True, (180, 255, 180))
        left_desc = self.small_font.render("A/D=YAW, W/S=THROTTLE", True, (200, 200, 200))
        
        self.screen.blit(left_label, (self.stick_center[0] - 50, self.stick_center[1] - 90))
        self.screen.blit(left_values, (self.stick_center[0] - 40, self.stick_center[1] - 70))
        self.screen.blit(left_desc, (self.stick_center[0] - 85, self.stick_center[1] - 50))
        
        # Правый стик (ROLL и PITCH)
        right_stick_x = self.r_stick_center[0] + (self.rx - 128) * 0.5
        right_stick_y = self.r_stick_center[1] - (self.ry - 128) * 0.5  # Инвертируем Y
        
        # Ограничиваем позицию внутри круга
        right_x_pos, right_y_pos = self.limit_position_to_circle(
            right_stick_x, right_stick_y,
            self.r_stick_center[0], self.r_stick_center[1],
            self.stick_radius
        )
        
        # Круг стика
        pygame.draw.circle(self.screen, (70, 70, 70), 
                          [self.r_stick_center[0], self.r_stick_center[1]], 
                          self.stick_radius + 2, 0)
        pygame.draw.circle(self.screen, (100, 100, 100), 
                          [self.r_stick_center[0], self.r_stick_center[1]], 
                          self.stick_radius, 2)
        
        # Центральные линии
        pygame.draw.line(self.screen, (60, 60, 60), 
                        (self.r_stick_center[0] - self.stick_radius, self.r_stick_center[1]),
                        (self.r_stick_center[0] + self.stick_radius, self.r_stick_center[1]), 1)
        pygame.draw.line(self.screen, (60, 60, 60), 
                        (self.r_stick_center[0], self.r_stick_center[1] - self.stick_radius),
                        (self.r_stick_center[0], self.r_stick_center[1] + self.stick_radius), 1)
        
        # Точка стика
        pygame.draw.circle(self.screen, (255, 50, 50), (right_x_pos, right_y_pos), 10)
        pygame.draw.circle(self.screen, (200, 0, 0), (right_x_pos, right_y_pos), 10, 2)
        
        # Подписи правого стика
        right_label = self.font.render(f"Right Stick", True, (255, 50, 50))
        right_values = self.small_font.render(f"X:{self.rx} Y:{self.ry}", True, (255, 180, 180))
        right_desc = self.small_font.render("J/L=ROLL, I/K=PITCH", True, (200, 200, 200))
        
        self.screen.blit(right_label, (self.r_stick_center[0] - 55, self.r_stick_center[1] - 90))
        self.screen.blit(right_values, (self.r_stick_center[0] - 40, self.r_stick_center[1] - 70))
        self.screen.blit(right_desc, (self.r_stick_center[0] - 80, self.r_stick_center[1] - 50))
        
        # === Throttle индикатор ===
        throttle_rect = pygame.Rect(self.screen_width - 160, 400, 120, 20)
        throttle_fill = pygame.Rect(self.screen_width - 160, 400, 
                                   int((self.y / 255) * 120), 20)
        
        pygame.draw.rect(self.screen, (40, 40, 40), throttle_rect, border_radius=3)
        pygame.draw.rect(self.screen, (200, 200, 0), throttle_fill, border_radius=3)
        pygame.draw.rect(self.screen, self.BLACK, throttle_rect, 2, border_radius=3)
        
        throttle_text = self.font.render(f"Throttle: {self.display_values['throttle_pct']}%", 
                                        True, (200, 200, 0))
        self.screen.blit(throttle_text, (self.screen_width - 155, 370))
        
        # === ЛЕГЕНДА УПРАВЛЕНИЯ ===
        controls_y = 600
        controls = [
            "=== Controls ===",
            "АРХИТЕКТУРА: Трекер → Калман → PID → Евро-фильтр → Джойстик",
            "A/D - YAW (левый стик X) - возвращается в центр",
            "W/S - THROTTLE (левый стик Y) - ФИКСИРУЕТСЯ",
            "J/L - ROLL (правый стик X) - возвращается в центр",
            "I/K - PITCH (правый стик Y) - возвращается в центр",
            "Q - DISARM (выключить двигатели)",
            "E - ARM (включить двигатели)",
            "0 - ANGLE режим вкл/выкл",
            "R - Сброс всех стиков (кроме газа)",
            "X - Сброс газа в нейтраль",
            "T - Вкл/Выкл трекинг",
            "F - Вкл/Выкл евро-фильтр (только для авто)",
            "TAB - Переключить интерактивный режим",
            "M - Переключить MODE (LOCAL/ELRS)",
            "ЛЕВАЯ КНОПКА МЫШИ - Выделить объект для трекинга",
            "ESC - Выход"
        ]
        
        # Найти самую широкую строку
        max_width = 0
        for control in controls:
            text_surface = self.small_font.render(control, True, (255, 255, 255))
            max_width = max(max_width, text_surface.get_width())

        # Позиция для всего блока
        block_x = self.screen_width - max_width - 20

        # Отрисовка
        for i, control in enumerate(controls):
            if i == 0:
                color = self.YELLOW
            elif i == 1:
                color = (0, 255, 0)  # Зеленый для архитектуры
            elif "===" in control:
                color = self.CYAN
            elif "МЫШИ" in control:  # Выделяем управление мышью
                color = (0, 255, 255)
            else:
                color = (200, 200, 255)
                
            control_text = self.small_font.render(control, True, color)
            self.screen.blit(control_text, (block_x, controls_y + i * 18))
    
    # ==================== ОСНОВНОЙ ЦИКЛ ====================
    
    def update(self):
        """Обновление состояния системы"""
        # Обновление кадра камеры
        try:
            if not self.frame_queue.empty():
                frame_data = self.frame_queue.get_nowait()
                img_bytes, width, height = frame_data[:3]
                
                # Преобразование в поверхность Pygame
                img_array = np.frombuffer(img_bytes, np.uint8)
                img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                img = cv2.resize(img, (640, 480))
                img_surface = pygame.surfarray.make_surface(img.swapaxes(0, 1))
                self.camera_surface = img_surface
        except Exception as e:
            pass
        
        # Обновление джойстика
        self.update_joystick_values()
        
        # Обработка интерактивного ввода
        if self.interactive_mode:
            self.handle_interactive_input()
        
        # Обновление отображения
        self.draw_interface()
        
        # Обновление счетчика анимации
        self.selection_animation_counter += self.selection_animation_speed
        
        # Обновление экрана
        pygame.display.flip()
    
    def handle_events(self):
        """Обработка событий Pygame"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
                return False
            
            # Обработка выделения мышью
            if self.handle_mouse_selection(event):
                continue
            
            elif event.type == pygame.KEYDOWN:
                # Переключение режимов
                if event.key == pygame.K_TAB:
                    self.interactive_mode = not self.interactive_mode
                    print(f"Interactive mode: {self.interactive_mode}")
                
                # Переключение LOCAL/ELRS режима
                elif event.key == pygame.K_m:
                    self.mode = "elrs" if self.mode == "local" else "local"
                    self.display_values['mode'] = self.mode.upper()
                    print(f"Mode switched to: {self.mode}")
                
                # Выход
                elif event.key == pygame.K_ESCAPE:
                    self.running = False
                    return False
            
            elif event.type == pygame.MOUSEBUTTONDOWN:
                # Обработка кликов по GUI элементам
                pos = event.pos
                
                # Кнопка LOCAL MODE
                if self.mode_local_rect.collidepoint(pos):
                    self.mode = "local"
                    self.display_values['mode'] = "LOCAL"
                    print("Switched to LOCAL mode")
                
                # Кнопка ELRS MODE
                elif self.mode_elrs_rect.collidepoint(pos):
                    self.mode = "elrs"
                    self.display_values['mode'] = "ELRS"
                    print("Switched to ELRS mode")
                
                # Кнопка интерактивного режима
                elif self.interactive_rect.collidepoint(pos):
                    self.interactive_mode = not self.interactive_mode
                    print(f"Interactive mode: {self.interactive_mode}")
                
                # Кнопка включения/выключения евро-фильтра
                elif self.euro_filter_toggle_rect.collidepoint(pos):
                    self.euro_filter_enabled = not self.euro_filter_enabled
                    if not self.euro_filter_enabled:
                        self.reset_euro_filters()
                    self.display_values['euro_filter'] = "ON" if self.euro_filter_enabled else "OFF"
                    print(f"Euro filter {'enabled' if self.euro_filter_enabled else 'disabled'}")
                
                # Кнопка сброса фильтров
                elif self.reset_filters_rect.collidepoint(pos):
                    self.reset_euro_filters()
                    print("Filters reset")
                
                # Кнопка сохранения конфигурации
                elif self.save_config_rect.collidepoint(pos):
                    self.save_config()
        
        return True
    
    def run(self):
        """Основной цикл программы"""
        clock = pygame.time.Clock()
        
        print("=" * 80)
        print("Starting Combined FPV System with Architecture:")
        print("  Tracker (CV) → Kalman (position) → PID → Euro Filter → Joystick")
        print("=" * 80)
        print("Controls:")
        print("  TAB - Toggle interactive mode")
        print("  M - Switch between LOCAL/ELRS modes")
        print("  F - Toggle Euro filter (automatic mode only)")
        print("  LEFT MOUSE BUTTON - Select object for tracking")
        print("  ESC - Exit")
        print("=" * 80)
        
        while self.running:
            # Обработка событий
            if not self.handle_events():
                break
            
            # Обновление состояния
            self.update()
            
            # Ограничение FPS
            clock.tick(60)
        
        # Очистка
        self.cleanup()
    
    def cleanup(self):
        """Очистка ресурсов"""
        print("Cleaning up...")
        
        # Сохранение конфигурации
        self.save_config()
        
        # Остановка потока трекинга
        if hasattr(self, 'tracking_stop_event'):
            self.tracking_stop_event.set()
        
        # Закрытие камеры
        if hasattr(self, 'camera') and self.camera:
            self.camera.release()
        
        # Закрытие serial порта
        if self.ser and self.ser.is_open:
            self.ser.close()
        
        # Остановка потоков
        self.running = False
        
        # Небольшая задержка для завершения потоков
        time.sleep(0.1)
        
        # Закрытие Pygame
        pygame.quit()

# ============================================================================
# Запуск программы
# ============================================================================
if __name__ == "__main__":
    # Установка переменных окружения для X11
    os.environ["XDG_SESSION_TYPE"] = "xcb"
    os.environ["QT_QPA_PLATFORM"] = "xcb"
    
    # Создание и запуск системы
    system = CombinedFPVSystem()
    system.run()

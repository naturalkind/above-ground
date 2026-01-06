#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Комбинированный код: ELRS трансмиттер + FPV трекинг с эмуляцией джойстика
# Архитектура: Трекер (CV) → Калман (позиция) → PID → Евро-фильтр → Джойстик
# Переведено на Dear PyGui с визуализацией стиков

# https://chat.deepseek.com/share/9y19p81un7ju7s06vj - рекомендации для переносаа данных из предыдущих версий
# https://chat.deepseek.com/share/itd45lanfwehuuqqwm - полан реализации:
#Backend: OpenCV
#IPC: ZMQ
#GUI: Dear PyGui
#┌─────────────────┐    ZMQ (IPC)     ┌─────────────────┐
#│                 │◄────────────────►│                 │
#│   Трекинг       │  video: 60FPS    │     GUI         │
#│   Процесс       │  tracking: 100Hz │   Процесс       │
#│                 │  commands: 10Hz  │                 │
#└─────────────────┘                  └─────────────────┘
#         │                                    │
#         ▼                                    ▼
#┌─────────────────┐                  ┌─────────────────┐
#│                 │                  │                 │
#│   ELRS Serial   │◄────────────────►│   Телеметрия    │
#│   Процесс       │  telemetry: 50Hz │   Dashboard     │
#│                 │                  │                 │
#└─────────────────┘                  └─────────────────┘
#



import sys
import os
import platform
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
import struct
from queue import Queue as ThreadQueue, Empty
import dearpygui.dearpygui as dpg
from threading import Lock

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
# Основной класс комбинированной системы
# ============================================================================

class CombinedFPVSystem:
    def __init__(self):
        # Инициализация Dear PyGui
        dpg.create_context()
        
        # Создание реестра шрифтов с поддержкой кириллицы
        with dpg.font_registry():
            # Добавляем шрифт DejaVu Sans с поддержкой кириллицы
            self.default_font = None
            
            # Пытаемся найти доступные шрифты
            font_paths = [
                # Ubuntu/Debian
                "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                # Arch Linux
                "/usr/share/fonts/TTF/DejaVuSans.ttf",
                # Windows (если работает через WSL)
                "C:/Windows/Fonts/arial.ttf",
                # MacOS
                "/System/Library/Fonts/Arial.ttf",
                # Fallback - попробуем использовать системный
                "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf"
            ]
            
            for font_path in font_paths:
                if os.path.exists(font_path):
                    try:
                        self.default_font = dpg.add_font(font_path, 16)
                        print(f"Loaded font: {font_path}")
                        # Добавляем поддержку кириллицы
                        dpg.add_font_range_hint(dpg.mvFontRangeHint_Cyrillic, parent=self.default_font)
                        dpg.add_font_range_hint(dpg.mvFontRangeHint_Default, parent=self.default_font)
                        break
                    except Exception as e:
                        print(f"Failed to load font {font_path}: {e}")
        
        # Если не нашли шрифт, создаем базовый
        if self.default_font is None:
            print("Warning: No suitable font found, using default")
            self.default_font = dpg.add_font("", 16)
        
        dpg.create_viewport(title='ELRS Transmitter + FPV Tracking System v2.0', width=1800, height=1000)
        dpg.setup_dearpygui()
        
        # Привязываем шрифт глобально
        dpg.bind_font(self.default_font)
        
        # Цвета для отрисовки
        self.WHITE = (255, 255, 255, 255)
        self.BLACK = (0, 0, 0, 255)
        self.GREEN = (0, 255, 0, 255)
        self.RED = (255, 0, 0, 255)
        self.BLUE = (0, 0, 255, 255)
        self.YELLOW = (255, 255, 0, 255)
        self.CYAN = (0, 255, 255, 255)
        self.ORANGE = (255, 165, 0, 255)
        self.GRAY = (100, 100, 100, 255)
        self.DARK_GRAY = (50, 50, 50, 255)
        self.LIGHT_GREEN = (100, 255, 100, 255)
        self.LIGHT_RED = (255, 100, 100, 255)
        
        # Режимы работы
        self.mode = "local"  # "local" или "elrs"
        self.interactive_mode = False
        self.tracking_active = False
        
        # Переменные для выделения мышью
        self.drawing = False
        self.start_x, self.start_y = -1, -1
        self.end_x, self.end_y = -1, -1
        self.current_bbox = None
        self.selection_animation_counter = 0
        self.selection_animation_speed = 0.1
        self.camera_texture_id = None
        
        # Настройки ELRS
        self.selected_serial_port = "Not Connected"
        self.selected_baud_rate = 921600
        self.baud_rates = [921600, 115200, 57600, 9600, 19200, 38400, 400000, 1870000, 3750000, 5250000]
        self.serial_ports = [port.device for port in serial.tools.list_ports.comports()]
        self.ser = None
        self.ser_lock = threading.Lock()
        self.running = True
        
        # Каналы CRSF (16 каналов)
        self.crsf_channels = [1500] * 16
        
        # Параметры PID для трекинга
        self.pid_x = PIDController(0.25, 0.01, 0.002)      # Для YAW
        self.pid_y = PIDController(0.35, 0.001, 0.001)    # Для PITCH
        self.pid_rx = PIDController(0.25, 0.01, 0.002)    # Для ROLL
        
        # Время последнего обновления
        self.last_update = time.time()
        
        # Значения джойстика (0-255)
        self.x = 128      # Левый стик X (YAW/A)
        self.y = 0        # Левый стик Y (THROTTLE/T) - начальное значение 0
        self.rx = 128     # Правый стик X (ROLL/R)
        self.ry = 128     # Правый стик Y (PITCH/E)
        
        # Для газа
        self._throttle_step = 0.5
        self._throttle_target = 0
        self._throttle_current = 0
        
        # Для AUX каналов
        self.aux1_state = 1000  # DISARM по умолчанию
        self.aux2_state = 1000  # ANGLE по умолчанию
        self.zero_pressed = False
        
        # Целевые скорости
        self.target_speed = {'x': 0, 'y': 0, 'rx': 0, 'ry': 0}
        
        # Текущие скорости
        self.current_speed = {'x': 0, 'y': 0, 'rx': 0, 'ry': 0}
        
        # Сглаженные значения
        self.smoothed_values = {'x': 128, 'y': 128, 'rx': 128, 'ry': 128}
        
        # Улучшенная система плавности
        self.interactive_config = {
            'max_speed': 2.0,
            'acceleration': 0.15,
            'deceleration': 0.25,
            'dead_zone': 0.05,
            'smooth_factor': 0.8,
            'response_curve': 1.2
        }
        
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
        
        # Анализ батареи
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
        
        # Профили квадрокоптера
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
            }
        }
        
        self.current_profile = '3inch'
        self.profile_auto_detected = True
        
        # Адаптивное управление
        self.adaptive_control = {
            'learning_enabled': True,
            'throttle_efficiency': 1.0,
            'power_efficiency': 1.0,
            'adaptation_rate': 0.01,
            'throttle_hover_history': deque(maxlen=50),
            'voltage_drop_history': deque(maxlen=50),
        }
        
        # Евро-фильтр
        self.euro_filter_enabled = True
        self.euro_filters = {}
        self.euro_filter_params = {
            'min_cutoff': 1.0,
            'beta': 0.1,
            'd_cutoff': 1.0
        }
        
        # Телеметрия
        self.telemetry_parser = CRSFTelemetryParser()
        self.telemetry_active = False
        self.last_rc_send_time = 0
        self.rc_send_interval = 0.02
        
        # Данные трекинга
        self.tracking_data = {
            "init_tracker": False,
            "error_x": 0,
            "error_y": 0,
            "confidence": 1.0,
            "fps": 0
        }
        self.tracking_data_lock = threading.Lock()
        
        # Очереди для обмена данными
        self.frame_queue = ThreadQueue(maxsize=3)
        self.bbox_queue = ThreadQueue()
        
        # Флаги потоков
        self.tracking_stop_event = threading.Event()
        
        # Трекер
        self.tracker_lib = TrackerLib()
        self.predictor = AdaptiveTargetPredictor(history_size=30, min_samples=10)
        self.current_tracker_bbox = None
        
        # Для FPS расчета
        self.proc_frame_count = 0
        self.proc_start_time = time.time()
        self.proc_fps = 0
        
        # Камера
        self.camera = None
        
        # Клавиши - используем словарь для отслеживания состояния
        self.keys_pressed = {}
        self._t_pressed = False
        self._f_pressed = False
        self._last_interactive_update = time.time()
        
        # Параметры стиков для отрисовки
        self.stick_radius = 80
        self.left_stick_center = (100, 100)
        self.right_stick_center = (100, 100)
        
        # Загрузка конфигурации
        self.load_config()
        
        # Создание GUI
        self.create_gui()
        
        # Настройка обработчиков клавиатуры
        self.setup_keyboard_handlers()
        
        # Запуск потоков
        self.start_serial_thread()
        self.start_tracking_thread()
        
        print("=" * 80)
        print("CombinedFPVSystem инициализирован с архитектурой:")
        print("  Трекер (CV) → Калман (позиция) → PID → Евро-фильтр → Джойстик")
        print("=" * 80)
    
    def setup_keyboard_handlers(self):
        """Настройка обработчиков клавиатуры"""
        # Регистрируем обработчики нажатий клавиш для главного окна
        with dpg.handler_registry():
            # Основные клавиши для левого стика (WASD)
            dpg.add_key_press_handler(key=dpg.mvKey_W, callback=lambda: self.key_press('w'))
            dpg.add_key_release_handler(key=dpg.mvKey_W, callback=lambda: self.key_release('w'))
            dpg.add_key_press_handler(key=dpg.mvKey_S, callback=lambda: self.key_press('s'))
            dpg.add_key_release_handler(key=dpg.mvKey_S, callback=lambda: self.key_release('s'))
            dpg.add_key_press_handler(key=dpg.mvKey_A, callback=lambda: self.key_press('a'))
            dpg.add_key_release_handler(key=dpg.mvKey_A, callback=lambda: self.key_release('a'))
            dpg.add_key_press_handler(key=dpg.mvKey_D, callback=lambda: self.key_press('d'))
            dpg.add_key_release_handler(key=dpg.mvKey_D, callback=lambda: self.key_release('d'))
            
            # Основные клавиши для правого стика (IJKL)
            dpg.add_key_press_handler(key=dpg.mvKey_I, callback=lambda: self.key_press('i'))
            dpg.add_key_release_handler(key=dpg.mvKey_I, callback=lambda: self.key_release('i'))
            dpg.add_key_press_handler(key=dpg.mvKey_K, callback=lambda: self.key_press('k'))
            dpg.add_key_release_handler(key=dpg.mvKey_K, callback=lambda: self.key_release('k'))
            dpg.add_key_press_handler(key=dpg.mvKey_J, callback=lambda: self.key_press('j'))
            dpg.add_key_release_handler(key=dpg.mvKey_J, callback=lambda: self.key_release('j'))
            dpg.add_key_press_handler(key=dpg.mvKey_L, callback=lambda: self.key_press('l'))
            dpg.add_key_release_handler(key=dpg.mvKey_L, callback=lambda: self.key_release('l'))
            
            # Специальные клавиши (Q, E, R, X, 0, T, F)
            dpg.add_key_press_handler(key=dpg.mvKey_Q, callback=lambda: self.key_press('q'))
            dpg.add_key_release_handler(key=dpg.mvKey_Q, callback=lambda: self.key_release('q'))
            dpg.add_key_press_handler(key=dpg.mvKey_E, callback=lambda: self.key_press('e'))
            dpg.add_key_release_handler(key=dpg.mvKey_E, callback=lambda: self.key_release('e'))
            dpg.add_key_press_handler(key=dpg.mvKey_R, callback=lambda: self.key_press('r'))
            dpg.add_key_release_handler(key=dpg.mvKey_R, callback=lambda: self.key_release('r'))
            dpg.add_key_press_handler(key=dpg.mvKey_X, callback=lambda: self.key_press('x'))
            dpg.add_key_release_handler(key=dpg.mvKey_X, callback=lambda: self.key_release('x'))
            
            # Клавиша 0 (ноль)
            dpg.add_key_press_handler(key=dpg.mvKey_0, callback=lambda: self.key_press('0'))
            dpg.add_key_release_handler(key=dpg.mvKey_0, callback=lambda: self.key_release('0'))
            
            # T для трекинга
            dpg.add_key_press_handler(key=dpg.mvKey_T, callback=lambda: self.key_press('t'))
            dpg.add_key_release_handler(key=dpg.mvKey_T, callback=lambda: self.key_release('t'))
            
            # F для евро-фильтра
            dpg.add_key_press_handler(key=dpg.mvKey_F, callback=lambda: self.key_press('f'))
            dpg.add_key_release_handler(key=dpg.mvKey_F, callback=lambda: self.key_release('f'))
            
            # M для переключения режима
            dpg.add_key_press_handler(key=dpg.mvKey_M, callback=lambda: self.key_press('m'))
            dpg.add_key_release_handler(key=dpg.mvKey_M, callback=lambda: self.key_release('m'))
            
            # TAB для интерактивного режима
            dpg.add_key_press_handler(key=dpg.mvKey_Tab, callback=lambda: self.key_press('tab'))
            dpg.add_key_release_handler(key=dpg.mvKey_Tab, callback=lambda: self.key_release('tab'))
    
    def key_press(self, key):
        """Обработка нажатия клавиши"""
        print(f"Key pressed: {key}")
        self.keys_pressed[key] = True
        
        # Немедленная обработка некоторых клавиш
        if key == 't':
            self.tracking_active = not self.tracking_active
            dpg.set_value("tracking_checkbox", self.tracking_active)
            print(f"Tracking toggled: {self.tracking_active}")
        
        elif key == 'f':
            self.euro_filter_enabled = not self.euro_filter_enabled
            dpg.set_value("euro_filter_checkbox", self.euro_filter_enabled)
            if not self.euro_filter_enabled:
                self.reset_euro_filters()
            self.display_values['euro_filter'] = "ON" if self.euro_filter_enabled else "OFF"
            print(f"Euro filter {'enabled' if self.euro_filter_enabled else 'disabled'}")
        
        elif key == 'm':
            self.mode = "elrs" if self.mode == "local" else "local"
            self.switch_mode(self.mode)
        
        elif key == 'tab':
            self.interactive_mode = not self.interactive_mode
            dpg.set_value("interactive_checkbox", self.interactive_mode)
            print(f"Interactive mode toggled: {self.interactive_mode}")
    
    def key_release(self, key):
        """Обработка отпускания клавиши"""
        self.keys_pressed[key] = False
        print(f"Key released: {key}")
    
    # ==================== ФУНКЦИИ ЕВРО-ФИЛЬТРА ====================
    
    def one_euro_filter(self, axis, value, timestamp):
        """One Euro Filter для сглаживания значений джойстика"""
        if axis not in self.euro_filters:
            self.euro_filters[axis] = {
                'prev_raw': value,
                'prev_filtered': value,
                'prev_time': timestamp,
                'prev_derivative': 0.0
            }
            return value
        
        state = self.euro_filters[axis]
        
        dt = timestamp - state['prev_time']
        if dt <= 0:
            return state['prev_filtered']
        
        derivative = (value - state['prev_raw']) / dt
        alpha_d = 1.0 / (1.0 + 1.0 / (self.euro_filter_params['d_cutoff'] * dt))
        smoothed_derivative = alpha_d * derivative + (1.0 - alpha_d) * state['prev_derivative']
        
        cutoff = self.euro_filter_params['min_cutoff'] + self.euro_filter_params['beta'] * abs(smoothed_derivative)
        alpha = 1.0 / (1.0 + 1.0 / (cutoff * dt))
        
        filtered_value = alpha * value + (1.0 - alpha) * state['prev_filtered']
        
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
        """Экспоненциальное сглаживание"""
        return current + (target - current) * factor
    
    def apply_response_curve(self, value, curve_factor):
        """Применение кривой отклика"""
        if abs(value) < 0.001:
            return 0
        sign = 1 if value >= 0 else -1
        normalized = abs(value)
        curved = pow(normalized, curve_factor)
        return sign * curved
    
    def limit_position_to_circle(self, x_pos, y_pos, center_x, center_y, radius):
        """Ограничивает позицию стика внутри круга"""
        dx = x_pos - center_x
        dy = y_pos - center_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance > radius:
            scale = radius / distance
            dx *= scale
            dy *= scale
        
        return int(center_x + dx), int(center_y + dy)
    
    # ==================== СИСТЕМНЫЕ ФУНКЦИИ ====================
    
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
                
                # Загрузка параметров евро-фильтра
                self.euro_filter_enabled = general.getboolean('euro_filter_enabled', True)
                self.euro_filter_params['min_cutoff'] = general.getfloat('euro_min_cutoff', 1.0)
                self.euro_filter_params['beta'] = general.getfloat('euro_beta', 0.1)
        
        self.display_values['mode'] = self.mode.upper()
    
    def save_config(self):
        """Сохранение конфигурации в файл"""
        config = configparser.ConfigParser()
        config['General'] = {
            'mode': self.mode,
            'serial_port': self.selected_serial_port,
            'baud_rate': str(self.selected_baud_rate),
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
        """Поток для отправки CRSF пакетов"""
        while self.running:
            if self.ser and self.ser.is_open and self.mode == "elrs":
                current_time = time.time()
                
                if current_time - self.last_rc_send_time >= self.rc_send_interval:
                    with self.ser_lock:
                        channels = self.get_crsf_channels()
                        packet = channelsCrsfToChannelsPacket(channels)
                        try:
                            self.ser.write(packet)
                            self.last_rc_send_time = current_time
                        except Exception as e:
                            print(f"Serial write error: {e}")
                    
                    time.sleep(0.005)
                else:
                    time.sleep(0.001)
            else:
                time.sleep(0.1)
    
    def serial_read_func(self):
        """Поток для чтения телеметрии CRSF"""
        while self.running:
            if self.ser and self.ser.is_open:
                try:
                    available = self.ser.in_waiting
                    if available > 0:
                        data = self.ser.read(available)
                        if data:
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
        """Запуск потока трекинга"""
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
        
        self.tracking_thread = threading.Thread(
            target=self.tracking_task,
            daemon=True
        )
        self.tracking_thread.start()
        print("Tracking thread started")
    
    def tracking_task(self):
        """Задача трекинга"""
        k_scale = 1.2
        
        # Инициализация камеры
        self.camera = cv2.VideoCapture(0)
        if not self.camera.isOpened():
            print("Error: Could not open camera")
            return
            
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
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
                
                # Преобразование для Dear PyGui
                img_rgb = cv2.cvtColor(_img, cv2.COLOR_BGR2RGB)
                img_resized = cv2.resize(img_rgb, (640, 480))
                img_data = img_resized.flatten().astype(np.float32) / 255.0
                
                # Обновление текстуры
                if self.camera_texture_id is not None:
                    dpg.set_value(self.camera_texture_id, img_data)
                
                time.sleep(0.01)
                
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
    
    def analyze_battery_type(self, voltage, current):
        """Анализ и автоопределение типа батареи и количества ячеек"""
        if not self.battery_analyzer['enabled']:
            return
        
        self.battery_analyzer['voltage_history'].append(voltage)
        self.battery_analyzer['current_history'].append(current)
        
        if len(self.battery_analyzer['voltage_history']) < 20:
            return
        
        max_voltage = max(self.battery_analyzer['voltage_history'])
        avg_current = np.mean(list(self.battery_analyzer['current_history'])) if self.battery_analyzer['current_history'] else 0
        
        cells = 0
        battery_type = 'Unknown'
        
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
        
        elif max_voltage > 3.0:
            voltages = list(self.battery_analyzer['voltage_history'])
            if len(voltages) > 10:
                discharge_rate = (voltages[0] - voltages[-1]) / len(voltages)
                if discharge_rate < 0.01:
                    battery_type = 'Li-Ion'
        
        if max_voltage <= 4.0 and max_voltage >= 3.0:
            if cells == 0:
                cells = round(max_voltage / 3.65)
                if cells > 0:
                    battery_type = 'LiFePO4'
        
        if cells > 0:
            self.battery_analyzer['detected_cells'] = cells
            self.battery_analyzer['battery_type'] = battery_type
            
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
            
            if avg_current > 0:
                time_samples = len(self.battery_analyzer['current_history'])
                avg_current_a = avg_current
                estimated_capacity = avg_current_a * 5 * 60
                self.battery_analyzer['capacity_estimated'] = int(estimated_capacity * 1000)
            
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
        try:
            voltage = float(self.telemetry_parser.telemetry['voltage'])
            current = float(self.telemetry_parser.telemetry['current'])
        except:
            voltage = 0
            current = 0
            
        if voltage > 0:
            self.analyze_battery_type(voltage, current)
        
        if self.battery_analyzer['auto_detection_complete'] and not self.profile_auto_detected:
            self.auto_detect_quadcopter_profile()
        
        with self.tracking_data_lock:
            error_x = self.tracking_data.get("error_x", 0)
            error_y = self.tracking_data.get("error_y", 0)
            fps = self.tracking_data.get("fps", 0)
            confidence = self.tracking_data.get("confidence", 0)
        
        if not self.interactive_mode and self.tracking_active:
            current_time = time.time()
            dt = current_time - self.last_update
            self.last_update = current_time
            
            pid_x = self.pid_x.update(0, error_x, dt)
            pid_y = self.pid_y.update(0, -error_y, dt)
            pid_rx = self.pid_rx.update(0, error_x, dt)
            
            pid_x = max(-20, min(20, pid_x))
            pid_y = max(-100, min(100, pid_y))
            pid_rx = max(-20, min(20, pid_rx))
            
            SCALE = 127 / 100.0
            
            raw_x = 128 + pid_x * SCALE
            raw_rx = 128 + pid_rx * SCALE
            
            fixed_pitch_percent = 0.24  
            SCALE = 127 / 100.0
            raw_pitch = 128 + int(fixed_pitch_percent * 100 * SCALE)
            
            max_pitch_forward = 180
            max_pitch_backward = 80
            self.ry = max(max_pitch_backward, min(max_pitch_forward, raw_pitch))
            
            base_throttle = self.display_values['hover_throttle']
            if voltage > 0:
                throttle_output = self.update_adaptive_throttle_pid(pid_y, voltage, dt)
            else:
                throttle_output = base_throttle + pid_y * self.display_values['pid_scale']
            raw_throttle = max(50, min(220, throttle_output))
            
            if self.euro_filter_enabled and not self.interactive_mode:
                filtered_x = self.one_euro_filter('x_auto', raw_x, current_time)
                filtered_rx = self.one_euro_filter('rx_auto', raw_rx, current_time)
                filtered_throttle = self.one_euro_filter('throttle_auto', raw_throttle, current_time)
                
                self.x = max(0, min(255, int(round(filtered_x))))
                self.rx = max(0, min(255, int(round(filtered_rx))))
                self.y = max(0, min(255, int(round(filtered_throttle))))
                
                self.display_values['filter_type'] = "АВТО (Евро)"
            else:
                smooth_factor = 0.7
                self.x = int(self.x + (raw_x - self.x) * smooth_factor)
                self.rx = int(self.rx + (raw_rx - self.rx) * smooth_factor)
                self.y = int(self.y + (raw_throttle - self.y) * smooth_factor)
                
                self.x = max(0, min(255, self.x))
                self.rx = max(0, min(255, self.rx))
                self.y = max(0, min(255, self.y))
                
                self.display_values['filter_type'] = "АВТО (Простой)"
            
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
    
    def handle_interactive_input(self):
        """Обработка интерактивного ввода"""
        current_time = time.time()
        dt = current_time - self._last_interactive_update
        self._last_interactive_update = current_time
        dt = min(dt, 0.05)
        
        config = self.interactive_config
        
        # Обработка клавиш
        if self.keys_pressed.get('w', False):
            self._throttle_target = min(255, self._throttle_target + self._throttle_step * dt * 60)
        if self.keys_pressed.get('s', False):
            self._throttle_target = max(0, self._throttle_target - self._throttle_step * dt * 60)
        
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
        
        # Основные оси
        target_x_speed = 0
        target_rx_speed = 0
        target_ry_speed = 0
        
        if self.keys_pressed.get('j', False):
            target_rx_speed = -config['max_speed']
        if self.keys_pressed.get('l', False):
            target_rx_speed = config['max_speed']
        
        if self.keys_pressed.get('i', False):
            target_ry_speed = config['max_speed']
        if self.keys_pressed.get('k', False):
            target_ry_speed = -config['max_speed']
        
        if self.keys_pressed.get('a', False):
            target_x_speed = -config['max_speed']
        if self.keys_pressed.get('d', False):
            target_x_speed = config['max_speed']
        
        # Плавное изменение с принудительным возвратом к центру
        axes_to_update = [
            ('x', target_x_speed, 128),
            ('rx', target_rx_speed, 128),
            ('ry', target_ry_speed, 128),
        ]
        
        for axis_name, target_speed, center_value in axes_to_update:
            current_speed = self.current_speed[axis_name]
            
            moving_to_target = abs(target_speed) > 0.001
            moving_to_center = abs(target_speed) < 0.001 and abs(current_speed) > 0.001
            
            if moving_to_target:
                acceleration = config['acceleration']
                final_target = target_speed
            else:
                acceleration = config['deceleration'] * 1.5
                final_target = 0
            
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
            
            curved_speed = self.apply_response_curve(new_speed, config['response_curve'])
            speed_ratio = curved_speed / config['max_speed']
            raw_value = center_value + speed_ratio * 127
            
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
        
        self.display_values['filter_type'] = "РУЧНОЙ (Без фильтра)"
        
        # AUX каналы
        if self.keys_pressed.get('q', False):
            self.aux1_state = 1000
            self.display_values['arm_state'] = "DISARMED"
        if self.keys_pressed.get('e', False):
            self.aux1_state = 2000
            self.display_values['arm_state'] = "ARMED"
        
        if self.keys_pressed.get('0', False) and not self.zero_pressed:
            self.zero_pressed = True
            self.aux2_state = 2000 if self.aux2_state == 1000 else 1000
            self.display_values['angle_state'] = "ON" if self.aux2_state == 2000 else "OFF"
        elif not self.keys_pressed.get('0', False):
            self.zero_pressed = False
        
        self.display_values['arm_state'] = "ARMED" if self.aux1_state == 2000 else "DISARMED"
        self.display_values['angle_state'] = "ON" if self.aux2_state == 2000 else "OFF"
        
        # Специальные клавиши
        if self.keys_pressed.get('r', False):
            self.x = 128
            self.rx = 128
            self.ry = 128
            
            for axis in ['x', 'rx', 'ry']:
                self.current_speed[axis] = 0
                self.smoothed_values[axis] = 128.0
        
        if self.keys_pressed.get('x', False):
            self._throttle_target = 0
        
        if self.keys_pressed.get('t', False) and not self._t_pressed:
            self._t_pressed = True
            self.tracking_active = not self.tracking_active
        elif not self.keys_pressed.get('t', False):
            self._t_pressed = False
        
        if self.keys_pressed.get('f', False) and not self._f_pressed:
            self._f_pressed = True
            self.euro_filter_enabled = not self.euro_filter_enabled
            if not self.euro_filter_enabled:
                self.reset_euro_filters()
            self.display_values['euro_filter'] = "ON" if self.euro_filter_enabled else "OFF"
            print(f"Euro filter {'enabled' if self.euro_filter_enabled else 'disabled'}")
        elif not self.keys_pressed.get('f', False):
            self._f_pressed = False
    
    # ==================== ОБРАБОТКА ВЫДЕЛЕНИЯ МЫШЬЮ ====================

    def handle_mouse_input(self):
        """Обработка ввода мыши с отрисовкой анимированной рамки во время выделения"""
        # Получаем состояние мыши
        mouse_pos = dpg.get_mouse_pos()
        left_mouse_pressed = dpg.is_mouse_button_down(dpg.mvMouseButton_Left)
        
        # Получаем позицию и размер области камеры
        camera_pos = dpg.get_item_pos("camera_drawlist")
        camera_size = dpg.get_item_rect_size("camera_drawlist")
        
        if camera_pos is None or camera_size is None:
            return
        
        # Проверяем, что мышь в области камеры
        mouse_in_camera_area = (
            camera_pos[0] <= mouse_pos[0] <= camera_pos[0] + camera_size[0] and
            camera_pos[1] <= mouse_pos[1] <= camera_pos[1] + camera_size[1]
        )
        
        if left_mouse_pressed and mouse_in_camera_area:
            if not self.drawing:
                # Начало выделения
                self.drawing = True
                self.start_x = int(mouse_pos[0] - camera_pos[0])
                self.start_y = int(mouse_pos[1] - camera_pos[1])
                self.end_x = self.start_x
                self.end_y = self.start_y
                print(f"Начало выделения: ({self.start_x}, {self.start_y})")
            else:
                # Продолжение выделения - обновляем конечную точку
                self.end_x = int(mouse_pos[0] - camera_pos[0])
                self.end_y = int(mouse_pos[1] - camera_pos[1])
        elif not left_mouse_pressed and self.drawing:
            # Завершение выделения (отпустили ЛКМ)
            self.complete_selection()
    
    def complete_selection(self):
        """Завершение выделения и отправка bbox в трекер"""
        self.drawing = False
        
        # Вычисляем координаты прямоугольника
        x1 = min(self.start_x, self.end_x)
        y1 = min(self.start_y, self.end_y)
        x2 = max(self.start_x, self.end_x)
        y2 = max(self.start_y, self.end_y)
        
        width = x2 - x1
        height = y2 - y1
        
        if width > 10 and height > 10:  # Минимальный размер
            # Масштабируем координаты для трекинга
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
                    dpg.set_value("tracking_checkbox", True)
                else:
                    print("Очередь bbox переполнена!")
            except Exception as e:
                print(f"Ошибка отправки bbox: {e}")
        else:
            print("Прямоугольник слишком мал для трекинга")
        
        # Сбрасываем координаты
        self.start_x, self.start_y = -1, -1
        self.end_x, self.end_y = -1, -1
    
    def draw_selection_rectangle(self):
        """Отрисовка анимированной рамки выделения (во время и после выделения)"""
        dpg.delete_item("selection_rect", children_only=False)
        
        if not self.drawing or self.start_x == -1 or self.end_x == -1:
            return
        
        x1 = min(self.start_x, self.end_x)
        y1 = min(self.start_y, self.end_y)
        x2 = max(self.start_x, self.end_x)
        y2 = max(self.start_y, self.end_y)
        
        width = x2 - x1
        height = y2 - y1
        # Рисуем анимированную рамку
        with dpg.draw_node(parent="camera_drawlist", tag="selection_rect"):
            # Эффект пульсации
            pulse = abs(math.sin(self.selection_animation_counter)) * 155 + 100
            color = (0, int(pulse), 0, 255)
            
            # Толщина линии с пульсацией
            line_thickness = max(2, int(3 * (0.5 + abs(math.sin(self.selection_animation_counter * 2)) * 0.5)))
            
            # Внешний прямоугольник
            dpg.draw_rectangle((x1, y1), (x2, y2), 
                              color=color, fill=(0, 100, 0, 30), thickness=line_thickness)
            
            # Угловые элементы (как в оригинальном коде)
            corner_size = 20
            
            # Левый верхний угол
            dpg.draw_line((x1, y1), (x1 + corner_size, y1), color=color, thickness=line_thickness)
            dpg.draw_line((x1, y1), (x1, y1 + corner_size), color=color, thickness=line_thickness)
            
            # Правый верхний угол
            dpg.draw_line((x2, y1), (x2 - corner_size, y1), color=color, thickness=line_thickness)
            dpg.draw_line((x2, y1), (x2, y1 + corner_size), color=color, thickness=line_thickness)
            
            # Правый нижний угол
            dpg.draw_line((x2, y2), (x2 - corner_size, y2), color=color, thickness=line_thickness)
            dpg.draw_line((x2, y2), (x2, y2 - corner_size), color=color, thickness=line_thickness)
            
            # Левый нижний угол
            dpg.draw_line((x1, y2), (x1 + corner_size, y2), color=color, thickness=line_thickness)
            dpg.draw_line((x1, y2), (x1, y2 - corner_size), color=color, thickness=line_thickness)
            
            # Анимированные точки по углам
            dot_size = int(4 * (0.5 + abs(math.sin(self.selection_animation_counter * 2)) * 0.5))
            
            # Рисуем точки в углах
            corners = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
            for cx, cy in corners:
                dpg.draw_circle((cx, cy), dot_size, 
                               color=color, fill=color, thickness=0)
            
            # Текст "Выделение" в центре (если прямоугольник достаточно большой)
            if width > 100 and height > 30:
                # Создаем текст с эффектом пульсации
                text_pulse = abs(math.sin(self.selection_animation_counter * 3)) * 155 + 100
                text_color = (0, int(text_pulse), 0, 255)
                
                # Для текста в DPG нужно использовать draw_text
                text_x = x1 + width / 2
                text_y = y1 + height / 2
                
                dpg.draw_text((text_x - 40, text_y - 10), "Выделение", 
                             color=text_color, size=16)
    
    # ==================== GUI И ВИЗУАЛИЗАЦИЯ СТИКОВ ====================
    
    def create_gui(self):
        """Создание интерфейса Dear PyGui"""
        
        # Создание текстуры для камеры
        with dpg.texture_registry():
            self.camera_texture_id = dpg.add_raw_texture(
                width=640, height=480, default_value=[0.0]*640*480*3, 
                format=dpg.mvFormat_Float_rgb, tag="camera_texture")
        
        # Главное окно
        with dpg.window(label="ELRS Transmitter + FPV Tracking System", tag="main_window", 
                       width=1800, height=1000, no_scrollbar=True):
            
            # Верхняя панель режимов
            with dpg.group(horizontal=True):
                with dpg.child_window(width=400, height=100):
                    with dpg.group(horizontal=True):
                        dpg.add_button(label="LOCAL MODE", width=150, height=40, 
                                     callback=lambda: self.switch_mode("local"),
                                     tag="local_mode_btn")
                        dpg.add_button(label="ELRS MODE", width=150, height=40,
                                     callback=lambda: self.switch_mode("elrs"),
                                     tag="elrs_mode_btn")
                    
                    with dpg.group(horizontal=True):
                        dpg.add_checkbox(label="Интерактивный", default_value=self.interactive_mode,
                                       callback=self.toggle_interactive_mode, tag="interactive_checkbox")
                        dpg.add_checkbox(label="Трекинг", default_value=self.tracking_active,
                                       callback=self.toggle_tracking, tag="tracking_checkbox")
                        dpg.add_checkbox(label="Евро-фильтр", default_value=self.euro_filter_enabled,
                                       callback=self.toggle_euro_filter, tag="euro_filter_checkbox")
                
                with dpg.child_window(width=400, height=100):
                    with dpg.group(horizontal=True):
                        dpg.add_text("Serial:")
                        dpg.add_combo(items=self.serial_ports, default_value=self.selected_serial_port,
                                    width=200, callback=self.select_serial_port, tag="serial_combo")
                        
                        dpg.add_text("Baud:")
                        dpg.add_combo(items=[str(b) for b in self.baud_rates], 
                                    default_value=str(self.selected_baud_rate),
                                    width=120, callback=self.select_baud_rate, tag="baud_combo")
                    
                    dpg.add_text(f"Статус: {self.display_values['serial_status']}", 
                               tag="serial_status_text")
                    
                    with dpg.group(horizontal=True):
                        dpg.add_button(label="Сбросить фильтры", callback=self.reset_euro_filters)
                        dpg.add_button(label="Сохранить конфиг", callback=self.save_config)
                
                with dpg.child_window(width=400, height=100):
                    with dpg.group(horizontal=True):
                        dpg.add_text("ARM:", color=(0, 255, 0) if self.display_values['arm_state'] == "ARMED" else (255, 0, 0))
                        dpg.add_text(self.display_values['arm_state'], tag="arm_state_text")
                        
                        dpg.add_text("ANGLE:", color=(0, 255, 0) if self.display_values['angle_state'] == "ON" else (100, 100, 100))
                        dpg.add_text(self.display_values['angle_state'], tag="angle_state_text")
                    
                    dpg.add_text("Throttle:", color=(200, 200, 0))
                    dpg.add_progress_bar(default_value=self.display_values['throttle_pct']/100, 
                                       tag="throttle_bar", overlay=f"{self.display_values['throttle_pct']}%",
                                       width=300, height=20)
            
            # Разделитель
            dpg.add_separator()
            
            # Основной контент
            with dpg.group(horizontal=True):
                # Левая панель - информация и телеметрия
                with dpg.child_window(width=400, height=700):
                    # Информация о системе
                    with dpg.collapsing_header(label="Информация о системе", default_open=True):
                        dpg.add_text(f"FPS: {self.display_values['fps']:.1f}", tag="fps_text")
                        dpg.add_text(f"Ошибка X: {self.display_values['error_x']:.2f}", tag="error_x_text")
                        dpg.add_text(f"Ошибка Y: {self.display_values['error_y']:.2f}", tag="error_y_text")
                        dpg.add_text(f"PID X: {self.display_values['pid_x']:.2f}", tag="pid_x_text")
                        dpg.add_text(f"PID Y: {self.display_values['pid_y']:.2f}", tag="pid_y_text")
                        
                        dpg.add_text(f"Уверенность:", tag="confidence_label")
                        dpg.add_text(f"{self.display_values['confidence']:.2f}", 
                                   tag="confidence_text")
                        
                        dpg.add_text(f"Трекинг:", tag="tracking_status_label")
                        dpg.add_text(self.display_values['tracking_status'], 
                                   tag="tracking_status_text")
                        
                        dpg.add_text(f"Тип фильтра:", tag="filter_type_label")
                        dpg.add_text(self.display_values['filter_type'], tag="filter_type_text")
                    
                    # Телеметрия
                    with dpg.collapsing_header(label="Телеметрия ELRS", default_open=True):
                        with dpg.group(horizontal=True):
                            dpg.add_text("Напряжение:")
                            dpg.add_text(self.telemetry_parser.telemetry['voltage'], tag="voltage_text")
                        
                        with dpg.group(horizontal=True):
                            dpg.add_text("Ток:")
                            dpg.add_text(self.telemetry_parser.telemetry['current'], tag="current_text")
                        
                        with dpg.group(horizontal=True):
                            dpg.add_text("Остаток:")
                            dpg.add_text(self.telemetry_parser.telemetry['fuel'], tag="fuel_text")
                        
                        with dpg.group(horizontal=True):
                            dpg.add_text("RSSI/LQ:")
                            dpg.add_text(f"{self.telemetry_parser.telemetry['link_stats']['downlink_rssi']}dB / "
                                       f"{self.telemetry_parser.telemetry['link_stats']['downlink_lq']}%", 
                                       tag="rssi_text")
                        
                        with dpg.group(horizontal=True):
                            dpg.add_text("Pitch/Roll/Yaw:")
                            dpg.add_text(f"{self.telemetry_parser.telemetry['attitude']['pitch']:+.1f}° / "
                                       f"{self.telemetry_parser.telemetry['attitude']['roll']:+.1f}° / "
                                       f"{self.telemetry_parser.telemetry['attitude']['yaw']:+.1f}°", 
                                       tag="attitude_text")
                        
                        with dpg.group(horizontal=True):
                            dpg.add_text("Режим полета:")
                            dpg.add_text(self.telemetry_parser.telemetry['flight_mode']['mode'], tag="mode_text")
                    
                    # Каналы CRSF
                    with dpg.collapsing_header(label="Каналы CRSF", default_open=True):
                        with dpg.table(header_row=False, policy=dpg.mvTable_SizingFixedFit):
                            for i in range(4):
                                dpg.add_table_column()
                            
                            for row in range(4):
                                with dpg.table_row():
                                    for col in range(4):
                                        ch_num = row * 4 + col + 1
                                        with dpg.group():
                                            dpg.add_text(f"CH{ch_num}", tag=f"ch_label_{ch_num}")
                                            dpg.add_progress_bar(tag=f"ch_bar_{ch_num}", 
                                                               default_value=0.5, 
                                                               width=80, height=20)
                                            dpg.add_text("1500", tag=f"ch_value_{ch_num}", indent=20)
                
                # Центральная панель - камера
                with dpg.child_window(width=640, height=700):
                    dpg.add_text("Камера FPV", color=self.CYAN)
                    # Область для камеры с возможностью выделения
                    with dpg.drawlist(width=640, height=480, tag="camera_drawlist"):
                        # Изображение с камеры будет нарисовано через текстуру
                        pass
                    
                    # Инструкция по выделению
                    dpg.add_text("ЛКМ - выделить объект для трекинга", color=(0, 255, 255))
                    dpg.add_text("Управление: WASD - левый стик, IJKL - правый стик", color=(200, 200, 200))
                    dpg.add_text("Q/E - ARM/DISARM, 0 - ANGLE, R - сброс, X - сброс газа", color=(200, 200, 200))
                    dpg.add_text("TAB - интерактивный режим, M - LOCAL/ELRS, F - евро-фильтр", color=(200, 200, 200))
                
                # Правая панель - стики и управление
                with dpg.child_window(width=400, height=700):
                    # Визуализация стиков
                    dpg.add_text("Визуализация стиков джойстика", color=self.YELLOW)
                    
                    # Левый стик (YAW/THROTTLE)
                    dpg.add_text("Левый стик (YAW/THROTTLE)", color=(0, 255, 0))
                    with dpg.drawlist(width=200, height=200, tag="left_stick_drawlist"):
                        # Здесь будет отрисовка левого стика
                        pass
                    
                    dpg.add_text(f"X (YAW): {self.x}", tag="left_stick_x_text")
                    dpg.add_text(f"Y (THROTTLE): {self.y}", tag="left_stick_y_text")
                    dpg.add_text("A/D - YAW, W/S - THROTTLE", color=(200, 200, 200))
                    
                    dpg.add_spacer(height=20)
                    
                    # Правый стик (ROLL/PITCH)
                    dpg.add_text("Правый стик (ROLL/PITCH)", color=(255, 50, 50))
                    with dpg.drawlist(width=200, height=200, tag="right_stick_drawlist"):
                        # Здесь будет отрисовка правого стика
                        pass
                    
                    dpg.add_text(f"X (ROLL): {self.rx}", tag="right_stick_x_text")
                    dpg.add_text(f"Y (PITCH): {self.ry}", tag="right_stick_y_text")
                    dpg.add_text("J/L - ROLL, I/K - PITCH", color=(200, 200, 200))
                    
                    dpg.add_spacer(height=20)
                    
                    # Активные клавиши
                    dpg.add_text("Активные клавиши:", color=self.CYAN)
                    dpg.add_text("", tag="active_keys_text")
                    
                    dpg.add_spacer(height=20)
                    
                    # Статус системы
                    dpg.add_text("Статус системы:", color=self.YELLOW)
                    dpg.add_text(f"Режим: {self.display_values['mode']}", tag="mode_text_display")
                    dpg.add_text(f"Интерактивный: {'ВКЛ' if self.interactive_mode else 'ВЫКЛ'}", tag="interactive_status")
                    dpg.add_text(f"Трекинг: {'АКТИВЕН' if self.tracking_active else 'ОСТАНОВЛЕН'}", tag="tracking_status_display")
                    dpg.add_text(f"Евро-фильтр: {self.display_values['euro_filter']}", tag="euro_filter_status")
            
            # Информация об архитектуре
            dpg.add_text("Архитектура: Трекер (CV) → Калман (позиция) → PID → Евро-фильтр → Джойстик", 
                        color=(0, 255, 0))
    
    def draw_joystick_visualization(self):
        """Отрисовка визуализации стиков джойстика"""
        # Очищаем предыдущие рисунки
        dpg.delete_item("left_stick_drawlist", children_only=True)
        dpg.delete_item("right_stick_drawlist", children_only=True)
        
        # Параметры для отрисовки
        center_x, center_y = 100, 100
        radius = 80
        
        # ============ ЛЕВЫЙ СТИК (YAW/THROTTLE) ============
        with dpg.draw_node(parent="left_stick_drawlist"):
            # Фон стика (круг)
            dpg.draw_circle((center_x, center_y), radius, 
                           color=self.DARK_GRAY, fill=self.DARK_GRAY, thickness=2)
            
            # Центральные линии
            dpg.draw_line((center_x - radius, center_y), (center_x + radius, center_y), 
                         color=self.GRAY, thickness=1)
            dpg.draw_line((center_x, center_y - radius), (center_x, center_y + radius), 
                         color=self.GRAY, thickness=1)
            
            # Позиция стика (преобразуем значения 0-255 в координаты -radius до +radius)
            stick_x = center_x + (self.x - 128) * radius / 128
            stick_y = center_y - (self.y - 128) * radius / 128  # Инвертируем Y для интуитивного отображения
            
            # Ограничиваем позицию внутри круга
            dx = stick_x - center_x
            dy = stick_y - center_y
            distance = math.sqrt(dx*dx + dy*dy)
            if distance > radius:
                scale = radius / distance
                dx *= scale
                dy *= scale
                stick_x = center_x + dx
                stick_y = center_y + dy
            
            # Точка стика
            dpg.draw_circle((stick_x, stick_y), 10, 
                           color=self.GREEN, fill=self.LIGHT_GREEN, thickness=2)
            
            # Линия от центра к точке
            dpg.draw_line((center_x, center_y), (stick_x, stick_y), 
                         color=self.GREEN, thickness=2)
            
            # Индикатор оси Y (Throttle) справа
            throttle_height = (self.y / 255.0) * (radius * 2)
            throttle_y = center_y + radius - throttle_height
            dpg.draw_rectangle((center_x + radius + 10, throttle_y), 
                             (center_x + radius + 20, center_y + radius),
                             color=self.GREEN, fill=self.GREEN, rounding=2)
            
            # Подписи
            dpg.draw_text((center_x - 15, center_y - radius - 20), "Throttle", color=self.GREEN)
            dpg.draw_text((center_x + radius + 25, center_y - radius - 20), "Yaw", color=self.GREEN)
        
        # ============ ПРАВЫЙ СТИк (ROLL/PITCH) ============
        with dpg.draw_node(parent="right_stick_drawlist"):
            # Фон стика (круг)
            dpg.draw_circle((center_x, center_y), radius, 
                           color=self.DARK_GRAY, fill=self.DARK_GRAY, thickness=2)
            
            # Центральные линии
            dpg.draw_line((center_x - radius, center_y), (center_x + radius, center_y), 
                         color=self.GRAY, thickness=1)
            dpg.draw_line((center_x, center_y - radius), (center_x, center_y + radius), 
                         color=self.GRAY, thickness=1)
            
            # Позиция стика
            stick_x = center_x + (self.rx - 128) * radius / 128
            stick_y = center_y - (self.ry - 128) * radius / 128  # Инвертируем Y
            
            # Ограничиваем позицию внутри круга
            dx = stick_x - center_x
            dy = stick_y - center_y
            distance = math.sqrt(dx*dx + dy*dy)
            if distance > radius:
                scale = radius / distance
                dx *= scale
                dy *= scale
                stick_x = center_x + dx
                stick_y = center_y + dy
            
            # Точка стика
            dpg.draw_circle((stick_x, stick_y), 10, 
                           color=self.RED, fill=self.LIGHT_RED, thickness=2)
            
            # Линия от центра к точки
            dpg.draw_line((center_x, center_y), (stick_x, stick_y), 
                         color=self.RED, thickness=2)
            
            # Индикатор оси Y (Pitch) справа
            pitch_height = (self.ry / 255.0) * (radius * 2)
            pitch_y = center_y + radius - pitch_height
            dpg.draw_rectangle((center_x + radius + 10, pitch_y), 
                             (center_x + radius + 20, center_y + radius),
                             color=self.RED, fill=self.RED, rounding=2)
            
            # Подписи
            dpg.draw_text((center_x - 15, center_y - radius - 20), "Pitch", color=self.RED)
            dpg.draw_text((center_x + radius + 25, center_y - radius - 20), "Roll", color=self.RED)
    
    # ==================== ОБРАБОТЧИКИ GUI ====================
    
    def switch_mode(self, mode):
        """Переключение режима LOCAL/ELRS"""
        self.mode = mode
        self.display_values['mode'] = self.mode.upper()
        
        # Обновление цвета кнопок
        if mode == "local":
            dpg.configure_item("local_mode_btn", color=(0, 255, 0, 255))
            dpg.configure_item("elrs_mode_btn", color=(100, 100, 100, 255))
        else:
            dpg.configure_item("local_mode_btn", color=(100, 100, 100, 255))
            dpg.configure_item("elrs_mode_btn", color=(0, 255, 0, 255))
        
        print(f"Mode switched to: {self.mode}")
    
    def toggle_interactive_mode(self, sender, app_data):
        """Включение/выключение интерактивного режима"""
        self.interactive_mode = app_data
        print(f"Interactive mode: {self.interactive_mode}")
    
    def toggle_tracking(self, sender, app_data):
        """Включение/выключение трекинга"""
        self.tracking_active = app_data
        print(f"Tracking: {self.tracking_active}")
    
    def toggle_euro_filter(self, sender, app_data):
        """Включение/выключение евро-фильтра"""
        self.euro_filter_enabled = app_data
        if not self.euro_filter_enabled:
            self.reset_euro_filters()
        self.display_values['euro_filter'] = "ON" if self.euro_filter_enabled else "OFF"
        print(f"Euro filter {'enabled' if self.euro_filter_enabled else 'disabled'}")
    
    def select_serial_port(self, sender, app_data):
        """Выбор serial порта"""
        self.selected_serial_port = app_data
        print(f"Selected serial port: {self.selected_serial_port}")
    
    def select_baud_rate(self, sender, app_data):
        """Выбор скорости передачи"""
        self.selected_baud_rate = int(app_data)
        print(f"Selected baud rate: {self.selected_baud_rate}")
    
    # ==================== ОБНОВЛЕНИЕ GUI ====================
    
    def update_gui(self):
        """Обновление элементов GUI"""
        # Обновление значений джойстика
        self.update_joystick_values()
        
        # Обработка интерактивного ввода
        if self.interactive_mode:
            self.handle_interactive_input()
        
        # Обработка ввода мыши
        self.handle_mouse_input()
        
        # Отрисовка визуализации стиков
        self.draw_joystick_visualization()
        
        # Обновление счетчика анимации
        self.selection_animation_counter += self.selection_animation_speed
        
        # Отрисовка прямоугольника выделения (во время выделения!)
        self.draw_selection_rectangle()
        
        # Отрисовка изображения камеры
        self.draw_camera_image()
        
        # Обновление информации о системе
        self.update_system_info()
        
        # Обновление телеметрии
        self.update_telemetry_display()
        
        # Обновление каналов CRSF
        self.update_crsf_channels()
        
        # Обновление активных клавиш
        self.update_active_keys()
        
        # Обновление статусов
        self.update_status_display()
    
    def draw_camera_image(self):
        """Отрисовка изображения с камеры"""
        dpg.delete_item("camera_image", children_only=False)
        with dpg.draw_node(parent="camera_drawlist", tag="camera_image"):
            if self.camera_texture_id is not None:
                dpg.draw_image("camera_texture", (0, 0), (640, 480))
                # Если есть выделение, рисуем его поверх изображения
                if self.drawing and self.start_x != -1 and self.end_x != -1:
                    self.draw_selection_rectangle()
                    
    def update_system_info(self):
        """Обновление информации о системе"""
        dpg.set_value("fps_text", f"FPS: {self.display_values['fps']:.1f}")
        dpg.set_value("error_x_text", f"Ошибка X: {self.display_values['error_x']:.2f}")
        dpg.set_value("error_y_text", f"Ошибка Y: {self.display_values['error_y']:.2f}")
        dpg.set_value("pid_x_text", f"PID X: {self.display_values['pid_x']:.2f}")
        dpg.set_value("pid_y_text", f"PID Y: {self.display_values['pid_y']:.2f}")
        dpg.set_value("confidence_text", f"{self.display_values['confidence']:.2f}")
        dpg.set_value("tracking_status_text", self.display_values['tracking_status'])
        dpg.set_value("filter_type_text", self.display_values['filter_type'])
        
        # Обновление значений стиков
        dpg.set_value("left_stick_x_text", f"X (YAW): {self.x}")
        dpg.set_value("left_stick_y_text", f"Y (THROTTLE): {self.y}")
        dpg.set_value("right_stick_x_text", f"X (ROLL): {self.rx}")
        dpg.set_value("right_stick_y_text", f"Y (PITCH): {self.ry}")
    
    def update_telemetry_display(self):
        """Обновление отображения телеметрии"""
        current_time = time.time()
        has_recent_telemetry = current_time - self.telemetry_parser.telemetry['last_update'] < 3
        
        if has_recent_telemetry:
            dpg.set_value("voltage_text", self.telemetry_parser.telemetry['voltage'])
            dpg.set_value("current_text", self.telemetry_parser.telemetry['current'])
            dpg.set_value("fuel_text", self.telemetry_parser.telemetry['fuel'])
            
            link = self.telemetry_parser.telemetry['link_stats']
            dpg.set_value("rssi_text", f"{link['downlink_rssi']}dB / {link['downlink_lq']}%")
            
            att = self.telemetry_parser.telemetry['attitude']
            dpg.set_value("attitude_text", f"{att['pitch']:+.1f}° / {att['roll']:+.1f}° / {att['yaw']:+.1f}°")
            
            mode = self.telemetry_parser.telemetry['flight_mode']
            dpg.set_value("mode_text", mode['mode'])
        else:
            dpg.set_value("voltage_text", "N/A")
            dpg.set_value("current_text", "N/A")
            dpg.set_value("fuel_text", "N/A")
            dpg.set_value("rssi_text", "N/A")
            dpg.set_value("attitude_text", "N/A")
            dpg.set_value("mode_text", "N/A")
    
    def update_crsf_channels(self):
        """Обновление отображения каналов CRSF"""
        channels = self.get_crsf_channels()
        for i in range(16):
            us_value = int(channels[i] * 1000 / 2000 + 1000)
            normalized = (us_value - 1000) / 1000.0
            dpg.set_value(f"ch_bar_{i+1}", normalized)
            dpg.set_value(f"ch_value_{i+1}", str(us_value))
    
    def update_active_keys(self):
        """Обновление отображения активных клавиш"""
        active_keys_list = []
        for key, pressed in self.keys_pressed.items():
            if pressed:
                active_keys_list.append(key.upper())
        
        if active_keys_list:
            active_keys = ", ".join(sorted(active_keys_list))
            dpg.set_value("active_keys_text", active_keys)
        else:
            dpg.set_value("active_keys_text", "Нет активных клавиш")
    
    def update_status_display(self):
        """Обновление статусов системы"""
        # Обновление ARM/ANGLE
        dpg.set_value("arm_state_text", self.display_values['arm_state'])
        dpg.set_value("angle_state_text", self.display_values['angle_state'])
        
        # Обновление throttle
        dpg.set_value("throttle_bar", self.display_values['throttle_pct']/100)
        dpg.configure_item("throttle_bar", overlay=f"{self.display_values['throttle_pct']}%")
        
        # Обновление статуса serial
        dpg.set_value("serial_status_text", f"Статус: {self.display_values['serial_status']}")
        
        # Обновление статусов
        dpg.set_value("mode_text_display", f"Режим: {self.display_values['mode']}")
        dpg.set_value("interactive_status", f"Интерактивный: {'ВКЛ' if self.interactive_mode else 'ВЫКЛ'}")
        dpg.set_value("tracking_status_display", f"Трекинг: {'АКТИВЕН' if self.tracking_active else 'ОСТАНОВЛЕН'}")
        dpg.set_value("euro_filter_status", f"Евро-фильтр: {self.display_values['euro_filter']}")
    
    # ==================== ОСНОВНОЙ ЦИКЛ ====================
    
    def run(self):
        """Основной цикл программы"""
        dpg.show_viewport()
        dpg.set_primary_window("main_window", True)
        
        print("=" * 80)
        print("Starting Combined FPV System with Dear PyGui")
        print("  Tracker (CV) → Kalman (position) → PID → Euro Filter → Joystick")
        print("=" * 80)
        print("Controls:")
        print("  WASD - Левый стик (YAW/THROTTLE)")
        print("  IJKL - Правый стик (ROLL/PITCH)")
        print("  Q/E - ARM/DISARM")
        print("  0 - ANGLE режим")
        print("  R - Сброс стиков")
        print("  X - Сброс газа")
        print("  TAB - Интерактивный режим (через GUI)")
        print("  M - LOCAL/ELRS режим (через GUI)")
        print("  F - Евро-фильтр (через GUI)")
        print("  ЛКМ - Выделить объект для трекинга")
        print("  ESC - Выход")
        print("=" * 80)
        
        # Главный цикл Dear PyGui
        while dpg.is_dearpygui_running() and self.running:
            self.update_gui()
            dpg.render_dearpygui_frame()
            time.sleep(0.01)  # Небольшая пауза для снижения нагрузки CPU
        
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
        
        # Закрытие Dear PyGui
        dpg.destroy_context()

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

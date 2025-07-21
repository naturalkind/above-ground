#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
FPV Джойстик Эмулятор с визуализацией
=====================================
"""

import uinput
import time
import math
import threading
import signal
import sys
import pygame

# Определение событий джойстика
JOYSTICK_EVENTS = [
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
    uinput.BTN_BASE3,
    uinput.BTN_BASE4,
    uinput.BTN_BASE5,
    uinput.BTN_BASE6,
]

class FPVJoystickEmulator:
    def __init__(self):
        print("Инициализация эмулятора джойстика FPV...")
        self.device = uinput.Device(JOYSTICK_EVENTS, name="FPV Joystick Emulator")
        print("Виртуальный джойстик создан")
        
        # Начальные значения
        self.x = 128
        self.y = 128
        self.rx = 128
        self.ry = 128
        self.throttle = 0
        
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
        
        # Флаг работы программы
        self.running = True
        
        # Инициализация Pygame
        self.init_pygame()
        
        # Обработка сигналов
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def init_pygame(self):
        """Инициализация графического интерфейса"""
        pygame.init()
        self.screen = pygame.display.set_mode((800, 600))
        pygame.display.set_caption("FPV Joystick Visualizer")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 24)
        
        # Позиции элементов
        self.stick_center = (200, 300)
        self.stick_radius = 50
        self.r_stick_center = (600, 300)
        self.button_positions = {
            'BTN_JOYSTICK': (50, 50),
            'BTN_TRIGGER': (50, 100),
            'BTN_THUMB': (50, 150),
            'BTN_THUMB2': (50, 200),
            'BTN_TOP': (50, 250),
            'BTN_TOP2': (50, 300),
            'BTN_PINKIE': (50, 350),
            'BTN_BASE': (50, 400),
        }

    def update_visualization(self):
        """Обновление графического интерфейса"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

        self.screen.fill((30, 30, 30))
        
        # Левый стик (X/Y)
        x_pos = self.stick_center[0] + (self.x - 128) * 0.5
        y_pos = self.stick_center[1] + (self.y - 128) * 0.5
        pygame.draw.circle(self.screen, (100, 100, 100), self.stick_center, self.stick_radius, 2)
        pygame.draw.circle(self.screen, (0, 255, 0), (int(x_pos), int(y_pos)), 10)

        # Правый стик (RX/RY)
        rx_pos = self.r_stick_center[0] + (self.rx - 128) * 0.5
        ry_pos = self.r_stick_center[1] + (self.ry - 128) * 0.5
        pygame.draw.circle(self.screen, (100, 100, 100), self.r_stick_center, self.stick_radius, 2)
        pygame.draw.circle(self.screen, (255, 0, 0), (int(rx_pos), int(ry_pos)), 10)

        # Газ
        throttle_height = self.throttle / 255 * 200
        pygame.draw.rect(self.screen, (200, 200, 0), (700, 400 - throttle_height, 40, throttle_height))

        # Кнопки
        for btn, pos in self.button_positions.items():
            color = (0, 255, 0) if self.button_states[btn] else (100, 100, 100)
            pygame.draw.circle(self.screen, color, pos, 15)
            text = self.font.render(btn.split('_')[-1], True, (255, 255, 255))
            self.screen.blit(text, (pos[0] - 10, pos[1] - 10))

        pygame.display.flip()
        self.clock.tick(60)

    def signal_handler(self, sig, frame):
        """Обработчик сигнала завершения"""
        print("\nЗавершение работы...")
        self.running = False
        time.sleep(0.5)
        pygame.quit()
        sys.exit(0)

    def emit_events(self):
        """Отправка событий и обновление визуализации"""
        self.device.emit(uinput.ABS_X, self.x)
        self.device.emit(uinput.ABS_Y, self.y)
        self.device.emit(uinput.ABS_RX, self.rx)
        self.device.emit(uinput.ABS_RY, self.ry)
        self.device.emit(uinput.ABS_THROTTLE, self.throttle)
        self.update_visualization()

    def demo_animation(self):
        """Демонстрационная анимация"""
        print("Демо-режим. Нажмите Ctrl+C или закройте окно для выхода")
        angle = 0
        
        while self.running:
            # Обновление позиций
            self.x = int(128 + 50 * math.cos(angle))
            self.y = int(128 + 50 * math.sin(angle))
            self.rx = int(128 + 50 * math.cos(angle + math.pi))
            self.ry = int(128 + 50 * math.sin(angle + math.pi))
            self.throttle = int(128 + 127 * math.sin(angle / 2))
            
            # Обновление кнопок
            button_state = int(angle / (math.pi/8)) % 8
            self.button_states = {
                'BTN_JOYSTICK': button_state == 0,
                'BTN_TRIGGER': button_state == 1,
                'BTN_THUMB': button_state == 2,
                'BTN_THUMB2': button_state == 3,
                'BTN_TOP': button_state == 4,
                'BTN_TOP2': button_state == 5,
                'BTN_PINKIE': button_state == 6,
                'BTN_BASE': button_state == 7,
            }
            
            self.emit_events()
            angle += 0.05
            time.sleep(0.02)

    def interactive_mode(self):
        """Интерактивное управление"""
        print("Интерактивный режим. Используйте WASD/IJKL/QE/1-9")
        import termios, tty
        
        def getch():
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                return sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
        
        while self.running:
            key = getch()
            
            if key == 'x': break
            elif key == 'r': 
                self.x = self.y = self.rx = self.ry = 128
                self.throttle = 0
            elif key == 'w': self.y = max(0, self.y-10)
            elif key == 's': self.y = min(255, self.y+10)
            elif key == 'a': self.x = max(0, self.x-10)
            elif key == 'd': self.x = min(255, self.x+10)
            elif key == 'i': self.ry = max(0, self.ry-10)
            elif key == 'k': self.ry = min(255, self.ry+10)
            elif key == 'j': self.rx = max(0, self.rx-10)
            elif key == 'l': self.rx = min(255, self.rx+10)
            elif key == 'q': self.throttle = max(0, self.throttle-10)
            elif key == 'e': self.throttle = min(255, self.throttle+10)
            elif key in '123456789':
                btn_map = {
                    '1': 'JOYSTICK', '2': 'TRIGGER', '3': 'THUMB',
                    '4': 'THUMB2', '5': 'TOP', '6': 'TOP2',
                    '7': 'PINKIE', '8': 'BASE', '9': 'BASE2'
                }
                btn = f'BTN_{btn_map[key]}'
                self.button_states[btn] = True
                self.emit_events()
                time.sleep(0.1)
                self.button_states[btn] = False
            
            self.emit_events()

    def run(self):
        """Запуск программы"""
        print("\nРежимы работы:\n1. Демо\n2. Интерактивный")
        choice = input("Выберите (1/2): ")
        
        if choice == "1": self.demo_animation()
        else: self.interactive_mode()
        
        pygame.quit()

if __name__ == "__main__":
    try:
        emulator = FPVJoystickEmulator()
        emulator.run()
    except Exception as e:
        print(f"Ошибка: {e}")
        print("Убедитесь что:\n- Установлен python-uinput\n- Запуск с sudo\n- Установлен pygame")

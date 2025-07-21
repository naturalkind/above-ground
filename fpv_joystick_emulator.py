#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
FPV Джойстик Эмулятор
=====================
Эта программа создает виртуальный джойстик в Ubuntu с использованием uinput.
Требуется установка python-uinput и соответствующие права доступа.

Установка зависимостей:
sudo apt-get install python3-pip
sudo pip3 install python-uinput

Запуск:
sudo python3 fpv_joystick_emulator.py
(права sudo необходимы для доступа к /dev/uinput)
"""

import uinput
import time
import math
import threading
import signal
import sys

# Определение событий джойстика
JOYSTICK_EVENTS = [
    uinput.ABS_X + (0, 255, 0, 0),       # X ось (левая/правая)
    uinput.ABS_Y + (0, 255, 0, 0),       # Y ось (вперёд/назад)
    uinput.ABS_RX + (0, 255, 0, 0),      # RX ось (крен)
    uinput.ABS_RY + (0, 255, 0, 0),      # RY ось (тангаж)
    uinput.ABS_THROTTLE + (0, 255, 0, 0),# Throttle (газ)
    uinput.BTN_JOYSTICK,                 # Основная кнопка джойстика
    uinput.BTN_TRIGGER,                  # Кнопка триггера
    uinput.BTN_THUMB,                    # Кнопка большого пальца
    uinput.BTN_THUMB2,                   # Вторая кнопка большого пальца
    uinput.BTN_TOP,                      # Верхняя кнопка
    uinput.BTN_TOP2,                     # Вторая верхняя кнопка
    uinput.BTN_PINKIE,                   # Кнопка мизинца
    uinput.BTN_BASE,                     # Кнопка базы
    uinput.BTN_BASE2,                    # Вторая кнопка базы
    uinput.BTN_BASE3,                    # Третья кнопка базы
    uinput.BTN_BASE4,                    # Четвертая кнопка базы
    uinput.BTN_BASE5,                    # Пятая кнопка базы
    uinput.BTN_BASE6,                    # Шестая кнопка базы
]

class FPVJoystickEmulator:
    def __init__(self):
        print("Инициализация эмулятора джойстика FPV...")
        self.device = uinput.Device(JOYSTICK_EVENTS, name="FPV Joystick Emulator")
        print("Виртуальный джойстик создан")
        
        # Начальные значения
        self.x = 128
        self.y = 128
        self.rx = 0
        self.ry = 128
        self.throttle = 0
        
        # Флаг для демонстрационной анимации
        self.running = True
        
        # Обработка завершения программы
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def signal_handler(self, sig, frame):
        """Обработчик сигнала завершения (Ctrl+C)"""
        print("\nЗавершение работы эмулятора...")
        self.running = False
        time.sleep(0.5)
        sys.exit(0)
        
    def emit_events(self):
        """Отправка событий устройства"""
        #print (dir(uinput))
        self.device.emit(uinput.ABS_X, self.x)
        self.device.emit(uinput.ABS_Y, self.y)
        self.device.emit(uinput.ABS_RX, self.rx)
        self.device.emit(uinput.ABS_RY, self.ry)
        self.device.emit(uinput.ABS_THROTTLE, self.throttle)
        
    def demo_animation(self):
        """Демонстрационная анимация джойстика"""
        print("Запуск демонстрационной анимации джойстика...")
        print("Нажмите Ctrl+C для выхода")
        
        angle = 0
        radius = 50
        center = 128
        
        while self.running:
            # Анимация кругового движения для осей X и Y
            self.x = int(center + radius * math.cos(angle))
            self.y = int(center + radius * math.sin(angle))
            
            # Анимация для осей RX и RY (в противофазе)
            self.rx = int(center + radius * math.cos(angle + math.pi))
            self.ry = int(center + radius * math.sin(angle + math.pi))
            
            # Анимация для газа (throttle)
            self.throttle = int(128 + 127 * math.sin(angle / 2))
            
            # Отправка событий
            self.emit_events()
            
            # Нажатие и отпускание разных кнопок
            button_state = int(angle / (math.pi/8)) % 8
            
            if button_state == 0:
                self.device.emit(uinput.BTN_JOYSTICK, 1)
            elif button_state == 1:
                self.device.emit(uinput.BTN_JOYSTICK, 0)
                self.device.emit(uinput.BTN_TRIGGER, 1)
            elif button_state == 2:
                self.device.emit(uinput.BTN_TRIGGER, 0)
                self.device.emit(uinput.BTN_THUMB, 1)
            elif button_state == 3:
                self.device.emit(uinput.BTN_THUMB, 0)
                self.device.emit(uinput.BTN_THUMB2, 1)
            elif button_state == 4:
                self.device.emit(uinput.BTN_THUMB2, 0)
                self.device.emit(uinput.BTN_TOP, 1)
            elif button_state == 5:
                self.device.emit(uinput.BTN_TOP, 0)
                self.device.emit(uinput.BTN_TOP2, 1)
            elif button_state == 6:
                self.device.emit(uinput.BTN_TOP2, 0)
                self.device.emit(uinput.BTN_PINKIE, 1)
            elif button_state == 7:
                self.device.emit(uinput.BTN_PINKIE, 0)
            
            # Увеличение угла для анимации
            angle += 0.05
            if angle > 2 * math.pi:
                angle -= 2 * math.pi
                
            time.sleep(0.02)  # 50 Гц (обычная частота обновления для джойстиков)
    
    def interactive_mode(self):
        """Интерактивный режим для ручного управления джойстиком"""
        print("\nИнтерактивный режим управления джойстиком")
        print("Команды:")
        print("  wasd - управление осями X/Y")
        print("  ijkl - управление осями RX/RY")
        print("  q/e - уменьшить/увеличить газ (throttle)")
        print("  1-9 - нажать соответствующую кнопку")
        print("  r - сбросить все значения")
        print("  h - показать эту справку")
        print("  x - выход")
        
        import termios
        import tty
        import sys
        
        def getch():
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(fd)
                ch = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch
        
        while self.running:
            key = getch()
            
            # Выход
            if key == 'x':
                break
                
            # Помощь
            elif key == 'h':
                print("\nКоманды:")
                print("  wasd - управление осями X/Y")
                print("  ijkl - управление осями RX/RY")
                print("  q/e - уменьшить/увеличить газ (throttle)")
                print("  1-9 - нажать соответствующую кнопку")
                print("  r - сбросить все значения")
                print("  h - показать эту справку")
                print("  x - выход")
                
            # Сброс значений
            elif key == 'r':
                self.x = 128
                self.y = 128
                self.rx = 0
                self.ry = 128
                self.throttle = 0
                print("\rЗначения сброшены           ")
                
            # Управление осями X/Y
            elif key == 'w' and self.y > 0:
                self.y -= 10
                print(f"\rY: {self.y}     ")
            elif key == 's' and self.y < 255:
                self.y += 10
                print(f"\rY: {self.y}     ")
            elif key == 'a' and self.x > 0:
                self.x -= 10
                print(f"\rX: {self.x}     ")
            elif key == 'd' and self.x < 255:
                self.x += 10
                print(f"\rX: {self.x}     ")
                
            # Управление осями RX/RY
            elif key == 'i' and self.ry > 0:
                self.ry -= 10
                print(f"\rRY: {self.ry}     ")
            elif key == 'k' and self.ry < 255:
                self.ry += 10
                print(f"\rRY: {self.ry}     ")
            elif key == 'j' and self.rx > 0:
                self.rx -= 10
                print(f"\rRX: {self.rx}     ")
            elif key == 'l' and self.rx < 255:
                self.rx += 10
                print(f"\rRX: {self.rx}     ")
                
            # Управление газом
            elif key == 'q' and self.throttle > 0:
                self.throttle -= 10
                print(f"\rThrottle: {self.throttle}     ")
            elif key == 'e' and self.throttle < 255:
                self.throttle += 10
                print(f"\rThrottle: {self.throttle}     ")
                
            # Нажатие кнопок
            elif key >= '1' and key <= '9':
                button_num = int(key)
                if button_num == 1:
                    self.device.emit(uinput.BTN_JOYSTICK, 1)
                    time.sleep(0.1)
                    self.device.emit(uinput.BTN_JOYSTICK, 0)
                elif button_num == 2:
                    self.device.emit(uinput.BTN_TRIGGER, 1)
                    time.sleep(0.1)
                    self.device.emit(uinput.BTN_TRIGGER, 0)
                elif button_num == 3:
                    self.device.emit(uinput.BTN_THUMB, 1)
                    time.sleep(0.1)
                    self.device.emit(uinput.BTN_THUMB, 0)
                elif button_num == 4:
                    self.device.emit(uinput.BTN_THUMB2, 1)
                    time.sleep(0.1)
                    self.device.emit(uinput.BTN_THUMB2, 0)
                elif button_num == 5:
                    self.device.emit(uinput.BTN_TOP, 1)
                    time.sleep(0.1)
                    self.device.emit(uinput.BTN_TOP, 0)
                elif button_num == 6:
                    self.device.emit(uinput.BTN_TOP2, 1)
                    time.sleep(0.1)
                    self.device.emit(uinput.BTN_TOP2, 0)
                elif button_num == 7:
                    self.device.emit(uinput.BTN_PINKIE, 1)
                    time.sleep(0.1)
                    self.device.emit(uinput.BTN_PINKIE, 0)
                elif button_num == 8:
                    self.device.emit(uinput.BTN_BASE, 1)
                    time.sleep(0.1)
                    self.device.emit(uinput.BTN_BASE, 0)
                elif button_num == 9:
                    self.device.emit(uinput.BTN_BASE2, 1)
                    time.sleep(0.1)
                    self.device.emit(uinput.BTN_BASE2, 0)
                print(f"\rНажата кнопка {button_num}     ")
            
            # Отправляем текущие значения
            self.emit_events()
    
    def run(self):
        """Запуск эмулятора с выбором режима"""
        print("\nВыберите режим работы:")
        print("1. Демонстрационная анимация (автоматическое движение)")
        print("2. Интерактивный режим (ручное управление)")
        choice = input("Выберите режим (1/2): ")
        
        if choice == "1":
            self.demo_animation()
        elif choice == "2":
            self.interactive_mode()
        else:
            print("Некорректный выбор. Запуск демонстрационной анимации...")
            self.demo_animation()

if __name__ == "__main__":
    try:
        emulator = FPVJoystickEmulator()
        emulator.run()
    except KeyboardInterrupt:
        print("\nПрограмма остановлена пользователем")
    except Exception as e:
        print(f"\nОшибка: {e}")
        print("\nУбедитесь, что установлен пакет python-uinput и программа запущена с правами sudo")

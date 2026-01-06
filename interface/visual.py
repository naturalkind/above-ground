import pygame
import math
import sys

# Инициализация Pygame
pygame.init()

# Настройки окна
WIDTH, HEIGHT = 400, 400
screen = pygame.Surface((WIDTH, HEIGHT))
window = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Pitch and Roll Indicator")

# Цвета
BACKGROUND = (25, 25, 40)
BLUE_LIGHT = (60, 129, 198)
BLUE_DARK = (35, 78, 121)
GRAY_LIGHT = (206, 206, 206)
GRAY_DARK = (155, 155, 155)
WHITE = (255, 255, 255)
GRAY = (110, 110, 110)
ORANGE = (253, 146, 96)
BLACK = (0, 0, 0)
DARK_GRAY = (50, 50, 50)

class PitchRollIndicator:
    def __init__(self, x, y, size):
        self.x = x
        self.y = y
        self.size = size
        self.radius = size // 2
        self.pitch = 0  # угол тангажа в градусах
        self.roll = 0   # угол крена в градусах
        
    def set_pitch(self, pitch):
        """Установить угол тангажа (-90 до 90 градусов)"""
        self.pitch = max(-90, min(90, pitch))
        
    def set_roll(self, roll):
        """Установить угол крена (-180 до 180 градусов)"""
        self.roll = roll % 360
        if self.roll > 180:
            self.roll -= 360
    
    def draw(self, surface):
        # Очищаем область под прибором
        pygame.draw.circle(surface, BACKGROUND, (self.x, self.y), self.radius)
        
        # Создаем временную поверхность для вращения
        temp_surface = pygame.Surface((self.size, self.size), pygame.SRCALPHA)
        
        # Рисуем прибор на временной поверхности
        self._draw_instrument(temp_surface)
        
        # Поворачиваем временную поверхность согласно углу крена
        rotated_surface = pygame.transform.rotate(temp_surface, -self.roll)
        
        # Получаем новый прямоугольник после поворота
        rotated_rect = rotated_surface.get_rect(center=(self.x, self.y))
        
        # Рисуем повернутую поверхность на основном экране
        surface.blit(rotated_surface, rotated_rect)
        
        # Рисуем фиксированный указатель крена
        self._draw_roll_indicator(surface)
    
    def _draw_instrument(self, surface):
        # Внешнее кольцо
        pygame.draw.circle(surface, GRAY, (self.radius, self.radius), self.radius, 2)
        
        # Внутреннее кольцо
        pygame.draw.circle(surface, GRAY, (self.radius, self.radius), self.radius - 10, 2)
        
        # Полукруг для неба и земли
        self._draw_sky_ground(surface)
        
        # Шкала тангажа
        self._draw_pitch_scale(surface)
        
        # Центральный самолетик
        self._draw_airplane(surface)
        
        # Метки крена
        self._draw_roll_ticks(surface)
        
        # Указатели направления
        self._draw_direction_indicators(surface)
    
    def _draw_sky_ground(self, surface):
        center_x, center_y = self.radius, self.radius
        
        # Вычисляем смещение для тангажа (1 пиксель = 1 градус)
        pitch_offset = self.pitch * 0.5
        
        # Небо (синий полукруг сверху)
        sky_rect = pygame.Rect(center_x - self.radius, 
                              center_y - self.radius - pitch_offset, 
                              self.size, self.radius)
        pygame.draw.rect(surface, BLUE_DARK, sky_rect)
        
        # Градиент для неба (упрощенный)
        for i in range(30):
            color = (BLUE_LIGHT[0] - i, BLUE_LIGHT[1] - i*2, BLUE_LIGHT[2] - i)
            pygame.draw.line(surface, color, 
                           (center_x - self.radius, center_y - self.radius - pitch_offset + i),
                           (center_x + self.radius, center_y - self.radius - pitch_offset + i))
        
        # Земля (серый полукруг снизу)
        ground_rect = pygame.Rect(center_x - self.radius, 
                                 center_y - pitch_offset, 
                                 self.size, self.radius)
        pygame.draw.rect(surface, GRAY_DARK, ground_rect)
        
        # Градиент для земли
        for i in range(30):
            color = (GRAY_LIGHT[0] - i, GRAY_LIGHT[1] - i, GRAY_LIGHT[2] - i)
            y_pos = center_y - pitch_offset + i
            pygame.draw.line(surface, color, 
                           (center_x - self.radius, y_pos),
                           (center_x + self.radius, y_pos))
        
        # Горизонтальная линия разделения
        pygame.draw.line(surface, WHITE, 
                        (center_x - self.radius, center_y - pitch_offset),
                        (center_x + self.radius, center_y - pitch_offset), 2)
    
    def _draw_pitch_scale(self, surface):
        center_x, center_y = self.radius, self.radius
        pitch_offset = self.pitch * 0.5
        
        # Основные линии тангажа
        pitch_lines = [
            (0, 3, 40, WHITE),      # 0 градусов
            (5, 2, 30, WHITE),      # 5 градусов
            (10, 2, 25, WHITE),     # 10 градусов
            (20, 2, 20, WHITE),     # 20 градусов
            (30, 2, 15, WHITE),     # 30 градусов
            (-5, 2, 30, GRAY),      # -5 градусов
            (-10, 2, 25, GRAY),     # -10 градусов
            (-20, 2, 20, GRAY),     # -20 градусов
            (-30, 2, 15, GRAY),     # -30 градусов
        ]
        
        for angle, width, length, color in pitch_lines:
            y_pos = center_y - pitch_offset + angle * 5
            if 0 <= y_pos <= self.size:
                start_x = center_x - length
                end_x = center_x + length
                pygame.draw.line(surface, color, (start_x, y_pos), (end_x, y_pos), width)
                
                # Цифры для основных линий
                if abs(angle) in [0, 10, 20, 30]:
                    font = pygame.font.SysFont(None, 20)
                    text = font.render(str(abs(angle)), True, color)
                    if angle >= 0:
                        surface.blit(text, (center_x + length + 5, y_pos - 10))
                    else:
                        surface.blit(text, (center_x - length - 25, y_pos - 10))
    
    def _draw_airplane(self, surface):
        center_x, center_y = self.radius, self.radius
        
        # Фюзеляж (корпус самолета)
        pygame.draw.line(surface, ORANGE, 
                        (center_x - 15, center_y),
                        (center_x + 15, center_y), 3)
        
        # Крылья
        pygame.draw.line(surface, ORANGE,
                        (center_x - 40, center_y - 2),
                        (center_x + 40, center_y - 2), 4)
        
        # Хвостовое оперение
        pygame.draw.polygon(surface, ORANGE, [
            (center_x - 2, center_y - 8),
            (center_x + 2, center_y - 8),
            (center_x, center_y - 15)
        ])
    
    def _draw_roll_ticks(self, surface):
        center_x, center_y = self.radius, self.radius
        outer_radius = self.radius - 15
        inner_radius = outer_radius - 8
        
        # Рисуем метки для крена
        for angle in range(0, 360, 10):
            rad = math.radians(angle)
            
            # Внешняя точка
            x1 = center_x + outer_radius * math.sin(rad)
            y1 = center_y - outer_radius * math.cos(rad)
            
            # Внутренняя точка (длина метки зависит от угла)
            tick_length = 5
            if angle % 30 == 0:
                tick_length = 10
            if angle % 90 == 0:
                tick_length = 15
            
            x2 = center_x + (outer_radius - tick_length) * math.sin(rad)
            y2 = center_y - (outer_radius - tick_length) * math.cos(rad)
            
            # Выбираем цвет в зависимости от угла
            if angle <= 180:
                color = WHITE
            else:
                color = GRAY
            
            pygame.draw.line(surface, color, (x1, y1), (x2, y2), 2)
            
            # Подписи для основных углов
            if angle % 30 == 0:
                font = pygame.font.SysFont(None, 18)
                text_angle = angle if angle <= 180 else angle - 360
                text = font.render(str(text_angle), True, color)
                text_rect = text.get_rect()
                text_x = center_x + (outer_radius - 20) * math.sin(rad) - text_rect.width // 2
                text_y = center_y - (outer_radius - 20) * math.cos(rad) - text_rect.height // 2
                surface.blit(text, (text_x, text_y))
    
    def _draw_direction_indicators(self, surface):
        center_x, center_y = self.radius, self.radius
        radius = self.radius - 20
        
        # Верхний треугольник (0°)
        points_up = [
            (center_x, center_y - radius),
            (center_x - 10, center_y - radius + 15),
            (center_x + 10, center_y - radius + 15)
        ]
        pygame.draw.polygon(surface, WHITE, points_up)
        
        # Правый треугольник (90°)
        points_right = [
            (center_x + radius, center_y),
            (center_x + radius - 15, center_y - 10),
            (center_x + radius - 15, center_y + 10)
        ]
        pygame.draw.polygon(surface, WHITE, points_right)
        
        # Левый треугольник (270°)
        points_left = [
            (center_x - radius, center_y),
            (center_x - radius + 15, center_y - 10),
            (center_x - radius + 15, center_y + 10)
        ]
        pygame.draw.polygon(surface, WHITE, points_left)
        
        # Нижний треугольник (180°)
        points_down = [
            (center_x, center_y + radius),
            (center_x - 10, center_y + radius - 15),
            (center_x + 10, center_y + radius - 15)
        ]
        pygame.draw.polygon(surface, GRAY, points_down)
        
        # Треугольники для 45° и -45°
        for angle, color in [(45, WHITE), (135, WHITE), (225, GRAY), (315, GRAY)]:
            rad = math.radians(angle)
            x = center_x + radius * math.sin(rad)
            y = center_y - radius * math.cos(rad)
            
            # Создаем маленький треугольник
            triangle_size = 8
            points = []
            for i in range(3):
                tri_angle = angle + i * 120
                tri_rad = math.radians(tri_angle)
                tri_x = x + triangle_size * math.sin(tri_rad)
                tri_y = y - triangle_size * math.cos(tri_rad)
                points.append((tri_x, tri_y))
            
            pygame.draw.polygon(surface, color, points)
    
    def _draw_roll_indicator(self, surface):
        """Рисует фиксированный указатель крена (не вращается)"""
        center_x, center_y = self.x, self.y
        
        # Маленький треугольник сверху
        points = [
            (center_x, center_y - self.radius + 10),
            (center_x - 10, center_y - self.radius + 25),
            (center_x + 10, center_y - self.radius + 25)
        ]
        pygame.draw.polygon(surface, ORANGE, points)
        
        # Точка в центре для тангажа
        pygame.draw.circle(surface, ORANGE, (center_x, center_y), 3)
        pygame.draw.circle(surface, WHITE, (center_x, center_y), 6, 1)

# Создаем индикатор
indicator = PitchRollIndicator(WIDTH // 2, HEIGHT // 2, 350)

# Основной цикл
clock = pygame.time.Clock()
auto_mode = True
pitch_change = 0.1
roll_change = 0.5

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_UP:
                indicator.set_pitch(indicator.pitch + 5)
                auto_mode = False
            elif event.key == pygame.K_DOWN:
                indicator.set_pitch(indicator.pitch - 5)
                auto_mode = False
            elif event.key == pygame.K_LEFT:
                indicator.set_roll(indicator.roll - 5)
                auto_mode = False
            elif event.key == pygame.K_RIGHT:
                indicator.set_roll(indicator.roll + 5)
                auto_mode = False
            elif event.key == pygame.K_SPACE:
                # Сброс в нулевое положение
                indicator.set_pitch(0)
                indicator.set_roll(0)
                auto_mode = False
            elif event.key == pygame.K_a:
                # Включить/выключить автоматический режим
                auto_mode = not auto_mode
    
    # Автоматическое изменение углов в авто-режиме
    if auto_mode:
        new_pitch = indicator.pitch + pitch_change
        new_roll = indicator.roll + roll_change
        
        if new_pitch > 30 or new_pitch < -30:
            pitch_change = -pitch_change
        if new_roll > 45 or new_roll < -45:
            roll_change = -roll_change
            
        indicator.set_pitch(new_pitch)
        indicator.set_roll(new_roll)
    
    # Очистка экрана
    window.fill(BACKGROUND)
    
    # Рисуем индикатор
    indicator.draw(window)
    
    # Отображаем текущие значения
    font = pygame.font.SysFont(None, 28)
    pitch_text = font.render(f"Pitch: {indicator.pitch:.1f}°", True, WHITE)
    roll_text = font.render(f"Roll: {indicator.roll:.1f}°", True, WHITE)
    control_text = font.render("Controls: Arrows = Manual, Space = Reset, A = Auto", True, WHITE)
    
    window.blit(pitch_text, (10, 10))
    window.blit(roll_text, (10, 40))
    window.blit(control_text, (10, HEIGHT - 30))
    
    # Обновление экрана
    pygame.display.flip()
    clock.tick(60)

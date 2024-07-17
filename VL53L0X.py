import smbus
import time
import numpy as np
from collections import deque

# Адрес VL53L0X
VL53L0X_ADDRESS = 0x29

# Регистры
SYSRANGE_START = 0x00
RESULT_RANGE_STATUS = 0x14
SOFT_RESET_GO2_SOFT_RESET_N = 0xBF
IDENTIFICATION_MODEL_ID = 0xC0
VALID_RANGE_MIN = 0
VALID_RANGE_MAX = 2000

bus = smbus.SMBus(5)

def write_byte_data(addr, reg, data):
    bus.write_byte_data(addr, reg, data)

def write_word_data(addr, reg, data):
    bus.write_word_data(addr, reg, data)

def read_byte_data(addr, reg):
    return bus.read_byte_data(addr, reg)

def read_word_data(addr, reg):
    return bus.read_word_data(addr, reg)

def init_sensor():
    # Выполнить сброс
    write_byte_data(VL53L0X_ADDRESS, SOFT_RESET_GO2_SOFT_RESET_N, 0x00)
    time.sleep(0.1)
    write_byte_data(VL53L0X_ADDRESS, SOFT_RESET_GO2_SOFT_RESET_N, 0x01)
    time.sleep(0.1)
    print("Sensor initialized")
#    write_byte_data(VL53L0X_ADDRESS, 0x91, 0x00) # В этом режиме датчик выполняет одно измерение при каждом запросе
#    write_byte_data(VL53L0X_ADDRESS, 0x91, 0x3c) # В этом режиме датчик автоматически выполняет измерения с определенной частотой

def check_sensor():
    model_id = read_byte_data(VL53L0X_ADDRESS, IDENTIFICATION_MODEL_ID)
    if model_id != 0xEE:
        print(f"Unexpected model ID: {model_id}")
        return False
    print("Sensor check passed")
    return True

def measure_distance():
    write_byte_data(VL53L0X_ADDRESS, SYSRANGE_START, 0x01)
    
    start_time = time.time()
    while (time.time() - start_time) < 1.0:
        if read_byte_data(VL53L0X_ADDRESS, RESULT_RANGE_STATUS) & 0x01:
            break
        time.sleep(0.010)
    
    # Чтение отдельных байтов
    low_byte = read_byte_data(VL53L0X_ADDRESS, RESULT_RANGE_STATUS + 10)
    high_byte = read_byte_data(VL53L0X_ADDRESS, RESULT_RANGE_STATUS + 11)
    
    print(f"Low byte: {low_byte}, High byte: {high_byte}")
    
    # Изменение порядка байтов
    range_mm = (low_byte << 8) | high_byte
    return range_mm


class Simple1DExponentialAverage:
    def __init__(self, tau, initial_estimate=0):
        self.tau = tau
        self._last_estimate = initial_estimate
        self._last_ts = time.monotonic()

    def _get_alpha(self):
        now = time.monotonic()
        dt = now - self._last_ts
        alpha = 1 - np.exp(- dt / self.tau)
        self._last_ts = now
        return alpha

    def update_estimate(self, mea):
        alpha = self._get_alpha()
        current_estimate = alpha * mea + (1 - alpha) * self._last_estimate
        self._last_estimate = current_estimate
        return current_estimate

class AdvancedCombinedFilter:
    def __init__(self, window_size=5, alpha=0.5, beta=0.1, tau=1.0):
        self.moving_average = deque(maxlen=window_size)
        self.alpha = alpha
        self.beta = beta
        self.x_hat = None
        self.v_hat = 0
        self.t_prev = None
        self.exp_filter = Simple1DExponentialAverage(tau)

    def update(self, measurement):
        if VALID_RANGE_MIN < measurement < VALID_RANGE_MAX:
            # Обновление скользящего среднего
            self.moving_average.append(measurement)
            moving_avg = sum(self.moving_average) / len(self.moving_average)

            # Применение экспоненциального фильтра
            exp_filtered = self.exp_filter.update_estimate(moving_avg)

            t_now = time.time()

            if self.x_hat is None:
                self.x_hat = exp_filtered
                self.v_hat = 0
                self.t_prev = t_now
                return self.x_hat

            # Применение α-β фильтра к результату экспоненциального фильтра
            dt = t_now - self.t_prev
            self.t_prev = t_now

            # Предсказание
            x_pred = self.x_hat + self.v_hat * dt
            
            # Коррекция
            residual = exp_filtered - x_pred
            self.x_hat = x_pred + self.alpha * residual
            self.v_hat = self.v_hat + (self.beta * residual) / dt

            return self.x_hat
        return None

class SensorFrequencyCounter:
    def __init__(self, update_interval=1.0):
        self.update_interval = update_interval
        self.measurement_count = 0
        self.last_update_time = time.time()

    def update(self):
        self.measurement_count += 1
        current_time = time.time()
        elapsed_time = current_time - self.last_update_time

        if elapsed_time >= self.update_interval:
            frequency = self.measurement_count / elapsed_time
            self.measurement_count = 0
            self.last_update_time = current_time
            return frequency
        return None

def main():
    init_sensor()
    time.sleep(1)

    if not check_sensor():
        print("Sensor initialization failed")
        return

#    advanced_filter = AdvancedCombinedFilter(window_size=5, alpha=0.5, beta=0.1, tau=1.0)
    advanced_filter = AdvancedCombinedFilter(window_size=3, alpha=0.7, beta=0.2, tau=0.5)
    
    frequency_counter = SensorFrequencyCounter(update_interval=5.0)
    try:
        while True:
            raw_distance = measure_distance()
            if raw_distance is not None:
                filtered_distance = advanced_filter.update(raw_distance)
                if filtered_distance is not None:
                    print(f"Расстояние: {filtered_distance:.2f} мм, Данные без фильтра: {raw_distance} мм")
                else:
                    print("Недопустимое измерение")
                # Обновление счетчика частоты
                frequency = frequency_counter.update()
                if frequency is not None:
                    print(f"Частота работы сенсора: {frequency:.2f} Гц")
            else:
                print("Ошибка измерения")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Программа остановлена пользователем")
    finally:
        bus.close()

if __name__ == "__main__":
    main()




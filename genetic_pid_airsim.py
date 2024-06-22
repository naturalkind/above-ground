import time
import airsim
import numpy as np
from deap import base, creator, tools, algorithms

########################
# Генетический алгоритм для создания pid
########################

# Функция для симуляции квадрокоптера с заданными параметрами PID
def simulate_pid(params):
    # Инициализация AirSim
    client = airsim.MultirotorClient()

    client.confirmConnection()

    # Задание параметров PID
    kp, ki, kd = params

    # Задание других параметров симуляции
    max_time = 10  # Продолжительность симуляции в секундах
    dt = 0.1  # Шаг времени в секундах

    # Начальные условия
    initial_pose = airsim.Pose(airsim.Vector3r(0, 0, -10), airsim.to_quaternion(0, 0, 0))
    client.simSetVehiclePose(initial_pose, True)

    # Симуляция
    for _ in range(int(max_time / dt)):
        # Получение текущей позиции
        pose = client.simGetVehiclePose()
        position = pose.position
        print (position)
        # Рассчет управляющего воздействия с использованием PID
        control_input = kp * position.x_val + ki * position.y_val + kd * position.z_val

        # Применение управляющего воздействия
#        client.moveByVelocityZ(control_input, 0, 0, dt)
        client.moveByRC(rcdata = airsim.RCData(pitch = 0.0,
                                               throttle = control_input,
                                               yaw=0.0,
                                               roll=0.0,
                                               is_initialized = True,
                                               is_valid = True))        

    # Получение финальной позиции
    final_pose = client.simGetVehiclePose()
    final_position = final_pose.position

    # Закрытие соединения
    client.reset()
    client.enableApiControl(False)

    # Возвращение значения функции приспособленности (цель - минимизация расстояния до целевой точки)
    return np.sqrt(final_position.x_val**2 + final_position.y_val**2 + final_position.z_val**2),

# Создание класса FitnessMin для минимизации функции приспособленности
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))

# Создание класса Individual с одним атрибутом, представляющим параметры PID
creator.create("Individual", list, fitness=creator.FitnessMin)

# Определение функции для инициализации особи
def init_individual():
    return [np.random.uniform(0, 1) for _ in range(3)]  # Инициализация случайных значений для параметров PID

# Определение генетических операторов
toolbox = base.Toolbox()
toolbox.register("individual", tools.initIterate, creator.Individual, init_individual)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)
toolbox.register("evaluate", simulate_pid)
toolbox.register("mate", tools.cxBlend, alpha=0.5)
toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=0.2, indpb=0.2)
toolbox.register("select", tools.selTournament, tournsize=3)

# Создание начальной популяции
population = toolbox.population(n=10)

# Запуск генетического алгоритма
algorithms.eaMuPlusLambda(population, toolbox, mu=10, lambda_=20, cxpb=0.7, mutpb=0.3, ngen=10, stats=None, halloffame=None)

# Вывод лучшей особи
best_individual = tools.selBest(population, k=1)[0]
print("Best Individual:", best_individual)
print("Best Fitness:", best_individual.fitness.values)


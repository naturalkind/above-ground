import time
import airsim
import numpy as np
from deap import base, creator, tools, algorithms

########################
# Генетический алгоритм для создания pid
########################

## Функция для симуляции квадрокоптера с заданными параметрами PID
#def simulate_pid(params):
#    # Инициализация AirSim
#    client = airsim.MultirotorClient()

#    client.confirmConnection()

#    # Задание параметров PID
#    kp, ki, kd = params

#    # Задание других параметров симуляции
#    max_time = 10  # Продолжительность симуляции в секундах
#    dt = 0.1  # Шаг времени в секундах

#    # Начальные условия
#    initial_pose = airsim.Pose(airsim.Vector3r(0, 0, -10), airsim.to_quaternion(0, 0, 0))
#    client.simSetVehiclePose(initial_pose, True)

#    # Симуляция
#    for _ in range(int(max_time / dt)):
#        # Получение текущей позиции
#        pose = client.simGetVehiclePose()
#        position = pose.position
#        print (position)
#        # Рассчет управляющего воздействия с использованием PID
#        control_input = kp * position.x_val + ki * position.y_val + kd * position.z_val

#        # Применение управляющего воздействия
##        client.moveByVelocityZ(control_input, 0, 0, dt)
#        client.moveByRC(rcdata = airsim.RCData(pitch = 0.0,
#                                               throttle = control_input,
#                                               yaw=0.0,
#                                               roll=0.0,
#                                               is_initialized = True,
#                                               is_valid = True))        

#    # Получение финальной позиции
#    final_pose = client.simGetVehiclePose()
#    final_position = final_pose.position

#    # Закрытие соединения
#    client.reset()
#    client.enableApiControl(False)

#    # Возвращение значения функции приспособленности (цель - минимизация расстояния до целевой точки)
#    return np.sqrt(final_position.x_val**2 + final_position.y_val**2 + final_position.z_val**2),

## Создание класса FitnessMin для минимизации функции приспособленности
#creator.create("FitnessMin", base.Fitness, weights=(-1.0,))

## Создание класса Individual с одним атрибутом, представляющим параметры PID
#creator.create("Individual", list, fitness=creator.FitnessMin)

## Определение функции для инициализации особи
#def init_individual():
#    return [np.random.uniform(0, 1) for _ in range(3)]  # Инициализация случайных значений для параметров PID

## Определение генетических операторов
#toolbox = base.Toolbox()
#toolbox.register("individual", tools.initIterate, creator.Individual, init_individual)
#toolbox.register("population", tools.initRepeat, list, toolbox.individual)
#toolbox.register("evaluate", simulate_pid)
#toolbox.register("mate", tools.cxBlend, alpha=0.5)
#toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=0.2, indpb=0.2)
#toolbox.register("select", tools.selTournament, tournsize=3)

## Создание начальной популяции
#population = toolbox.population(n=10)

## Запуск генетического алгоритма
#algorithms.eaMuPlusLambda(population, toolbox, mu=10, lambda_=20, cxpb=0.7, mutpb=0.3, ngen=10, stats=None, halloffame=None)

## Вывод лучшей особи
#best_individual = tools.selBest(population, k=1)[0]
#print("Best Individual:", best_individual)
#print("Best Fitness:", best_individual.fitness.values)

######
######
"""

Этот код создает экземпляр PID-регулятора с определенными коэффициентами Kp, Ki и Kd. 
Затем он подключается к симулятору AirSim и запускает цикл управления. В цикле он 
получает текущую высоту, вычисляет ошибку относительно целевой высоты, обновляет 
PID-регулятор и применяет получившийся управляющий сигнал к функции moveByRC

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
    def __init__(self, target_height, Kp, Ki, Kd):
        self.target_height = target_height
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0
        self.integral = 0

    def get_pid(self, current_height):
        error = self.target_height - current_height
        self.integral += error
        derivative = error - self.last_error
        self.last_error = error

        pid_output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return pid_output

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

# Create a PID controller object
pid_controller = PIDController(target_height, Kp, Ki, Kd)

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
    
#    pid_output = f_kalman.update(target_height, current_height)
    # Calculate the PID output
    pid_output = pid_controller.get_pid(current_height)

    # Adjust throttle based on PID output
    client.moveByRC(rcdata = airsim.RCData(pitch = 0.0,
                                           throttle = -pid_output,
                                           yaw=0.0,
                                           roll=0.0,
                                           is_initialized = True,
                                           is_valid = True))
    print (-pid_output, current_height, "error:", pid_controller.last_error, "time:", time.time()-start_time)
    time.sleep(0.01)
    
    

#import random

## Функция генерации начальной популяции
#def generate_population(size):
#    population = []
#    for _ in range(size):
#        kp = random.uniform(0, 1)
#        ki = random.uniform(0, 1)
#        kd = random.uniform(0, 1)
#        population.append((kp, ki, kd))
#    return population

## Функция вычисления приспособленности популяции
#def fitness(population):
#    fitness_scores = []
#    for params in population:
#        # Здесь происходит симуляция полета quadrocopter с использованием PID-регулятора с заданными параметрами params
#        # Вычисляем приспособленность на основе успешности слежения за объектом (например, расстояния до него)
#        fitness_scores.append(compute_fitness(params))
#    return fitness_scores

## Функция селекции родителей для скрещивания
#def select_parents(population, scores, num_parents):
#    parents = []
#    for _ in range(num_parents):
#        max_index = scores.index(max(scores))
#        parents.append(population[max_index])
#        scores[max_index] = -1 * float('inf') # Удаляем выбранного родителя из списка кандидатов
#    return parents

## Функция скрещивания родителей
#def crossover(parents, offspring_size):
#    offspring = []
#    while len(offspring) < offspring_size:
#        parent1_index = random.randint(0, len(parents) - 1)
#        parent2_index = random.randint(0, len(parents) - 1)
#        if parent1_index != parent2_index:
#            parent1 = parents[parent1_index]
#            parent2 = parents[parent2_index]
#            offspring.append((
#                parent1[0],
#                parent2[1],
#                parent1[2]
#            ))
#    return offspring

## Функция мутации потомков
#def mutate(offspring, mutation_rate):
#    for i in range(len(offspring)):
#        if random.random() < mutation_rate:
#            offspring[i] = (
#                random.uniform(0, 1),
#                random.uniform(0, 1),
#                random.uniform(0, 1)
#            )
#    return offspring

## Основная функция генетического алгоритма
#def genetic_algorithm(population_size, num_generations, mutation_rate, num_parents, num_offspring):
#    population = generate_population(population_size)
#    
#    for generation in range(num_generations):
#        fitness_scores = fitness(population)
#        
#        parents = select_parents(population, fitness_scores, num_parents)
#        
#        offspring = crossover(parents, num_offspring)
#        
#        offspring = mutate(offspring, mutation_rate)
#        
#        population = parents + offspring
#    
#    # Извлекаем лучший набор параметров и возвращаем его
#    best_params = max(population, key=compute_fitness)
#    return best_params

## Функция вычисления приспособленности на основе успешности слежения за объектом
#def compute_fitness(params):
#    # Реализация логики для вычисления приспособленности
#    pass

## Пример использования генетического алгоритма
#population_size = 50
#num_generations = 100
#mutation_rate = 0.1
#num_parents = 10
#num_offspring = 20

#best_params = genetic_algorithm(population_size, num_generations, mutation_rate, num_parents, num_offspring)

#print("Лучшие параметры найдены:", best_params)

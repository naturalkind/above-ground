import random
import time
import json
import numpy as np
import os
from scipy.signal import argrelextrema
from matplotlib import pyplot as plt
from deap import base, creator, tools, algorithms

# data = {"list_time_thr":list_time_thr, 
#         "list_rc_thr":list_rc_thr, 
#         "list_target_thr":list_target_thr,
#         "list_pid_thr":list_pid_thr,

#         "list_target_yaw":list_target_yaw,
#         "list_rc_yaw":list_rc_yaw,
#         "list_time_yaw":list_time_yaw,
#         "list_pid_yaw":list_pid_yaw

#         }

count_data = []
big_data = {}

def simulate_pid(params):
    params = params
    _big_data = big_data[_key]["list_target_thr"][-1]
    # _big_data = np.average(big_data[_key]["list_target_thr"])
    return np.sqrt(_big_data**2),

def ga_run(t_data_key, big_data):
    # Создание класса FitnessMin для минимизации функции приспособленности
    creator.create("FitnessMin", base.Fitness, weights=(-1.0,))

    ## Создание класса Individual с одним атрибутом, представляющим параметры PID
    creator.create("Individual", list, fitness=creator.FitnessMin)

    ## Определение функции для инициализации особи
    def init_individual():
        global _key
        _key = choose_and_remove(t_data_key)

        return big_data[_key]["list_pid_thr"][-1]#, np.average(big_data[_key]["list_target_thr"])
        # return [np.random.uniform(0, 0.004) for _ in range(3)]  # Инициализация случайных значений для параметров PID

    ## Определение генетических операторов
    toolbox = base.Toolbox()
    toolbox.register("individual", tools.initIterate, creator.Individual, init_individual)
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)
    toolbox.register("evaluate", simulate_pid)
    toolbox.register("mate", tools.cxBlend, alpha=0.5)
    toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=0.2, indpb=0.2)
    toolbox.register("select", tools.selTournament, tournsize=3)

    ## Создание начальной популяции
    population = toolbox.population(n=26)

    ## Запуск генетического алгоритма
    algorithms.eaMuPlusLambda(population, toolbox, mu=100, lambda_=30, cxpb=0.7, mutpb=0.3, ngen=10, stats=None, halloffame=None)

    ## Вывод лучшей особи
    best_individual = tools.selBest(population, k=1)[0]
    print("Best Individual:", best_individual)
    print("Best Fitness:", best_individual.fitness.values)

# this will choose one and remove it
def choose_and_remove(items):
    # pick an item index
    if items:
        index = random.randrange(len(items))
        return items.pop(index)
    # nothing left!
    return None

def visual_pid(data):
    OS = "thr"
    START = 0
    END = len(data["list_time_"+OS])

    # visual
    temp_time_list = data["list_time_"+OS][START:END]
    sumOfNums = sum(temp_time_list)
    count = len(temp_time_list)
    average = sumOfNums / count
    list_time_arr = np.arange(0, count)#np.array(list_time)

    np_list_pid_thr = np.array(data["list_pid_"+OS])
    sumKp = sum(np_list_pid_thr[START:END][0])
    averageKp = sumKp/count


    print (averageKp, sum(temp_time_list[:]))
    sum_lisg_rc = sum(data["list_rc_"+OS][START:END])
    average_rc = sum_lisg_rc/count

    list_rc_arr = np.array(data["list_rc_"+OS][START:END])
    ix_max = argrelextrema(list_rc_arr, np.greater)
    ix_min = argrelextrema(list_rc_arr, np.less)

    plt.scatter(list_time_arr[ix_max], list_rc_arr[ix_max])
    plt.scatter(list_time_arr[ix_min], list_rc_arr[ix_min])
    plt.plot(data["list_rc_"+OS][START:END])
    plt.axline((0, average_rc), (count, average_rc))
    plt.title('PID')
    plt.xlabel('step')
    plt.ylabel('throttle')
    plt.show()

def open_files_in_folder(path):
    items = os.scandir(path)
    for item in items:
        if item.is_file():
            

            with open(item.path, 'r') as file:
                json_data = file.read()
            data = json.loads(json_data)
            # нужно получить список
            temp_count_data = [len(data[d]) for d in list(data.keys())]
            count_data.append(temp_count_data[0])

            big_data[item.name] = data
            # print(f"Opening file: {item.name}    {temp_count_data[0]}") #path: {item.path}
    _max = max(count_data)
    _min = min(count_data)
    print (count_data, _max, _min)

    # подготовка данных к генетическомму алгоритму thr
    for d in big_data:
        # print (f"{d}, {big_data[d].keys()}")
        for k in big_data[d]:
            if k[-3:] == "thr":
                big_data[d][k] = big_data[d][k][:_min]
                
    # Визуализация
    # for d in big_data:
    #     # print (f"{d}, {big_data[d].keys()}")
    #     print (d)
    #     visual_pid(big_data[d])
    
    t_data_key = list(big_data.keys())
    # _key = choose_and_remove(t_data_key)
    # print (big_data[_key]["list_pid_thr"])
    ga_run(t_data_key, big_data)


                

if __name__ == '__main__':
    folder_path = ""
    open_files_in_folder(folder_path)

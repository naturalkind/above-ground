import os
import sys
import cv2
import time
import curses
import socket
import pickle
import struct
import tracker_lib
from multiprocessing import Process, Value, Array, Manager, Queue
from collections import deque
from itertools import cycle
from yamspy import MSPy
from threading import Thread
 

def track_objects(input_queue, output_queue, dict_):
    csrt_tracker = cv2.TrackerCSRT_create()
    kcf_tracker = cv2.TrackerKCF_create()
    init_switch = False
    while True:
        img = input_queue.get()

        if img is None:
            break

        # Тут нужно добавить код для обработки видеофрейма и отслеживания объектов с помощью CSRT и KCF трекеров

        img_center = lib_start.get_center(img, 0, 0, img.shape[1], img.shape[0])
        if dict_["state"] > 1:
            if sum(dict_["bbox"][-2:]) > 10:
                cv2.rectangle(img, dict_["bbox"], (255, 0, 0), 10)
                kcf_tracker.init(img, dict_["bbox"])
                csrt_tracker.init(img, dict_["bbox"])
                init_switch = True
                dict_["state"] = 0
                dict_["state_2"] = False
        if init_switch:
            # Обновление трекера CSRT
            csrt_success, csrt_bbox = csrt_tracker.update(img)
            
            # Обновление трекера KCF
            kcf_success, kcf_bbox = kcf_tracker.update(img)
                  

            # Взвешивание результатов трекинга
            if csrt_success and kcf_success:                
                bbox = (0.6 * csrt_bbox[0] + 0.4 * kcf_bbox[0],
                         0.6 * csrt_bbox[1] + 0.4 * kcf_bbox[1],
                         0.6 * csrt_bbox[2] + 0.4 * kcf_bbox[2],
                         0.6 * csrt_bbox[3] + 0.4 * kcf_bbox[3])

                bbox = [int(x) for x in bbox]
                lib_start.image_process(img, bbox, img_center)
            
            elif csrt_success:
                bbox = csrt_bbox 
                lib_start.image_process(img, bbox, img_center)
            elif kcf_success:     
                bbox = kcf_bbox 
                lib_start.image_process(img, bbox, img_center)
        #----------------------->
        output_queue.put(img)

    input_queue.close()
    output_queue.close()

if __name__ == '__main__':
    dict_ = Manager().dict()
    dict_["init_tracker"] = False
    dict_["state"] = 0
    dict_["state_2"] = True
    lib_start = tracker_lib.TrackerLib()
    lib_start.create_win()
    input_queue = Queue()
    output_queue = Queue()

    p = Process(target=track_objects, args=(input_queue, output_queue, dict_, ))
    p.start()

    cap = cv2.VideoCapture(1)
    ix = 0
    sum_ = 0 
    while True:
        start_time = time.time()
        ret, frame = cap.read()

        if not ret:
            break

        input_queue.put(frame)

        output_frame = output_queue.get()
        end_time = time.time()
        seconds = end_time - start_time
        fps = 1.0 / seconds
        
        cv2.putText(output_frame, f"{int(fps)} fps", (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,0,255),2) #cv2.FONT_HERSHEY_COMPLEX
        # Если начальные и конечные координаты прямоугольника определены
        if lib_start.start_x != -1 and lib_start.end_x != -1:
            if lib_start.state != 0:
                # Рисование прямоугольника на изображении
                cv2.rectangle(output_frame, (lib_start.start_x, lib_start.start_y), (lib_start.end_x, lib_start.end_y), (0, 255, 0), 2)
                dict_["bbox"] = lib_start.bbox
                if dict_["state_2"]:
                    dict_["state"] = lib_start.state
                else:
                    lib_start.state = dict_["state"]
        cv2.imshow("win", output_frame)
        if cv2.waitKey(1) & 0xff == ord('q'):
            break    
        ix += 1
        sum_ += fps
        #if ix == 100:
        #    break
    print (sum_/ix)
    
    input_queue.put(None)
    p.join()

    cap.release()
    cv2.destroyAllWindows()
            
"""
1 запускать автоматически 
2 при нажатии армить
3 при нажатии запускать выбор цели

"""    

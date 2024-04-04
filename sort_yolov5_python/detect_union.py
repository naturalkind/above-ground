import multiprocessing
import cv2
import time
import os
import sys
import socket
import pickle
import struct
import tracker_lib
from multiprocessing import Process, Value, Array, Manager, Queue
from threading import Thread
from rknnpool import rknnPoolExecutor
#Функция обработки изображений, вам необходимо изменить ее самостоятельно в процессе фактического применения
from func import myFunc


def cv_tracker(dict_, pipe, pipe2):
    csrt_tracker = cv2.TrackerCSRT_create()
    kcf_tracker = cv2.TrackerKCF_create()
    init_switch = False
    bbox = [0,0,0,0]
    while True:
        img = pipe2.recv()
        
        if img is None:
            break

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
            elif csrt_success:
                bbox = csrt_bbox 
            elif kcf_success:     
                bbox = kcf_bbox 
        #----------------------->
        pipe.send(bbox)

def yolo_tracker(dict_, pipe):
    pass



if __name__ == '__main__':
    
    modelPath = "./rknnModel/yolov5s_relu_tk2_RK3588_i8.rknn"
    # Увеличьте количество линий, увеличьте скорость
    TPEs = 3
    pool = rknnPoolExecutor(
        rknnModel=modelPath,
        TPEs=TPEs,
        func=myFunc)

    dict_ = Manager().dict()
    dict_["init_tracker"] = False
    dict_["state"] = 0
    dict_["state_2"] = True
    lib_start = tracker_lib.TrackerLib()
    lib_start.create_win()
    
    
    parent_conn, child_conn = multiprocessing.Pipe()
    parent_conn2, child_conn2 = multiprocessing.Pipe()
    p = multiprocessing.Process(target=cv_tracker, args=(dict_, child_conn, parent_conn2, ))
    p.start()

    cap = cv2.VideoCapture(1)
    # Инициализируйте требуемый кадр асинхронно
    if (cap.isOpened()):
        for i in range(TPEs + 1):
            ret, frame = cap.read()
            if not ret:
                cap.release()
                del pool
                exit(-1)
            pool.put(frame)
    
    
    while (cap.isOpened()):
        start_time = time.time()
        ret, frame = cap.read()

        if not ret:
            break
        pool.put(frame)
        child_conn2.send(frame)
        frame, flag = pool.get()
        box = parent_conn.recv()  # Receive coordinates from csrt_tracker process
        if not dict_["state_2"]:
            img_center = lib_start.get_center(frame, 0, 0, frame.shape[1], frame.shape[0])
            lib_start.image_process(frame, box, img_center)        
        
        end_time = time.time()
        seconds = end_time - start_time
        fps = 1.0 / seconds
        
        cv2.putText(frame, f"{int(fps)} fps", (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,0,255),2) #cv2.FONT_HERSHEY_COMPLEX
        # Если начальные и конечные координаты прямоугольника определены
        if lib_start.start_x != -1 and lib_start.end_x != -1:
            if lib_start.state != 0:
                # Рисование прямоугольника на изображении
                cv2.rectangle(frame, (lib_start.start_x, lib_start.start_y), (lib_start.end_x, lib_start.end_y), (0, 255, 0), 2)
                dict_["bbox"] = lib_start.bbox
                if dict_["state_2"]:
                    dict_["state"] = lib_start.state
                else:
                    lib_start.state = dict_["state"]
        cv2.imshow("win", frame)
        if cv2.waitKey(1) & 0xff == ord('q'):
            break    
    
    p.join()
    cap.release()
    cv2.destroyAllWindows()



#### Рабочий концепт

"""
def cv_tracker(pipe):
    tracker = cv2.TrackerCSRT_create()
    cap = cv2.VideoCapture(1)

    while True:
        _, frame = cap.read()
        x, y, w, h = cv2.selectROI("Select Object to Track", frame)

        tracker.init(frame, (x, y, w, h))

        while True:
            start_time = time.time()
            success, frame = cap.read()

            if not success:
                break

            _, box = tracker.update(frame)

            pipe.send(box)
            end_time = time.time()
            seconds = end_time - start_time
            fps = 1.0 / seconds
            cv2.putText(frame, f"{int(fps)} fps", (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,0,255),2) #cv2.FONT_HERSHEY_COMPLEX
            cv2.rectangle(frame, (int(box[0]), int(box[1])), (int(box[0]+box[2]), int(box[1]+box[3])), (255,0,0), 2)
            cv2.imshow("CSRT Tracker", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()

def yolo_intersection(pipe):
    while True:
        start_time = time.time()
        box = pipe.recv()  # Receive coordinates from csrt_tracker process
        print (box)
        end_time = time.time()
        seconds = end_time - start_time
        fps = 1.0 / seconds
        
        print (fps)
        
        # Perform Intersection over Union with YOLO detections

if __name__ == "__main__":
    parent_conn, child_conn = multiprocessing.Pipe()

    csrt_process = multiprocessing.Process(target=cv_tracker, args=(child_conn,))
    yolo_process = multiprocessing.Process(target=yolo_intersection, args=(parent_conn,))

    csrt_process.start()
    yolo_process.start()

    csrt_process.join()
    yolo_process.join()
"""    

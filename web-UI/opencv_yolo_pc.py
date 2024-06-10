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
from threading import Thread
from ultralytics import YOLO
import multiprocessing
import torch
from ultralytics.utils.plotting import Annotator

# Model
model = YOLO('yolov8n.pt')
#export LD_LIBRARY_PATH=${PWD}/.venv/lib64/python3.11/site-packages/nvidia/cublas/lib:${PWD}/.venv/lib64/python3.11/site-packages/nvidia/cudnn/lib
 
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

    cap = cv2.VideoCapture(0)
    while (cap.isOpened()):
        start_time = time.time()
        ret, frame = cap.read()

        if not ret:
            break
        child_conn2.send(frame)
        
        results = model(frame)
        for r in results:
            annotator = Annotator(frame)
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0]  # get box coordinates in (left, top, right, bottom) format
                c = box.cls
                annotator.box_label(b, model.names[int(c)])
        frame = annotator.result()
        
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

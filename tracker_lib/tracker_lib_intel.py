from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import os
import sys
import cv2
import time
import argparse
import numpy as np
from glob import glob
from scipy.spatial import distance
from collections import deque
import concurrent
from concurrent.futures import ThreadPoolExecutor
import logging
import io
from contextlib import redirect_stdout, redirect_stderr

os.environ["XDG_SESSION_TYPE"] = "xcb"
os.environ["QT_QPA_PLATFORM"] = "xcb"

class TrackerLib(object):
    def __init__(self):
        # Флаг для отслеживания режима рисования прямоугольника
        self.drawing = False
        # Координаты начала и конца прямоугольника
        self.start_x, self.start_y = -1, -1
        self.end_x, self.end_y = -1, -1
        # Our ROI, defined by two points
        self.p1, self.p2 = None, None
        self.state = 0
        self.init_switch = False
        self.bbox = [0, 0, 0, 0]
        self.last_bbox = [0, 0, 0, 0]
        self.Error_track = False
        self.dst = 0
        self.obj_center = [0,0]
        self.lost_object_counter = 0
        self.recent_positions = deque(maxlen=10)
        self.max_lost_frames = 30
        self.trackers = {}
        self.tracker_weights = {'csrt': 0.6, 'kcf': 0.4}
        


    # Функция для рисования прямоугольника-обработчик событий мыши
    def draw_rectangle(self, event, x, y, flags, userdata):
        # Если происходит нажатие левой кнопки мыши
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.start_x, self.start_y = x, y
            self.end_x, self.end_y = x, y
            self.p1 = [x,y]
            self.state += 1
            self.init_switch = False
        # Если происходит движение мыши с нажатой кнопкой
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.drawing:
                self.end_x, self.end_y = x, y
        
        # Если кнопка мыши отпущена
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            self.end_x, self.end_y = x, y
            #self.p2 = [x,y]
            self.state += 1
            
            if self.p1[0] > x:
                self.end_x = self.p1[0]
                self.p1[0] = x
            else:
                 self.end_x = x   
            
            if self.p1[1] > y:
                self.end_y = self.p1[1]
                self.p1[1] = y
            else:
                self.end_y = y
            self.p2 = [self.end_x, self.end_y]
            self.bbox = (self.p1[0], self.p1[1], self.p2[0]-self.p1[0], self.p2[1]-self.p1[1])
            #self.init_switch = True
            
    def get_center(self, img, x, y, w, h):
        xcentr = int(x+(w/2))
        ycentr = int(y+(h/2))
        cv2.circle(img, (xcentr, ycentr), radius=0, color=(0, 0, 255), thickness=5)
        return (xcentr, ycentr)    


    def draw_box(self, img, bbox, color_border_box = (255, 0, 255)):
        x, y, w, h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
        cv2.rectangle(img, (x, y), ((x+w), (y+h)), color_border_box, 3, 1)
        return self.get_center(img, x, y, w, h)


    def increase_brightness(self, img, value=10):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        v = np.clip(v.astype(np.int32) + value, 0, 255).astype(np.uint8)
        final_hsv = cv2.merge((h, s, v))
        return cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    
    def create_win(self):
        # Register the mouse callback
        cv2.namedWindow('win')#, cv2.WINDOW_NORMAL)  
        cv2.setMouseCallback('win', self.draw_rectangle)   
    

    def image_process(self, img, bbox, img_center, color_border_box=(255, 0, 255)):
        self.obj_center = self.draw_box(img, bbox, color_border_box)
        distance = np.linalg.norm(np.array(self.obj_center) - np.array(img_center))
        cv2.line(img, img_center, self.obj_center, (255, 0, 0), 4)
        cv2.putText(img, f"{int(distance)}", (bbox[0], bbox[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        self.recent_positions.append(self.obj_center)      

        
    def init_tracker(self, img, bbox):
        self.state = 0
        self.trackers['csrt'] = cv2.TrackerCSRT_create()
        self.trackers['csrt'].init(img, bbox)
        self.trackers['kcf'] = cv2.TrackerKCF_create()
        self.trackers['kcf'].init(img, bbox)
        self.init_switch = True



    def aim_visual(self, img, size_box=50, corner_size=20, line_thickness=3, color=(1, 152, 117), full=False):
        height, width = img.shape[:2]
        center_y, center_x = height // 2, width // 2
        top = center_y - size_box
        bottom = center_y + size_box
        left = center_x - size_box
        right = center_x + size_box

        if full:
            # Рисуем полный прямоугольник
            cv2.rectangle(img, (left, top), (right, bottom), color, line_thickness)
            cv2.rectangle(img, (left, top), (right, bottom), (255,0,0), 1)
        else:
            # Рисуем углы прямоугольника
            corners = [ (left, top),  # Левый верхний
                        (right, top),  # Правый верхний
                        (right, bottom),  # Правый нижний
                        (left, bottom)  # Левый нижний
                      ]

            for x, y in corners:
                if x == left:
                    cv2.line(img, (x, y), (x + corner_size, y), color, line_thickness)
                else:
                    cv2.line(img, (x, y), (x - corner_size, y), color, line_thickness)

                if y == top:
                    cv2.line(img, (x, y), (x, y + corner_size), color, line_thickness)
                else:
                    cv2.line(img, (x, y), (x, y - corner_size), color, line_thickness)

        return img

    def process_img_server(self, img, init_tracker):
        img_center = self.get_center(img, 0, 0, img.shape[1], img.shape[0])
        if self.init_switch or init_tracker:
            bbox_results = {}
            with ThreadPoolExecutor() as executor:
                future_to_tracker = {executor.submit(tracker.update, img): name for name, tracker in self.trackers.items()}
                for future in concurrent.futures.as_completed(future_to_tracker):
                    tracker_name = future_to_tracker[future]
                    success, bbox = future.result()
                    
                    if success:
                        bbox_results[tracker_name] = bbox

            if bbox_results:
                weighted_bbox = np.average([bbox for bbox in bbox_results.values()], 
                                           axis=0, 
                                           weights=[self.tracker_weights[name] for name in bbox_results.keys()])
                bbox = [int(x) for x in weighted_bbox]
                self.dst = distance.euclidean(self.last_bbox, bbox)
                self.last_bbox = bbox
                self.image_process(img, bbox, img_center)
                self.lost_object_counter = 0
            else:
                self.lost_object_counter += 1
                if self.lost_object_counter >= self.max_lost_frames:
                    self.reinitialize_trackers(img)

        return img, self.obj_center, img_center


    def reinitialize_trackers(self, img):
        if self.recent_positions:
            last_known_position = self.recent_positions[-1]
            search_area = self.expand_search_area(last_known_position, img.shape)
            roi = img[search_area[1]:search_area[3], search_area[0]:search_area[2]]
            
            # Use a simple feature matching or template matching here
            # For simplicity, let's use template matching
            result = cv2.matchTemplate(roi, img[self.last_bbox[1]:self.last_bbox[1]+self.last_bbox[3], 
                                            self.last_bbox[0]:self.last_bbox[0]+self.last_bbox[2]], 
                                       cv2.TM_CCOEFF_NORMED)
            _, _, _, max_loc = cv2.minMaxLoc(result)
            
            new_bbox = (search_area[0] + max_loc[0], search_area[1] + max_loc[1], 
                        self.last_bbox[2], self.last_bbox[3])
            
            self.init_tracker(img, new_bbox)
            self.lost_object_counter = 0

    def expand_search_area(self, center, img_shape, factor=1.5):
        x, y = center
        w, h = self.last_bbox[2:]
        x1 = max(0, int(x - w*factor/2))
        y1 = max(0, int(y - h*factor/2))
        x2 = min(img_shape[1], int(x + w*factor/2))
        y2 = min(img_shape[0], int(y + h*factor/2))
        return (x1, y1, x2, y2)

    def start_stream(self, id_cma=0):
        self.cap = cv2.VideoCapture(id_cma)
        while self.cap.isOpened():
            start_time = time.time()
            success, img = self.cap.read()
            print (img.shape)
            if not success:
                break

            img_center = self.get_center(img, 0, 0, img.shape[1], img.shape[0])
            
            if self.state > 1 and sum(self.bbox[-2:]) > 10:
                cv2.rectangle(img, self.bbox, (255, 0, 0), 10)
                self.init_tracker(img, self.bbox)

            if self.init_switch:
                img, obj_center, img_center = self.process_img_server(img, False)

            fps = 1.0 / (time.time() - start_time)
            cv2.putText(img, f"{int(fps)} fps", (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

            if self.start_x != -1 and self.end_x != -1 and self.state != 0:
                cv2.rectangle(img, (self.start_x, self.start_y), (self.end_x, self.end_y), (0, 255, 0), 2)

            cv2.imshow("win", img)
            if cv2.waitKey(1) & 0xff == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    print ("START")
    lib_start = TrackerLib()
    lib_start.create_win()
    lib_start.start_stream(id_cma=0)

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


# NanoTrack
sys.path.append(os.getcwd())
from tracker_lib.NanoTrack.core.config import cfg
from tracker_lib.NanoTrack.models.rknnlite_rk3588_tracker import NnoTracker_RKNNLite

parser = argparse.ArgumentParser(description='tracking demo')
parser.add_argument('--config', default='./tracker_lib/NanoTrack/models/config/config.yaml', type=str, help='config file')
parser.add_argument('--save', action='store_true', help='whether visualzie result')
args = parser.parse_args()


# YOLO + SORT
from tracker_lib.sort_yolov5_python.rknnpool import rknnPoolExecutor
from tracker_lib.sort_yolov5_python.func import myFunc2, draw2, myFunc, draw


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
        v = cv2.add(v,value)
        v[v > 255] = 255
        v[v < 0] = 0
        final_hsv = cv2.merge((h, s, v))
        img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
        return img
    
    def create_win(self):
        # Register the mouse callback
        cv2.namedWindow('win')#, cv2.WINDOW_NORMAL)  
        cv2.setMouseCallback('win', self.draw_rectangle)   
    

    def image_process(self, img, bbox, img_center, color_border_box = (255, 0, 255)):
        self.obj_center = self.draw_box(img, bbox, color_border_box)
        x_dist = (self.obj_center[0] - img_center[0])**2
        y_dist = (self.obj_center[1] - img_center[1])**2 

        cv2.line(img, img_center, self.obj_center, (255,0,0), 4) 
        cv2.putText(img, "{}".format(int(np.sqrt(x_dist + y_dist))), (bbox[0],bbox[1]),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        #img_center
        w_sector, h_sector = img.shape[1]//2, img.shape[0]//2
        M = int(np.sqrt(w_sector**2 + h_sector**2))
        g = int(np.sqrt(x_dist + y_dist))
        p_dist = (g/M) * 100


        M1 = int(np.sqrt(img.shape[1]**2 + img.shape[0]**2))
        g1_point = int(np.sqrt(bbox[0]**2 + bbox[1]**2))
        p_dist_point = (g1_point/M1) * 100


        x_, y_ = bbox[0]+bbox[2], bbox[1]+bbox[3]
        x_, y_ = img.shape[1]-x_, img.shape[0]-y_


        g2_point = int(np.sqrt(x_**2 + y_**2))
        p_dist_point2 = (g2_point/M1) * 100  

        A = (bbox[0]+bbox[2])+x_//2 
        B = (bbox[1]+bbox[3])+y_//2 

        distance = (p_dist+p_dist_point+p_dist_point2)/3        


    def start_stream(self, id_cma = 1):
        self.cap = cv2.VideoCapture(id_cma)
        # ROI in video
        while self.cap.isOpened():
            # FPS варианты
            start_time = time.time()
#            timer = cv2.getTickCount()
            success, img = self.cap.read()
            #img = self.increase_brightness(img)
            #img = cv2.flip(img, 1)
            img_center = self.get_center(img, 0, 0, img.shape[1], img.shape[0])
            if self.state > 1:
                if sum(self.bbox[-2:]) > 10:
                    cv2.rectangle(img, self.bbox, (255, 0, 0), 10)
                    self.init_tracker(img, self.bbox, A = True, B = True)  
                
            if self.init_switch:
                # Обновление трекера CSRT
                csrt_success, csrt_bbox = self.csrt_tracker.update(img)
                
                # Обновление трекера KCF
                kcf_success, kcf_bbox = self.kcf_tracker.update(img)
                if float(self.dst)>7.0:
                    if self.Error_track == "A":
                        self.init_tracker(img, self.last_bbox, A=True)
                        print (float(self.dst), self.Error_track)
                    if self.Error_track == "B":
                        self.init_tracker(img, self.last_bbox, B=True)
                        print (float(self.dst), self.Error_track)
                      

                # Взвешивание результатов трекинга
                if csrt_success and kcf_success:                
                    bbox = (0.6 * csrt_bbox[0] + 0.4 * kcf_bbox[0],
                             0.6 * csrt_bbox[1] + 0.4 * kcf_bbox[1],
                             0.6 * csrt_bbox[2] + 0.4 * kcf_bbox[2],
                             0.6 * csrt_bbox[3] + 0.4 * kcf_bbox[3])

                    self.dst = distance.euclidean(self.last_bbox, bbox)
                    bbox = [int(x) for x in bbox]
                    self.last_bbox = bbox
                    self.image_process(img, bbox, img_center)
                    self.Error_track = "A+B"
                
                elif csrt_success:
                    bbox = csrt_bbox 
                    self.dst = distance.euclidean(self.last_bbox, bbox)
                    self.image_process(img, bbox, img_center)
                    self.last_bbox = bbox
                    self.Error_track = "A"
                elif kcf_success:     
                    bbox = kcf_bbox 
                    self.dst = distance.euclidean(self.last_bbox, bbox)
                    self.image_process(img, bbox, img_center)
                    self.last_bbox = bbox
                    self.Error_track = "B"
                else:
                    if self.Error_track == False:
                        #self.Error_track = True
                        # self.init_tracker(img, self.last_bbox)
                        self.image_process(img, self.last_bbox, img_center)
                        print ("TrackerLib Error")

            # FPS варианты
            #fps = cv2.getTickFrequency()/(cv2.getTickCount()-timer)
            end_time = time.time()
            seconds = end_time - start_time
            fps = 1.0 / seconds
            
            cv2.putText(img, f"{int(fps)} fps", (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,0,255),2) #cv2.FONT_HERSHEY_COMPLEX
            
            # Если начальные и конечные координаты прямоугольника определены
            if self.start_x != -1 and self.end_x != -1:
                if self.state != 0:
                    # Рисование прямоугольника на изображении
                    cv2.rectangle(img, (self.start_x, self.start_y), (self.end_x, self.end_y), (0, 255, 0), 2)
            cv2.imshow("win", img)
            if cv2.waitKey(1) & 0xff == ord('q'):
                break    
        self.cap
        cv2.destroyAllWindows()
        
    def init_tracker(self, img, bbox, A = False, B = False):
        self.state = 0 
       
        if B:
            self.csrt_tracker = cv2.TrackerCSRT_create()
            self.csrt_tracker.init(img, bbox) 
        if A:
            self.kcf_tracker = cv2.TrackerKCF_create()
            self.kcf_tracker.init(img, bbox)
        self.init_switch = True
    
    def init_yolo(self):
        modelPath = "/home/orangepi/above-ground/tracker_lib/sort_yolov5_python/rknnModel/yolov5s_relu_tk2_RK3588_i8.rknn"
        # Увеличьте количество линий, увеличьте скорость
        self.TPEs = 3
        self.pool = rknnPoolExecutor(
                                    rknnModel=modelPath,
                                    TPEs=self.TPEs,
                                    # func=myFunc) # YOLO
                                    func=myFunc2) # YOLO + SORT
    def init_NanoTrack(self, img, bbox):
        # load config
        cfg.merge_from_file(args.config)

        # load_weight
        Tback_weight = './tracker_lib/NanoTrack/weights/track_backbone_T.rknn'
        Xback_weight = './tracker_lib/NanoTrack/weights/track_backbone_X.rknn'
        Head_weight = './tracker_lib/NanoTrack/weights/head.rknn'

        self.NanoTracker = NnoTracker_RKNNLite(Tback_weight, Xback_weight, Head_weight)   
        self.NanoTracker.init(img, bbox)

                                   
    def process_img_server(self, img, init_tracker):
        img_center = self.get_center(img, 0, 0, img.shape[1], img.shape[0])
        # отправить в поток NPU (yolo)
        self.pool.put(img)
        if self.init_switch == True or init_tracker == True:
            # Обновление трекера CSRT
            csrt_success, csrt_bbox = self.csrt_tracker.update(img)
            
            # Обновление трекера KCF
            kcf_success, kcf_bbox = self.kcf_tracker.update(img)
            
            # получить из поток NPU (yolo)
            boxes_yolo, flag = self.pool.get()
            # Взвешивание результатов трекинга
            if csrt_success and kcf_success:                
                bbox = (0.6 * csrt_bbox[0] + 0.4 * kcf_bbox[0],
                         0.6 * csrt_bbox[1] + 0.4 * kcf_bbox[1],
                         0.6 * csrt_bbox[2] + 0.4 * kcf_bbox[2],
                         0.6 * csrt_bbox[3] + 0.4 * kcf_bbox[3])
                self.dst = distance.euclidean(self.last_bbox, bbox)
                bbox = [int(x) for x in bbox]
                self.last_bbox = bbox
                self.image_process(img, bbox, img_center)
                self.Error_track = "A+B"
                                
            elif csrt_success:
                bbox = csrt_bbox 
                self.dst = distance.euclidean(self.last_bbox, bbox)
                self.image_process(img, bbox, img_center)
                self.last_bbox = bbox
                self.Error_track = "A"
            elif kcf_success:     
                bbox = kcf_bbox 
                self.dst = distance.euclidean(self.last_bbox, bbox)
                self.image_process(img, bbox, img_center)
                self.last_bbox = bbox
                self.Error_track = "A"
            # yolo мультипоточность 
            # получать сдесь
        
            
            if boxes_yolo is not None:
               # draw(img, boxes_yolo[0], boxes_yolo[1], boxes_yolo[2]) # YOLO  
               draw2(img, boxes_yolo[:,:-1], boxes_yolo[:,-1]) # YOLO + SORT
                 
        #self.state = 0
        return img, self.obj_center, img_center

    # прицел
    def aim_visual(self, img, 
                   size_box = 70,
                   _size = 30,
                   line_box = 3,
                   color = [1,152,117],
                   full = False):
        shape_image = img.shape
        
        size_0 = int(shape_image[0]/2+size_box)
        size_1 = int(shape_image[0]/2-size_box)
        
        size_2 = int(shape_image[1]/2+size_box)
        size_3 = int(shape_image[1]/2-size_box)
        area_OIU = [shape_image[0]/2-size_box, shape_image[1]/2-size_box, shape_image[0]/2+size_box, shape_image[1]/2+size_box]
        area_OIU = [int(d) for d in area_OIU]
        if full:
            # Прямоугольник сплошные линии
            img[size_0:size_0+line_box,size_2-(size_box*2):size_2,:] = color
            img[size_0-(size_box*2):size_0, size_2:size_2+line_box,:] = color
            img[size_1:size_1+line_box, size_3:size_3+(size_box*2),:] = color # x line top
            img[size_0-(size_box*2):size_0, size_3:size_3+line_box,:] = color # y line left
            cv2.rectangle(img, (area_OIU[1], area_OIU[0]), (area_OIU[3],area_OIU[2]), color=(255,0,0), thickness=1)
        else:
            # Прямоугольник контуры углов
            
            # левый верхний угол
            img[size_1:size_1+line_box, size_3:size_3+_size,:] = color
            img[size_0-(size_box*2):size_0-(size_box*2)+_size, size_3:size_3+line_box,:] = color
            # правый верхний угол
            img[size_0-(size_box*2):size_0-(size_box*2)+_size, size_2:size_2+line_box,:] = color
            img[size_1:size_1+line_box, size_3+(size_box*2)-_size:size_3+(size_box*2),:] = color
            # правый нижний угол
            img[size_0-_size:size_0, size_2:size_2+line_box,:] = color
            img[size_0:size_0+line_box, size_2-_size:size_2,:] = color
            # левый нижний угол
            img[size_0:size_0+line_box, size_2-(size_box*2):size_2-(size_box*2)+_size,:] = color
            img[size_0-_size:size_0, size_3:size_3+line_box,:] = color
        return img


    def process_img_server_NanoTrack(self, img, init_tracker):
        #print (img.shape)
        img_center = self.get_center(img, 0, 0, img.shape[1], img.shape[0])
        if self.init_switch == True or init_tracker == True:
            # Обновление трекера CSRT
            csrt_success, csrt_bbox = self.csrt_tracker.update(img)
            
            # Обновление трекера KCF
            kcf_success, kcf_bbox = self.kcf_tracker.update(img)
            
            # получить из поток NPU (NanoTrack)
            outputs_NanoTrack = self.NanoTracker.track(img)
                
            if 'polygon' in outputs_NanoTrack:
                polygon = np.array(outputs_NanoTrack['polygon']).astype(np.int32)
                cv2.polylines(img, [polygon.reshape((-1, 1, 2))],
                              True, (0, 255, 0), 3)
                mask = ((outputs_NanoTrack['mask'] > cfg.TRACK.MASK_THERSHOLD) * 255)
                mask = mask.astype(np.uint8)
                mask = np.stack([mask, mask * 255, mask]).transpose(1, 2, 0)
                frame = cv2.addWeighted(img, 0.77, mask, 0.23, -1)
            else:
                bbox = list(map(int, outputs_NanoTrack['bbox']))
                self.image_process(img, bbox, img_center, color_border_box = (255, 0, 35))           
                #self.last_bbox = bbox

            # Взвешивание результатов трекинга
            if csrt_success and kcf_success:                
                bbox = (0.6 * csrt_bbox[0] + 0.4 * kcf_bbox[0],
                         0.6 * csrt_bbox[1] + 0.4 * kcf_bbox[1],
                         0.6 * csrt_bbox[2] + 0.4 * kcf_bbox[2],
                         0.6 * csrt_bbox[3] + 0.4 * kcf_bbox[3])
                self.dst = distance.euclidean(self.last_bbox, bbox)
                bbox = [int(x) for x in bbox]
                self.last_bbox = bbox
                self.image_process(img, bbox, img_center)
                self.Error_track = "A+B"
                                
            elif csrt_success:
                bbox = csrt_bbox 
                self.dst = distance.euclidean(self.last_bbox, bbox)
                self.image_process(img, bbox, img_center)
                self.last_bbox = bbox
                self.Error_track = "A"
            elif kcf_success:     
                bbox = kcf_bbox 
                self.dst = distance.euclidean(self.last_bbox, bbox)
                self.image_process(img, bbox, img_center)
                self.last_bbox = bbox
                self.Error_track = "A"


        #self.state = 0
        return img, self.obj_center, img_center


    def start_stream_noTracker(self, id_cma = 1):
        # load config
        cfg.merge_from_file(args.config)

        # load_weight
        Tback_weight = './NanoTrack/weights/track_backbone_T.rknn'
        Xback_weight = './NanoTrack/weights/track_backbone_X.rknn'
        Head_weight = './NanoTrack/weights/head.rknn'

        tracker = NnoTracker_RKNNLite(Tback_weight, Xback_weight, Head_weight)
        first_frame = True
 
        self.cap = cv2.VideoCapture(id_cma)
        # ROI in video
        while self.cap.isOpened():
            # FPS варианты
            start_time = time.time()
#            timer = cv2.getTickCount()
            success, img = self.cap.read()
            #img = self.increase_brightness(img)
            #img = cv2.flip(img, 1)
            img_center = self.get_center(img, 0, 0, img.shape[1], img.shape[0])
            if self.state > 1:
                if sum(self.bbox[-2:]) > 10:
                    if first_frame:
                        cv2.rectangle(img, self.bbox, (255, 0, 0), 10)
                        tracker.init(img, self.bbox)
                        first_frame = False
                        self.init_switch = True
                        self.state = False
            if self.init_switch:
                outputs = tracker.track(img)
                if 'polygon' in outputs:
                    polygon = np.array(outputs['polygon']).astype(np.int32)
                    cv2.polylines(img, [polygon.reshape((-1, 1, 2))],
                                  True, (0, 255, 0), 3)
                    mask = ((outputs['mask'] > cfg.TRACK.MASK_THERSHOLD) * 255)
                    mask = mask.astype(np.uint8)
                    mask = np.stack([mask, mask * 255, mask]).transpose(1, 2, 0)
                    frame = cv2.addWeighted(img, 0.77, mask, 0.23, -1)
                else:
                    bbox = list(map(int, outputs['bbox']))
                    self.image_process(img, bbox, img_center, color_border_box = (255, 0, 35))
     
            # FPS варианты
            #fps = cv2.getTickFrequency()/(cv2.getTickCount()-timer)
            end_time = time.time()
            seconds = end_time - start_time
            fps = 1.0 / seconds
            
            cv2.putText(img, f"{int(fps)} fps", (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,0,255),2) #cv2.FONT_HERSHEY_COMPLEX
            
            # Если начальные и конечные координаты прямоугольника определены
            if self.start_x != -1 and self.end_x != -1:
                if self.state != 0:
                    # Рисование прямоугольника на изображении
                    cv2.rectangle(img, (self.start_x, self.start_y), (self.end_x, self.end_y), (0, 255, 0), 2)
            cv2.imshow("win", img)
            if cv2.waitKey(1) & 0xff == ord('q'):
                break    
        self.cap
        cv2.destroyAllWindows()



if __name__ == "__main__":
    print ("START")
    lib_start = TrackerLib()
    lib_start.create_win()
    # lib_start.start_stream(id_cma=0)
    lib_start.start_stream_noTracker(id_cma=0)

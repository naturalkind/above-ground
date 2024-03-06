import  cv2
import time
import numpy as np

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


    def draw_box(self, img, bbox):
        x, y, w, h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
        cv2.rectangle(img, (x, y), ((x+w), (y+h)), (255, 0, 255), 3, 1)
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
        
    def start_stream(self):
        self.cap = cv2.VideoCapture(1)
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
                    self.state = 0
                    self.csrt_tracker = cv2.TrackerCSRT_create()
                    self.kcf_tracker = cv2.TrackerKCF_create()
                    self.csrt_tracker.init(img, self.bbox) 
                    self.kcf_tracker.init(img, self.bbox) 
                    self.init_switch = True
                
            if self.init_switch:
                # Обновление трекера CSRT
                csrt_success, csrt_bbox = self.csrt_tracker.update(img)
                
                # Обновление трекера KCF
                kcf_success, kcf_bbox = self.kcf_tracker.update(img)
                
                # Взвешивание результатов трекинга
                if csrt_success and kcf_success:                
                    bbox = (0.6 * csrt_bbox[0] + 0.4 * kcf_bbox[0],
                             0.6 * csrt_bbox[1] + 0.4 * kcf_bbox[1],
                             0.6 * csrt_bbox[2] + 0.4 * kcf_bbox[2],
                             0.6 * csrt_bbox[3] + 0.4 * kcf_bbox[3])
                    bbox = [int(x) for x in bbox]
                    obj_center = self.draw_box(img, bbox)
                    x_dist = (obj_center[0] - img_center[0])**2
                    y_dist = (obj_center[1] - img_center[1])**2 
                    cv2.line(img, img_center, obj_center, (255,0,0), 4) 
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
                                        
                elif csrt_success:
                    bbox = csrt_bbox 
                    obj_center = self.draw_box(img, bbox)
                    x_dist = (obj_center[0] - img_center[0])**2
                    y_dist = (obj_center[1] - img_center[1])**2 
                    
                    cv2.line(img, img_center, obj_center, (255,0,0), 4) 
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
                elif kcf_success:     
                    bbox = kcf_bbox 
                    obj_center = self.draw_box(img, bbox)
                    x_dist = (obj_center[0] - img_center[0])**2
                    y_dist = (obj_center[1] - img_center[1])**2 
                    
                    cv2.line(img, img_center, obj_center, (255,0,0), 4) 
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
        
    def init_tracker(self, img, bbox):
        self.state = 0 
        self.csrt_tracker = cv2.TrackerCSRT_create()
        self.kcf_tracker = cv2.TrackerKCF_create()
        self.csrt_tracker.init(img, bbox) 
        self.kcf_tracker.init(img, bbox)
        self.init_switch = True
        


    def process_img_server(self, img, init_tracker):
        img_center = self.get_center(img, 0, 0, img.shape[1], img.shape[0])
        obj_center = [0,0]
        if self.init_switch == True or init_tracker == True:
            # Обновление трекера CSRT
            csrt_success, csrt_bbox = self.csrt_tracker.update(img)
            
            # Обновление трекера KCF
            kcf_success, kcf_bbox = self.kcf_tracker.update(img)
            
            # Взвешивание результатов трекинга
            if csrt_success and kcf_success:                
                bbox = (0.6 * csrt_bbox[0] + 0.4 * kcf_bbox[0],
                         0.6 * csrt_bbox[1] + 0.4 * kcf_bbox[1],
                         0.6 * csrt_bbox[2] + 0.4 * kcf_bbox[2],
                         0.6 * csrt_bbox[3] + 0.4 * kcf_bbox[3])
                bbox = [int(x) for x in bbox]
                obj_center = self.draw_box(img, bbox)
                x_dist = (obj_center[0] - img_center[0])**2
                y_dist = (obj_center[1] - img_center[1])**2 
                cv2.line(img, img_center, obj_center, (255,0,0), 4) 
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
                                    
            elif csrt_success:
                bbox = csrt_bbox 
                obj_center = self.draw_box(img, bbox)
                x_dist = (obj_center[0] - img_center[0])**2
                y_dist = (obj_center[1] - img_center[1])**2 
                
                cv2.line(img, img_center, obj_center, (255,0,0), 4) 
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
            elif kcf_success:     
                bbox = kcf_bbox 
                obj_center = self.draw_box(img, bbox)
                x_dist = (obj_center[0] - img_center[0])**2
                y_dist = (obj_center[1] - img_center[1])**2 
                
                cv2.line(img, img_center, obj_center, (255,0,0), 4) 
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
        #self.state = 0
        return img, obj_center, img_center

    def stream_from_ue(self, img):
        start_time = time.time()
        img_center = self.get_center(img, 0, 0, img.shape[1], img.shape[0])
        obj_center = [0,0]
        if self.state > 1:
            if sum(self.bbox[-2:]) > 10:
                cv2.rectangle(img, self.bbox, (255, 0, 0), 10)  
                self.state = 0
                self.csrt_tracker = cv2.TrackerCSRT_create()
                self.kcf_tracker = cv2.TrackerKCF_create()
                self.csrt_tracker.init(img, self.bbox) 
                self.kcf_tracker.init(img, self.bbox) 
                self.init_switch = True
            
        if self.init_switch:
            # Обновление трекера CSRT
            csrt_success, csrt_bbox = self.csrt_tracker.update(img)
            
            # Обновление трекера KCF
            kcf_success, kcf_bbox = self.kcf_tracker.update(img)
            
            # Взвешивание результатов трекинга
            if csrt_success and kcf_success:                
                bbox = (0.6 * csrt_bbox[0] + 0.4 * kcf_bbox[0],
                         0.6 * csrt_bbox[1] + 0.4 * kcf_bbox[1],
                         0.6 * csrt_bbox[2] + 0.4 * kcf_bbox[2],
                         0.6 * csrt_bbox[3] + 0.4 * kcf_bbox[3])
                bbox = [int(x) for x in bbox]
                obj_center = self.draw_box(img, bbox)
                x_dist = (obj_center[0] - img_center[0])**2
                y_dist = (obj_center[1] - img_center[1])**2 
                cv2.line(img, img_center, obj_center, (255,0,0), 4) 
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
                                    
            elif csrt_success:
                bbox = csrt_bbox 
                obj_center = self.draw_box(img, bbox)
                x_dist = (obj_center[0] - img_center[0])**2
                y_dist = (obj_center[1] - img_center[1])**2 
                
                cv2.line(img, img_center, obj_center, (255,0,0), 4) 
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
            elif kcf_success:     
                bbox = kcf_bbox 
                obj_center = self.draw_box(img, bbox)
                x_dist = (obj_center[0] - img_center[0])**2
                y_dist = (obj_center[1] - img_center[1])**2 
                
                cv2.line(img, img_center, obj_center, (255,0,0), 4) 
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
        return img, obj_center, img_center

if __name__ == "__main__":
    print ("START")
    lib_start = TrackerLib()
    lib_start.create_win()
    lib_start.start_stream()
    

import  cv2
import time
import numpy as np

# Калибровка
class TrackerLib(object):
    def __init__(self):
        print ("INIT")

##        self.cap = cv2.VideoCapture(0)
##        # ROI in video
##        cv2.namedWindow('win')#, cv2.WINDOW_NORMAL)

        # Флаг для отслеживания режима рисования прямоугольника
        self.drawing = False
        # Координаты начала и конца прямоугольника
        self.start_x, self.start_y = -1, -1
        self.end_x, self.end_y = -1, -1
        # Our ROI, defined by two points
        self.p1, self.p2 = None, None
        self.state = 0
        self.init_switch = False
        self.calibration_state = 0
        self.focalLength = 0       
##        csrt_tracker = cv2.TrackerCSRT_create()
##        # Register the mouse callback
##        cv2.namedWindow('win')#, cv2.WINDOW_NORMAL)  
##        cv2.setMouseCallback('win', self.on_mouse)

        self.csrt_tracker = cv2.TrackerCSRT_create()
        self.kcf_tracker = cv2.TrackerKCF_create()
        

    # Найти целевую функцию
    def find_marker(self, image):
        # convert the image to grayscale, blur it, and detect edges
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  
        gray = cv2.GaussianBlur(gray, (5, 5), 0)        
        edged = cv2.Canny(gray, 35, 125)               
     
        # find the contours in the edged image and keep the largest one;
        # we'll assume that this is our piece of paper in the image
        (cnts, _) = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)  
         # Найти наибольшую площадь 
        c = max(cnts, key = cv2.contourArea)
     
        return cv2.minAreaRect(c)
 

    # Функция расчета расстояния 
    def focal_length(self, measured_distance, real_width, width_in_rf_image):
        """
        This Function Calculate the Focal Length(distance between lens to CMOS sensor), it is simple constant we can find by using
        MEASURED_DISTACE, REAL_WIDTH(Actual width of object) and WIDTH_OF_OBJECT_IN_IMAGE
        :param1 Measure_Distance(int): It is distance measured from object to the Camera while Capturing Reference image

        :param2 Real_Width(int): It is Actual width of object, in real world (like My face width is = 14.3 centimeters)
        :param3 Width_In_Image(int): It is object width in the frame /image in our case in the reference image(found by Face detector)
        :retrun focal_length(Float):"""
        focal_length_value = (width_in_rf_image * measured_distance) / real_width
        return focal_length_value 
 

    def on_mouse(self, event, x, y, flags, userdata):
##        global state, p1, p2, calibration_state
        #print (event)
        # Calibration
        if event == cv2.EVENT_MBUTTONDOWN:
            pass
        
        # Left click
        if event == cv2.EVENT_LBUTTONUP:
            # Select first point
            if self.state == 0:
                self.p1 = (x,y)
                self.state += 1
            # Select second point
            elif self.state == 1:
                self.p2 = (x,y)
                self.state += 1
                
                self.calibration_state = 1
        # Right click (erase current ROI)
        if event == cv2.EVENT_RBUTTONUP:
            self.p1, self.p2 = None, None
            self.state = 0

    # Функция для рисования прямоугольника-обработчик событий мыши
    def draw_rectangle(self, event, x, y, flags, userdata):
#        global drawing, start_x, start_y, end_x, end_y
        
        # Если происходит нажатие левой кнопки мыши
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.start_x, self.start_y = x, y
            self.end_x, self.end_y = x, y
            self.p1 = (x,y)
            self.state += 1
        # Если происходит движение мыши с нажатой кнопкой
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.drawing:
                self.end_x, self.end_y = x, y
        
        # Если кнопка мыши отпущена
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            self.end_x, self.end_y = x, y
            self.p2 = (x,y)
            self.state += 1
            
        # Right click (erase current ROI)
        if event == cv2.EVENT_RBUTTONUP:
            self.p1, self.p2 = None, None
            self.state = 0
            self.start_x, self.start_y = -1, -1
            self.end_x, self.end_y = -1, -1


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
    
    
    def init_tracker(self, img, bbox):
        self.csrt_tracker.init(img, bbox) 
        self.kcf_tracker.init(img, bbox) 
        self.init_switch = True
    
    def create_win(self):
        # Register the mouse callback
        cv2.namedWindow('win')#, cv2.WINDOW_NORMAL)  
        cv2.setMouseCallback('win', self.draw_rectangle)   
        
    def process_img_server(self, img):
        img_center = self.get_center(img, 0, 0, img.shape[1], img.shape[0])
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
        return img


    def process_img_client(self, img):
        if self.state > 1:
            bbox = (self.p1[0], self.p1[1], self.p2[0]-self.p1[0], self.p2[1]-self.p1[1])
            self.init_switch = True
        else:
            bbox = (0, 0, 0, 0)
        return bbox

    def start_stream(self):
        self.cap = cv2.VideoCapture(1)
        # ROI in video
        while self.cap.isOpened():
            # FPS варианты
            start_time = time.time()
#            timer = cv2.getTickCount()
            success, img = self.cap.read()
            img = self.increase_brightness(img)
            #img = cv2.flip(img, 1)
            img_center = self.get_center(img, 0, 0, img.shape[1], img.shape[0])
            
            if self.state > 1:
                cv2.rectangle(img, self.p1, self.p2, (255, 0, 0), 10)  
                bbox = (self.p1[0], self.p1[1], self.p2[0]-self.p1[0], self.p2[1]-self.p1[1])
                self.csrt_tracker.init(img, bbox) 
                self.kcf_tracker.init(img, bbox) 
                
                self.init_switch = True
                self.state = 0
                
            if self.init_switch:
#                success, bbox = csrt_tracker.update(img)
                
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


if __name__ == "__main__":
    print ("START")
    lib_start = TrackerLib()
    lib_start.create_win()
    #print (dir(lib_start))
    lib_start.start_stream()
    

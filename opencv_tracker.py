import  cv2
import time
import numpy as np

# Калибровка

# Найти целевую функцию
def find_marker(image):
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
def distance_to_camera(knownWidth, focalLength, perWidth):  
    # compute and return the distance from the maker to the camera
    return (knownWidth * focalLength) / perWidth            

# focal length finder function
def focal_length(measured_distance, real_width, width_in_rf_image):
    """
    This Function Calculate the Focal Length(distance between lens to CMOS sensor), it is simple constant we can find by using
    MEASURED_DISTACE, REAL_WIDTH(Actual width of object) and WIDTH_OF_OBJECT_IN_IMAGE
    :param1 Measure_Distance(int): It is distance measured from object to the Camera while Capturing Reference image

    :param2 Real_Width(int): It is Actual width of object, in real world (like My face width is = 14.3 centimeters)
    :param3 Width_In_Image(int): It is object width in the frame /image in our case in the reference image(found by Face detector)
    :retrun focal_length(Float):"""
    focal_length_value = (width_in_rf_image * measured_distance) / real_width
    return focal_length_value

 
# initialize the known distance from the camera to the object, which
# in this case is ... mm
KNOWN_DISTANCE = 30.0 # centimeter
 
# initialize spec my box 
KNOWN_WIDTH = 7.7 # centimeter
KNOWN_HEIGHT = 10.5 # centimeter
 
cap = cv2.VideoCapture(0)
# ROI in video
cv2.namedWindow('win')#, cv2.WINDOW_NORMAL)
# Our ROI, defined by two points
p1, p2 = None, None
state = 0
init_switch = False
calibration_state = 0
focalLength = 0
# Called every time a mouse event happen
def on_mouse(event, x, y, flags, userdata):
    global state, p1, p2, calibration_state
    
    # Calibration
    if event == cv2.EVENT_MBUTTONDOWN:
        pass
    
    # Left click
    if event == cv2.EVENT_LBUTTONUP:
        # Select first point
        if state == 0:
            p1 = (x,y)
            state += 1
        # Select second point
        elif state == 1:
            p2 = (x,y)
            state += 1
            
            calibration_state = 1
    # Right click (erase current ROI)
    if event == cv2.EVENT_RBUTTONUP:
        p1, p2 = None, None
        state = 0
# Register the mouse callback
cv2.setMouseCallback('win', on_mouse)

#tracker = cv2.TrackerKCF_create() #TrackerMOSSE_create()
tracker = cv2.TrackerCSRT_create()
#bbox = cv2.selectROI("Tracking", img, False)
#tracker.init(img, bbox)

def get_center(x, y, w, h):
    xcentr = int(x+(w/2))
    ycentr = int(y+(h/2))
    cv2.circle(img, (xcentr, ycentr), radius=0, color=(0, 0, 255), thickness=5)
    return (xcentr, ycentr)    

def draw_box(img, bbox):
    x, y, w, h = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
    cv2.rectangle(img, (x, y), ((x+w), (y+h)), (255, 0, 255), 3, 1)
    return get_center(x, y, w, h)

def increase_brightness(img, value=10):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    v = cv2.add(v,value)
    v[v > 255] = 255
    v[v < 0] = 0
    final_hsv = cv2.merge((h, s, v))
    img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return img


while cap.isOpened():
    # FPS варианты
    start_time = time.time()
    timer = cv2.getTickCount()
    success, img = cap.read()
    img = increase_brightness(img)
    #img = cv2.flip(img, 1)
    img_center = get_center(0, 0, img.shape[1], img.shape[0])
    
    if state > 1:
        cv2.rectangle(img, p1, p2, (255, 0, 0), 10)  
        bbox = (p1[0], p1[1], p2[0]-p1[0], p2[1]-p1[1])
        tracker.init(img, bbox) 
        init_switch = True
        state = 0
        
    if init_switch:
        success, bbox = tracker.update(img)
        if success:
            obj_center = draw_box(img, bbox)
            cv2.line(img, img_center, obj_center, (255,0,0), 4)   
            x_dist = (obj_center[0] - img_center[0])**2
            y_dist = (obj_center[1] - img_center[1])**2 
            #cv2.putText(img, "{}".format(int(np.sqrt(x_dist + y_dist))), (bbox[0],bbox[1]),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                
            # расстояние
            """
            Измерять дистанцию в процентах
            Получить гепотенузу одной части экрана M s
            Получить гепотенузу между 0 и центром обьекта g
            Пересчитать сколько процентов гипотенуза g от гипотенузы М, обозначу L
            100%-L = наш процент по центрам

            все секторы переводить в положительные числа

            Если гипотенуза
            Если x1 > 0 и y1 > 0 и g%

            Применить one euro filter
             
            # размер получаемого изображения
            w = 640
            h = 480
            C(k1, k2) - центр  
            с = Корень / k1**2+b3**2
            
            """
            #img_center
            w_sector, h_sector = img.shape[1]//2, img.shape[0]//2
            M = int(np.sqrt(w_sector**2 + h_sector**2))
            g = int(np.sqrt(x_dist + y_dist))
            p_dist = (g/M) * 100
            
            
            M1 = int(np.sqrt(img.shape[1]**2 + img.shape[0]**2))
            g1_point = int(np.sqrt(bbox[0]**2 + bbox[1]**2))
            p_dist_point = (g1_point/M1) * 100
            #cv2.line(img, (0,0), (bbox[0], bbox[1]), (255,0,0), 4)  
            cv2.line(img, (0,0), (bbox[0], bbox[1]), (255,0,0), 4)
            cv2.putText(img, f"{int(p_dist_point)}%", (bbox[0]//2,bbox[1]//2+10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
            
            # площадь в процентах
            S0=img.shape[1]*bbox[0]
            S1=bbox[2]*bbox[3]
            P_s = (S1/S0)*100
            
            #cv2.putText(img, f"distance: {int(p_dist)} rectangle area: {int(P_s)}", (bbox[0],bbox[1]),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
            
            
            
            #(bbox[0]+bbox[2]), (bbox[1]+bbox[3])
             
            cv2.line(img, (bbox[0]+bbox[2], bbox[1]+bbox[3]), (img.shape[1], img.shape[0]), (255,0,0), 4) 
            
            x_, y_ = bbox[0]+bbox[2], bbox[1]+bbox[3]
            x_, y_ = img.shape[1]-x_, img.shape[0]-y_
            
            
            g2_point = int(np.sqrt(x_**2 + y_**2))
            p_dist_point2 = (g2_point/M1) * 100  
            
            A = (bbox[0]+bbox[2])+x_//2 
            B = (bbox[1]+bbox[3])+y_//2 
            
            #A = x_#//2 
            #B = y_#//2 
            
            cv2.putText(img, f"{int(p_dist_point2)}%", (A, B),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
            
            if int(p_dist)>int(p_dist_point) and int(p_dist_point)>int(p_dist_point2):
                print ("WAR 1")
                #distance = (p_dist_point2+p_dist_point)/2
                #distance = (p_dist+p_dist_point)/2
            elif int(p_dist)<int(p_dist_point) and int(p_dist_point)<int(p_dist_point2):
                print ("WAR 2")
            elif int(p_dist)>int(p_dist_point2) and int(p_dist_point)>int(p_dist_point2):
                print ("WAR 3")
            elif int(p_dist)<int(p_dist_point2) and int(p_dist_point)<int(p_dist_point2):
                print ("WAR 4")
            else:
                print ("WAR 5")
                #if p_dist_point2>p_dist_point:
                #    distance = (p_dist+p_dist_point)/2
                #else:
                #    distance = (p_dist+p_dist_point2)/2
            """"""
            
            distance = (p_dist+p_dist_point+p_dist_point2)/3
            
            #apx_distance = round(((1 - (boxes[0][i][3] - boxes[0][i][1]))**4),1)
            cv2.putText(img, f"distance: {int(p_dist)}%, sum dis: {int(distance)}%", (bbox[0],bbox[1]),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
            
            
            print (int(distance), A, B, x_, y_)
            
            """
            cv2.putText(img, 
                        f"distance: {int(distance)} cm",
                        (img.shape[1] - 400, img.shape[0] - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.7, 
                        (0, 255, 0), 
                        2)
           """ 
            
    # FPS варианты
    #fps = cv2.getTickFrequency()/(cv2.getTickCount()-timer)
    end_time = time.time()
    seconds = end_time - start_time
    fps = 1.0 / seconds
    
    
    cv2.putText(img, f"{int(fps)} fps", (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,0,255),2) #cv2.FONT_HERSHEY_COMPLEX
    
    cv2.imshow("win", img)
    

    if cv2.waitKey(1) & 0xff == ord('q'):
        break      

# https://www.section.io/engineering-education/approximating-the-speed-of-an-object-and-its-distance/
# https://gist.github.com/Pawandeep-prog/48725026639a841e67081094b7db033a
# https://ai.stackexchange.com/questions/25074/how-to-calculate-the-distance-between-the-camera-and-an-object-using-computer-vi
# https://www.baeldung.com/cs/cv-compute-distance-from-object-video



# https://pythonprogramming.net/detecting-distances-self-driving-car/
# https://github.com/mandoo92/DeepSort_OpenCV/tree/main
# https://github.innominds.com/shaoshengsong/DeepSORT








































































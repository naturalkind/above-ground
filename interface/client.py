import cv2
import socket
import pickle
import struct
import tracker_lib

lib_start = tracker_lib.TrackerLib()
lib_start.create_win()
# Создание сокета
client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
host_name = socket.gethostname()
host_ip = socket.gethostbyname(host_name)
# host_ip = '10.42.0.1'
#host_ip = '192.168.1.123'  # Адрес сервера
port = 9999
client_socket.connect((host_ip, port))
data = b""
payload_size = struct.calcsize("Q")

while True:
    timer = cv2.getTickCount()
    if lib_start.state > 1:
        lib_start.state = 0
    # получение данных
    while len(data) < payload_size:
        packet = client_socket.recv(4*1024)
        if not packet: break
        data += packet
    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack("Q",packed_msg_size)[0]
    
    while len(data) < msg_size:
        data += client_socket.recv(4*1024)
    frame_data = data[:msg_size]
    data = data[msg_size:]
    img, init_tracker = pickle.loads(frame_data)
    img = cv2.imdecode(img, 1)
        
    # Если начальные и конечные координаты прямоугольника определены
    if lib_start.start_x != -1 and lib_start.end_x != -1:
        if lib_start.state != 0:
            # Рисование прямоугольника на изображении
            cv2.rectangle(img, (lib_start.start_x, lib_start.start_y), (lib_start.end_x, lib_start.end_y), (0, 255, 0), 2)
            
    fps = cv2.getTickFrequency()/(cv2.getTickCount()-timer)    
    cv2.putText(img, f"{int(fps)} fps", (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,0,255),2)
    
    # прицел
    size_box = 70
    line_box = 2
    shape_image = img.shape
    
    size_0 = int(shape_image[0]/2+size_box)
    size_1 = int(shape_image[0]/2-size_box)
    
    size_2 = int(shape_image[1]/2+size_box)
    size_3 = int(shape_image[1]/2-size_box)
    
    # Прямоугольник
#    img[size_0:size_0+line_box,size_2-(size_box*2):size_2,:] = [1,152,117]
#    img[size_0-(size_box*2):size_0, size_2:size_2+line_box,:] = [1,152,117]
#    img[size_1:size_1+line_box, size_3:size_3+(size_box*2),:] = [1,152,117]
#    img[size_0-(size_box*2):size_0, size_3:size_3+line_box,:] = [1,152,117]

    # левый верхний угол
    img[size_1:size_1+line_box, size_3:size_3+20,:] = [1,152,117]
    img[size_0-(size_box*2):size_0-(size_box*2)+20, size_3:size_3+line_box,:] = [1,152,117]
    
    
    # правый верхний угол
    img[size_0-(size_box*2):size_0-(size_box*2)+20, size_2:size_2+line_box,:] = [1,152,117]
    img[size_1:size_1+line_box, size_3+(size_box*2)-20:size_3+(size_box*2),:] = [1,152,117]
    
    # правый нижний угол
    img[size_0-20:size_0, size_2:size_2+line_box,:] = [1,152,117]
    img[size_0:size_0+line_box, size_2-20:size_2,:] = [1,152,117]
    
    # левый нижний угол
    img[size_0:size_0+line_box, size_2-(size_box*2):size_2-(size_box*2)+20,:] = [1,152,117]
    img[size_0-20:size_0, size_3:size_3+line_box,:] = [1,152,117]
    
    
#    img[size_0:size_0+line_box, size_2-(size_box*2):size_2-(size_box*2),:] = [1,152,117]
   
#    img[size_0-(size_box*2):size_0-(size_box*2)+20, size_3:size_3+line_box,:] = [1,152,117]
    
    # визуализация
    cv2.imshow("win", img)
    if cv2.waitKey(1) & 0xff == ord('q'):
        client_socket.close()
        break
             
    # Отправка обработанного изображения серверу
    #img = cv2.flip(img, 1)
    a = pickle.dumps((lib_start.bbox,
                      lib_start.state,
                      lib_start.init_switch))
    message = struct.pack("Q", len(a)) + a
    client_socket.sendall(message)
    #print (init_tracker, lib_start.init_switch) 

     
cv2.destroyAllWindows()
client_socket.close()





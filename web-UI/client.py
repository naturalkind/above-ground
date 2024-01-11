import cv2
import socket
import pickle
import struct
import tracker_lib

lib_start = tracker_lib.TrackerLib()
lib_start.create_win()
# Создание сокета
client_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
host_ip = '192.168.1.123'  # Адрес сервера
port = 9999
client_socket.connect((host_ip, port))
data = b""
payload_size = struct.calcsize("Q")

while True:
    timer = cv2.getTickCount()
    
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
     
    if init_tracker:
        lib_start.init_switch = False
        lib_start.state = 0
    # выделение мышкой
    bbox = lib_start.process_img_client(img)
                        
    # Если начальные и конечные координаты прямоугольника определены
    if lib_start.start_x != -1 and lib_start.end_x != -1:
        if lib_start.state != 0:
            # Рисование прямоугольника на изображении
            cv2.rectangle(img, (lib_start.start_x, lib_start.start_y), (lib_start.end_x, lib_start.end_y), (0, 255, 0), 2)
            
    fps = cv2.getTickFrequency()/(cv2.getTickCount()-timer)    
    cv2.putText(img, f"{int(fps)} fps", (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,0,255),2)
    # визуализация
    cv2.imshow("win", img)
    if cv2.waitKey(1) & 0xff == ord('q'):
        client_socket.close()
        break
          
    # Отправка обработанного изображения серверу
    #img = cv2.flip(img, 1)
    a = pickle.dumps((bbox,
                      lib_start.state,
                      lib_start.init_switch))
    message = struct.pack("Q", len(a)) + a
    client_socket.sendall(message)
        

     
cv2.destroyAllWindows()
client_socket.close()





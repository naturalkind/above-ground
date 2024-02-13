import cv2, time
import socket
import pickle
import struct
import tracker_lib

from threading import Thread


class VideoStreamWidget(object):
    def __init__(self, src=1):
        self.capture = cv2.VideoCapture(src)
        # Start the thread to read frames from the video stream
        self.thread = Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()

    def update(self):
        # Read the next frame from the stream in a different thread
        while True:
            if self.capture.isOpened():
                (self.status, self.frame) = self.capture.read()
            time.sleep(.01)
    def get_frame(self):
        print ("OKK")
        return self.frame

lib_start = tracker_lib.TrackerLib()
# Создание сокета
server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
host_name = socket.gethostname()
#host_ip = '10.42.0.1'
host_ip ='192.168.1.123'#socket.gethostbyname(host_name)
print('Хост IP:', host_ip)
port = 9999
socket_address = (host_ip, port)

encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
# Привязка сокета
server_socket.bind(socket_address)

# Ожидание подключения клиента
server_socket.listen(5)
print("Ожидание подключения клиента...")
#cap = cv2.VideoCapture(1)
video_stream_widget = VideoStreamWidget()

payload_size = struct.calcsize("Q")
data = b""
init_tracker = False
while True:
    client_socket,addr = server_socket.accept()
    print('Получено соединение от:', addr)
    if client_socket:
        while(video_stream_widget.capture.isOpened()):
            try:
                _img = video_stream_widget.get_frame()
            except AttributeError:
                pass
            if init_tracker == False:
                # Сжатие кадра в формат JPEG
                _, _img_send = cv2.imencode('.jpg', _img, encode_param)
                # отправка данных
                a = pickle.dumps([_img_send, init_tracker])
                message = struct.pack("Q", len(a)) + a
                client_socket.sendall(message)
            
            
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
            bbox, state, init_switch = pickle.loads(frame_data)  
            
            print (state, init_switch, init_tracker)
            if state > 1 and init_switch is True:
                if init_tracker == False:
                    lib_start.init_tracker(_img, bbox)
                init_tracker = True
            img = lib_start.process_img_server(_img)     
            
            if init_tracker == True:
                # Сжатие кадра в формат JPEG
                _, img = cv2.imencode('.jpg', img[0], encode_param)
                a = pickle.dumps([img, init_tracker])
                message = struct.pack("Q", len(a)) + a
                client_socket.sendall(message)


#lib_start = tracker_lib.TrackerLib()
## Создание сокета
#server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
#host_name = socket.gethostname()
#host_ip = '10.42.0.1'
##host_ip ='192.168.1.123'#socket.gethostbyname(host_name)
#print('Хост IP:', host_ip)
#port = 9999
#socket_address = (host_ip, port)

#encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
## Привязка сокета
#server_socket.bind(socket_address)

## Ожидание подключения клиента
#server_socket.listen(5)
#print("Ожидание подключения клиента...")
#cap = cv2.VideoCapture(1)

#payload_size = struct.calcsize("Q")
#data = b""
#init_tracker = False
#while True:
#    client_socket,addr = server_socket.accept()
#    print('Получено соединение от:', addr)
#    if client_socket:
#        while(cap.isOpened()):
#            success, _img = cap.read()
#            if init_tracker == False:
#                # Сжатие кадра в формат JPEG
#                _, _img_send = cv2.imencode('.jpg', _img, encode_param)
#                # отправка данных
#                a = pickle.dumps([_img_send, init_tracker])
#                message = struct.pack("Q", len(a)) + a
#                client_socket.sendall(message)
#            
#            
#            # получение данных
#            while len(data) < payload_size:
#                packet = client_socket.recv(4*1024)
#                if not packet: break
#                data += packet
#            packed_msg_size = data[:payload_size]
#            data = data[payload_size:]
#            msg_size = struct.unpack("Q",packed_msg_size)[0]
#            
#            while len(data) < msg_size:
#                data += client_socket.recv(4*1024)
#            frame_data = data[:msg_size]
#            data = data[msg_size:]
#            bbox, state, init_switch = pickle.loads(frame_data)  
#            
#            print (state, init_switch, init_tracker)
#            if state > 1 and init_switch is True:
#                if init_tracker == False:
#                    lib_start.init_tracker(_img, bbox)
#                init_tracker = True
#            img = lib_start.process_img_server(_img)     
#            
#            
#            if init_tracker == True:
#                # Сжатие кадра в формат JPEG
#                _, img = cv2.imencode('.jpg', img[0], encode_param)
#                a = pickle.dumps([img, init_tracker])
#                message = struct.pack("Q", len(a)) + a
#                client_socket.sendall(message)
#            # визуализация
##            cv2.imshow("win", img)
##            if cv2.waitKey(1) & 0xff == ord('q'):
##                client_socket.close()
##                break 
#                
#                
#        cap.release()
#        cv2.destroyAllWindows()












import cv2
import multiprocessing

print (multiprocessing.Array('i', [0, 0]))
def send_frame(frame, coordinates):
    # ... код для отправки frame и coordinates на другой компьютер ...
    print ("SEND")

def recv_frame():
    # ... код для получения frame и coordinates с другого компьютера ...
    print ("REC")
    
def track(frame, coordinates):
    tracker1 = cv2.TrackerCSRT_create()
    tracker2 = cv2.TrackerKCF_create()

    # ... код для инициализации трекеров ...

    while True:
        # Обновляем трекеры
        success1, box1 = tracker1.update(frame)
        success2, box2 = tracker2.update(frame)

        if success1:
            coordinates[0] = box1
        if success2:
            coordinates[1] = box2

        # Отправляем frame и coordinates на другой компьютер
        process_send_frame = multiprocessing.Process(target=send_frame, args=(frame, coordinates))
        process_send_frame.start()
        process_send_frame.join()

        # Получаем frame и coordinates с другого компьютера
        process_recv_frame = multiprocessing.Process(target=recv_frame)
        process_recv_frame.start()
        process_recv_frame.join()

        # Обновляем frame и coordinates
        frame, coordinates = recv_frame()

        # ... код для обработки frame ...

if __name__ == '__main__':
    # Загружаем видео или получаем кадр с камеры
    video = cv2.VideoCapture(1)
    _, frame = video.read()

    # Инициализируем переменные coordinates
    coordinates = multiprocessing.Array('i', [0, 0])

    # Запускаем отслеживание
    track(frame, coordinates)
    video.release()

import cv2
import multiprocessing
import time


def csrt_kcf_tracker(frame_queue, result_queue):
    # Создать CSRT трекер
    csrt_tracker = cv2.TrackerCSRT_create()
    # Создать KCF трекер
    kcf_tracker = cv2.TrackerKCF_create()
    # Получить первый кадр
    frame = frame_queue.get()
    # Инициализировать трекер
    bbox = cv2.selectROI("Frame", frame, False)
    csrt_tracker.init(frame, bbox)
    kcf_tracker.init(frame, bbox)
    init_ = 0
    while True:
        # Получить кадр из очереди
        if init_ != 0:
            frame = frame_queue.get()
        init_ = 1
        # Обновить трекеры
        csrt_success, csrt_bbox = csrt_tracker.update(frame)
        kcf_success, kcf_bbox = kcf_tracker.update(frame)
        print (csrt_bbox, kcf_bbox)
        # Выбрать наилучший результат
        if csrt_success and kcf_success:
            csrt_area = csrt_bbox[2] * csrt_bbox[3]
            kcf_area = kcf_bbox[2] * kcf_bbox[3]

            if csrt_area > kcf_area:
                result_queue.put(csrt_bbox)
            else:
                result_queue.put(kcf_bbox)
        elif csrt_success:
            result_queue.put(csrt_bbox)
        elif kcf_success:
            result_queue.put(kcf_bbox)
        else:
            result_queue.put(None)
        
def main():
    # Создать очередь для кадров
    frame_queue = multiprocessing.Queue()
    # Создать очередь для результатов
    result_queue = multiprocessing.Queue()        
    # Создать и запустить процесс трекера
    tracker_process = multiprocessing.Process(target=csrt_kcf_tracker, args=(frame_queue, result_queue))
    tracker_process.start()

    # Загрузить видео
    video = cv2.VideoCapture(1)

    while True:
        timer = cv2.getTickCount()
        # Прочитать кадр из видео
        ret, frame = video.read()
        #print (frame.shape, ret)
        if not ret:
            break

        # Поместить кадр в очередь
        frame_queue.put(frame)
        # Получить результат от трекера
        bbox = result_queue.get()

        if bbox is not None:
            # Отобразить рамку вокруг объекта
            x, y, w, h = [int(coord) for coord in bbox]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        fps = cv2.getTickFrequency()/(cv2.getTickCount()-timer)    
        cv2.putText(frame, f"{int(fps)} fps", (20,40), cv2.FONT_HERSHEY_SIMPLEX, 0.7,(0,0,255),2)
        # Отобразить кадр
        cv2.imshow("Frame", frame)

        # Выход по нажатию клавиши 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Остановить процесс трекера
    tracker_process.terminate()
    tracker_process.join()

    # Освободить ресурсы
    video.release()
    cv2.destroyAllWindows()
    
    
if __name__ == "__main__":
    main()    

import cv2
import time
from rknnpool import rknnPoolExecutor
#Функция обработки изображений, вам необходимо изменить ее самостоятельно в процессе фактического применения
from func import myFunc

cap = cv2.VideoCapture(1)
#cap = cv2.VideoCapture(0)
modelPath = "./rknnModel/yolov5s_relu_tk2_RK3588_i8.rknn"
# Увеличьте количество линий, увеличьте скорость
TPEs = 3
pool = rknnPoolExecutor(
    rknnModel=modelPath,
    TPEs=TPEs,
    func=myFunc)

# Инициализируйте требуемый кадр асинхронно
if (cap.isOpened()):
    for i in range(TPEs + 1):
        ret, frame = cap.read()
        if not ret:
            cap.release()
            del pool
            exit(-1)
        pool.put(frame)

frames, loopTime, initTime = 0, time.time(), time.time()
while (cap.isOpened()):
    frames += 1
    ret, frame = cap.read()
    if not ret:
        break
    pool.put(frame)
    frame, flag = pool.get()
    if flag == False:
        break
    cv2.imshow('test', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if frames % 30 == 0:
        print("Средняя частота 30 кадров:\t", 30 / (time.time() - loopTime), "кадр")
        loopTime = time.time()

print("Общая средняя частота кадров\t", frames / (time.time() - initTime))
cap.release()
cv2.destroyAllWindows()
pool.release()

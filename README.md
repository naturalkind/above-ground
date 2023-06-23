# above-ground

gui для создания данных, и будущая игровая площадка для бота   
```
python3 main.py
```

склеить изображение   
```
python3 glue_part.py
```

![Иллюстрация к проекту](https://github.com/naturalkind/above-ground/blob/main/media-info/%D0%A1%D0%BB%D0%B0%D0%B9%D0%B4%201.png)

### Нужно сделать
- [x] нарезать разного размера изображение, из снимка местности   
- [ ] вносить искажения   
- [x] использовать дискрипторы для воссоздания изображения   
- [ ] обучать нейронную сеть   
- [ ] графф для сортировки изображений в нужной последовательности   

#### tkinter рисование с зажатой кнопкой
* https://stackoverflow.com/questions/34522095/gui-button-hold-down-tkinter
* http://pythonicway.com/python-examples/python-gui-examples/28-python-paint
* https://stackoverflow.com/questions/15736530/python-tkinter-rotate-image-animation

#### интересно
* https://habr.com/ru/articles/650013/

#### спрямление длина кривой
* https://stackoverflow.com/questions/5228383/how-do-i-find-the-distance-between-two-points
* https://stackoverflow.com/questions/31543775/how-to-perform-cubic-spline-interpolation-in-python
* https://stackoverflow.com/questions/41175004/python-finding-a-curve-length
* https://russianblogs.com/article/3131128474/

#### поворот на угл в tkinter, numpy
* https://qna.habr.com/q/945435
* https://stackoverflow.com/questions/23463070/rotate-a-square-by-an-angle-in-degree
* https://stackoverflow.com/questions/20840692/rotation-of-a-2d-array-over-an-angle-using-rotation-matrix
* https://stackoverflow.com/questions/54742326/python-numpy-angled-slice-of-3d-array
* https://russianblogs.com/article/5253134646/
* https://stackoverflow.com/questions/37177811/crop-rectangle-returned-by-minarearect-opencv-python
* https://theailearner.com/2020/11/03/opencv-minimum-area-rectangle/
* https://stackoverflow.com/questions/60490882/cut-mask-out-of-image-with-certain-pixel-margin-numpy-opencv
* https://stackoverflow.com/questions/21566610/crop-out-partial-image-using-numpy-or-scipy
* https://stackoverflow.com/questions/36921249/drawing-angled-rectangles-in-opencv
* https://stackoverflow.com/questions/44457064/displaying-stitched-images-together-without-cutoff-using-warpaffine
* https://learnopencv.com/rotation-matrix-to-euler-angles/
* https://stackoverflow.com/questions/10969170/rotation-angle-of-the-perspective-matrix
* https://stackoverflow.com/questions/58538984/how-to-get-the-rotation-angle-from-findhomography

#### детекторы дискрипторы
* https://oil-mcut.github.io/chapter-7/
* https://datahacker.rs/feature-matching-methods-comparison-in-opencv/
* https://habr.com/ru/companies/solarsecurity/articles/580488/
* https://habr.com/ru/articles/516116/
* https://www.programmersought.com/article/443410229012/ simple error
* https://habr.com/ru/companies/coptertime/articles/373911/
* https://habr.com/ru/articles/403389/
* https://stackoverflow.com/questions/38491959/how-to-apply-ransac-in-python-opencv
* https://docs.opencv.org/3.4/d1/de0/tutorial_py_feature_homography.html
* https://magamig.github.io/posts/accurate-image-alignment-and-registration-using-opencv/

#### получить координаты камеры
* https://stackoverflow.com/questions/14444433/calculate-camera-world-position-with-opencv-python
* https://www.fdxlabs.com/calculate-x-y-z-real-world-coordinates-from-a-single-camera-using-opencv/
* https://stackoverflow.com/questions/8927771/computing-camera-pose-with-homography-matrix-based-on-4-coplanar-points/10781165#10781165
* https://stackru.com/questions/10402785/vyichislenie-pozyi-kameryi-s-matritsej-gomografii-na-osnove-4-koplanarnyih-tochek
* https://pythonpath.wordpress.com/import-cv2/

#### оптический поток
* https://mpolinowski.github.io/docs/IoT-and-Machine-Learning/ML/2021-12-10--opencv-optical-flow-tracking/2021-12-10/

#### спектральная кластеризация
* https://calvinfeng.gitbook.io/machine-learning-notebook/unsupervised-learning/clustering/spectral_clustering

#### стабилизация изображения
* https://datahacker.rs/006-advanced-computer-vision-video-stabilization/

#### создание панорам
* http://matthewalunbrown.com/papers/ijcv2007.pdf
* https://pyimagesearch.com/2016/01/11/opencv-panorama-stitching/
* https://datahacker.rs/005-how-to-create-a-panorama-image-using-opencv-with-python/
* https://russianblogs.com/article/31711623484/
* https://russianblogs.com/article/2855376100/
* https://digitrain.ru/articles/330620/
* https://machinelearningmastery.ru/image-panorama-stitching-with-opencv-2402bde6b46c/
* https://www.programmersought.com/article/63932526410/
* https://russianblogs.com/article/84121976861/
* https://russianblogs.com/article/62961265120/ - с++ реализация

#### создание панорам код
* https://github.com/fuenwang/PanoramaUtility
* https://github.com/ndvinh98/Panorama/
* https://github.com/VCL3D/DeepPanoramaLighting
* https://github.com/aartighatkesar/Image-Mosaicing
* https://github.com/RyanWu2233/Panorama
* https://github.com/sheoranhimansh/AutoPanorama - хорошая реализация с использованием нескольких подходов

#### готовые решения
* https://github.com/generalized-intelligence/GAAS
* https://github.com/kregmi/cross-view-image-matching
* https://github.com/pacogarcia3/hta0-horizontal-robot-arm
* https://github.com/amov-lab/Prometheus
* https://github.com/stytim/Drone_Visual_SLAM
* https://github.com/matlabbe/rtabmap_drone_example

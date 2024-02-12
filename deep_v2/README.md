
```
g++ -Wall -fexceptions -O3 -I/home/rock/rknpu2/examples/3rdparty/rga/RK3588/include -I/usr/local/include/opencv4 -I/home/rock/rknpu2/runtime/RK3588/Linux/librknn_api/include -c postprocess.cpp -o build/postprocess.o
```

```
g++ -Wall -fexceptions -O3 -I/home/rock/rknpu2/examples/rknn_yolov5_demo/include -I/home/rock/rknpu2/examples/3rdparty/rga/RK3588/include -I/usr/local/include/opencv4 -I/home/rock/rknpu2/runtime/RK3588/Linux/librknn_api/include -c main.cpp -o build/main.o
```

```
g++  -o NPU build/main.o build/postprocess.o  -O3 -fopenmp -I/usr/local/include/opencv4 -L/usr/local/lib -lopencv_gapi -lopencv_stitching -lopencv_alphamat -lopencv_aruco -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_dnn_objdetect -lopencv_dnn_superres -lopencv_dpm -lopencv_face -lopencv_freetype -lopencv_fuzzy -lopencv_hdf -lopencv_hfs -lopencv_img_hash -lopencv_intensity_transform -lopencv_line_descriptor -lopencv_mcc -lopencv_quality -lopencv_rapid -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_sfm -lopencv_stereo -lopencv_structured_light -lopencv_phase_unwrapping -lopencv_superres -lopencv_optflow -lopencv_surface_matching -lopencv_tracking -lopencv_highgui -lopencv_datasets -lopencv_text -lopencv_plot -lopencv_videostab -lopencv_videoio -lopencv_wechat_qrcode -lopencv_xfeatures2d -lopencv_shape -lopencv_ml -lopencv_ximgproc -lopencv_video -lopencv_xobjdetect -lopencv_objdetect -lopencv_calib3d -lopencv_imgcodecs -lopencv_features2d -lopencv_dnn -lopencv_flann -lopencv_xphoto -lopencv_photo -lopencv_imgproc -lopencv_core -ldl -lpthread -pthread -lgomp -DNDEBUG -rdynamic -march=armv8.2-a -s  /usr/local/lib/librknnrt.so /usr/local/lib/librga.so
```

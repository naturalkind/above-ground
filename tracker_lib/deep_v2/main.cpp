/*-------------------------------------------
                Includes
-------------------------------------------*/
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include <math.h>

#define _BASETSD_H

#include "im2d.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "postprocess.h"
#include "rga.h"
#include "rknn_api.h"

//DeepSort
#include "tracker.h"//deepsort
#include "FeatureTensor.h"

#define PERF_WITH_POST 1

class detect_result
{
public:
    int classId;
    float confidence;
    cv::Rect_<float> box;
};



/*-------------------------------------------
                  Functions
-------------------------------------------*/

static void dump_tensor_attr(rknn_tensor_attr* attr)
{
  printf("  index=%d, name=%s, n_dims=%d, dims=[%d, %d, %d, %d], n_elems=%d, size=%d, fmt=%s, type=%s, qnt_type=%s, "
         "zp=%d, scale=%f\n",
         attr->index, attr->name, attr->n_dims, attr->dims[0], attr->dims[1], attr->dims[2], attr->dims[3],
         attr->n_elems, attr->size, get_format_string(attr->fmt), get_type_string(attr->type),
         get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);
}

double __get_us(struct timeval t) { return (t.tv_sec * 1000000 + t.tv_usec); }

static unsigned char* load_data(FILE* fp, size_t ofst, size_t sz)
{
  unsigned char* data;
  int            ret;

  data = NULL;

  if (NULL == fp) {
    return NULL;
  }

  ret = fseek(fp, ofst, SEEK_SET);
  if (ret != 0) {
    printf("blob seek failure.\n");
    return NULL;
  }

  data = (unsigned char*)malloc(sz);
  if (data == NULL) {
    printf("buffer malloc failure.\n");
    return NULL;
  }
  ret = fread(data, 1, sz, fp);
  return data;
}

static unsigned char* load_model(const char* filename, int* model_size)
{
  FILE*          fp;
  unsigned char* data;

  fp = fopen(filename, "rb");
  if (NULL == fp) {
    printf("Open file %s failed.\n", filename);
    return NULL;
  }

  fseek(fp, 0, SEEK_END);
  int size = ftell(fp);

  data = load_data(fp, 0, size);

  fclose(fp);

  *model_size = size;
  return data;
}

/*-------------------------------------------
                 Deep Sort
-------------------------------------------*/

void get_detections(DETECTBOX box,float confidence,DETECTIONS& d)
{
    DETECTION_ROW tmpRow;
    tmpRow.tlwh = box;//DETECTBOX(x, y, w, h);

    tmpRow.confidence = confidence;
    d.push_back(tmpRow);
}

void test_deepsort(cv::Mat& frame, std::vector<detect_result>& results, tracker& mytracker)
{
    std::vector<detect_result> objects;

    DETECTIONS detections;
    for (detect_result dr : results)
    {
        //cv::putText(frame, classes[dr.classId], cv::Point(dr.box.tl().x+10, dr.box.tl().y - 10), cv::FONT_HERSHEY_SIMPLEX, .8, cv::Scalar(0, 255, 0));
        if(dr.classId == 4) // 4 - airplane
        {
            objects.push_back(dr);
            cv::rectangle(frame, dr.box, cv::Scalar(255, 0, 0), 2);
            get_detections(DETECTBOX(dr.box.x, dr.box.y, dr.box.width,  dr.box.height), dr.confidence,  detections);
        }
    }

    std::cout<<"begin track"<<std::endl;
    if(FeatureTensor::getInstance()->getRectsFeature(frame, detections))
    {
        std::cout << "get feature succeed!"<<std::endl;
        mytracker.predict();
        mytracker.update(detections);
        std::vector<RESULT_DATA> result;
        for(Track& track : mytracker.tracks) {
            if(!track.is_confirmed() || track.time_since_update > 1) continue;
            result.push_back(std::make_pair(track.track_id, track.to_tlwh()));
        }
        for(unsigned int k = 0; k < detections.size(); k++)
        {
            DETECTBOX tmpbox = detections[k].tlwh;
            cv::Rect rect(tmpbox(0), tmpbox(1), tmpbox(2), tmpbox(3));
            cv::rectangle(frame, rect, cv::Scalar(0,0,255), 4);
            // cvScalar的储存顺序是B-G-R，CV_RGB的储存顺序是R-G-B
//            std::cout << "info"<<std::endl;
            for(unsigned int k = 0; k < result.size(); k++)
            {
                DETECTBOX tmp = result[k].second;
                cv::Rect rect = cv::Rect(tmp(0), tmp(1), tmp(2), tmp(3));
                rectangle(frame, rect, cv::Scalar(255, 255, 0), 2);

                std::string label = cv::format("%d", result[k].first);
                cv::putText(frame, label, cv::Point(rect.x, rect.y), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);
            }
        }
    }
    std::cout<<"end track"<<std::endl;
}

/*-------------------------------------------
                  Main Functions
-------------------------------------------*/


int main(int argc, char** argv)
{
    char*          model_name = NULL;
    const char*    imagepath = argv[1];
    rknn_context   ctx;
    int            img_width          = 0;
    int            img_height         = 0;
    const float    nms_threshold      = NMS_THRESH;
    const float    box_conf_threshold = BOX_THRESH;
    int            ret;

    float f;
    float FPS[16];
    int i, Fcnt=0;
    std::chrono::steady_clock::time_point Tbegin, Tend;
    
    // Deep Sort
    const int nn_budget=100;
    const float max_cosine_distance=0.2;
    std::vector<detect_result> results;
    tracker mytracker(max_cosine_distance, nn_budget);
    

    for(i=0;i<16;i++) FPS[i]=0.0;


    // init rga context
    rga_buffer_t src;
    rga_buffer_t dst;
    im_rect      src_rect;
    im_rect      dst_rect;
    memset(&src_rect, 0, sizeof(src_rect));
    memset(&dst_rect, 0, sizeof(dst_rect));
    memset(&src, 0, sizeof(src));
    memset(&dst, 0, sizeof(dst));

    if (argc != 2) {
        fprintf(stderr,"Usage: %s [imagepath]\n", argv[0]);
        return -1;
    }

    printf("post process config: box_conf_threshold = %.2f, nms_threshold = %.2f\n", box_conf_threshold, nms_threshold);

    model_name       = (char*) "/home/orange/Общедоступные/YoloV5-NPU-Rock-5/model/yolov5s-640-640.rknn";


    // Create the neural network
    printf("Loading mode...\n");
    int            model_data_size = 0;
    unsigned char* model_data      = load_model(model_name, &model_data_size);
    ret                            = rknn_init(&ctx, model_data, model_data_size, 0, NULL);
    if (ret < 0) {
        printf("rknn_init error ret=%d\n", ret);
        return -1;
    }

    rknn_sdk_version version;
    ret = rknn_query(ctx, RKNN_QUERY_SDK_VERSION, &version, sizeof(rknn_sdk_version));
    if (ret < 0) {
        printf("rknn_init error ret=%d\n", ret);
        return -1;
    }
    printf("sdk version: %s driver version: %s\n", version.api_version, version.drv_version);

    rknn_input_output_num io_num;
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret < 0) {
        printf("rknn_init error ret=%d\n", ret);
        return -1;
    }
    printf("model input num: %d, output num: %d\n", io_num.n_input, io_num.n_output);

    rknn_tensor_attr input_attrs[io_num.n_input];
    memset(input_attrs, 0, sizeof(input_attrs));
    for(uint32_t i = 0; i < io_num.n_input; i++) {
        input_attrs[i].index = i;
        ret                  = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));
        if (ret < 0) {
            printf("rknn_init error ret=%d\n", ret);
            return -1;
        }
        dump_tensor_attr(&(input_attrs[i]));
    }

    rknn_tensor_attr output_attrs[io_num.n_output];
    memset(output_attrs, 0, sizeof(output_attrs));
    for (uint32_t i = 0; i < io_num.n_output; i++) {
        output_attrs[i].index = i;
        ret                   = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
        dump_tensor_attr(&(output_attrs[i]));
    }

    int channel = 3;
    int width   = 0;
    int height  = 0;
    if (input_attrs[0].fmt == RKNN_TENSOR_NCHW) {
        printf("model is NCHW input fmt\n");
        channel = input_attrs[0].dims[1];
        height  = input_attrs[0].dims[2];
        width   = input_attrs[0].dims[3];
    }
    else {
        printf("model is NHWC input fmt\n");
        height  = input_attrs[0].dims[1];
        width   = input_attrs[0].dims[2];
        channel = input_attrs[0].dims[3];
    }

    printf("model input height=%d, width=%d, channel=%d\n", height, width, channel);

    rknn_input inputs[1];
    memset(inputs, 0, sizeof(inputs));
    inputs[0].index        = 0;
    inputs[0].type         = RKNN_TENSOR_UINT8;
    inputs[0].size         = width * height * channel;
    inputs[0].fmt          = RKNN_TENSOR_NHWC;
    inputs[0].pass_through = 0;

    // You may not need resize when src resulotion equals to dst resulotion
    void* resize_buf = nullptr;
    cv::Mat orig_img;
    cv::Mat img;

    cv::VideoCapture cap(imagepath);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 3);
    if (!cap.isOpened())
    {
        std::cout << "!!! Failed to open file: " << imagepath << std::endl;
        return -1;
    }
    printf("Start grabbing, press ESC on Live window to terminated...\n");
    
    // fps
    double fps;    
    while(1){
        double start = cv::getTickCount(); // начало отсчета времен
        
        if (!cap.read(orig_img))
            break;
        char key = cv::waitKey(20);
        if (key == 27) // ESC
            break;

        Tbegin = std::chrono::steady_clock::now();

        cv::cvtColor(orig_img, img, cv::COLOR_BGR2RGB);
        img_width  = img.cols;
        img_height = img.rows;

        if (img_width != width || img_height != height) {
            if(resize_buf == nullptr){
                resize_buf = malloc(height * width * channel);
            }
            memset(resize_buf, 0x00, height * width * channel);

            src = wrapbuffer_virtualaddr((void*)img.data, img_width, img_height, RK_FORMAT_RGB_888);
            dst = wrapbuffer_virtualaddr((void*)resize_buf, width, height, RK_FORMAT_RGB_888);
            ret = imcheck(src, dst, src_rect, dst_rect);
            if (IM_STATUS_NOERROR != ret) {
                printf("%d, check error! %s", __LINE__, imStrError((IM_STATUS)ret));
                return -1;
            }
            imresize(src, dst);

            inputs[0].buf = resize_buf;
        } else {
            inputs[0].buf = (void*)img.data;
        }

//      gettimeofday(&start_time, NULL);
        rknn_inputs_set(ctx, io_num.n_input, inputs);

        rknn_output outputs[io_num.n_output];
        memset(outputs, 0, sizeof(outputs));
        for (uint32_t i = 0; i < io_num.n_output; i++) {
            outputs[i].want_float = 0;
        }

        ret = rknn_run(ctx, NULL);
        ret = rknn_outputs_get(ctx, io_num.n_output, outputs, NULL);

//        gettimeofday(&stop_time, NULL);
//        printf("once run use %f ms\n", (__get_us(stop_time) - __get_us(start_time)) / 1000);

        // post process
        float scale_w = (float)width / img_width;
        float scale_h = (float)height / img_height;

        detect_result_group_t detect_result_group;
        std::vector<float>    out_scales;
        std::vector<int32_t>  out_zps;
        for (uint32_t i = 0; i < io_num.n_output; ++i) {
            out_scales.push_back(output_attrs[i].scale);
            out_zps.push_back(output_attrs[i].zp);
        }
        post_process((int8_t*)outputs[0].buf, (int8_t*)outputs[1].buf, (int8_t*)outputs[2].buf, height, width,
                   box_conf_threshold, nms_threshold, scale_w, scale_h, out_zps, out_scales, &detect_result_group);

//typedef struct _BOX_RECT
//{
//    int left;
//    int right;
//    int top;
//    int bottom;
//} BOX_RECT;

//typedef struct __detect_result_t
//{
//    char name[OBJ_NAME_MAX_SIZE];
//    BOX_RECT box;
//    float prop;
//} detect_result_t;

//typedef struct _detect_result_group_t
//{
//    int id;
//    int count;
//    detect_result_t results[OBJ_NUMB_MAX_SIZE];
//} detect_result_group_t;



//class detect_result
//{
//public:
//    int classId;
//    float confidence;
//    cv::Rect_<float> box;
//};


//         Draw Objects
        char text[256];
        for (int i = 0; i < detect_result_group.count; i++) {
            
            detect_result_t* det_result = &(detect_result_group.results[i]);

            int x1 = det_result->box.left;
            int y1 = det_result->box.top;
            int x2 = det_result->box.right;
            int y2 = det_result->box.bottom;
            
            // Отображение класса
//            cv::rectangle(orig_img, cv::Point(x1, y1), cv::Point(x2, y2),cv::Scalar(255, 0, 0));

//            //put some text
//            sprintf(text, "%s %.1f%%", det_result->name, det_result->prop * 100);

//            int baseLine = 0;
//            cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

//            int x = det_result->box.left;
//            int y = det_result->box.top - label_size.height - baseLine;
//            if (y < 0) y = 0;
//            if (x + label_size.width > orig_img.cols) x = orig_img.cols - label_size.width;

//            cv::rectangle(orig_img, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)), cv::Scalar(255, 255, 255), -1);

//            cv::putText(orig_img, text, cv::Point(x, y + label_size.height), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
            
            // данные для трекера
            detect_result dr;
            int idx = det_result->classId;
            dr.box = cv::Rect(cv::Point(x1, y1), cv::Size((x2 - x1), (y2 - y1)));
            dr.classId = idx;
            dr.confidence = det_result->prop ;
            results.push_back(dr);
            
        }

        ret = rknn_outputs_release(ctx, io_num.n_output, outputs);

        Tend = std::chrono::steady_clock::now();
        //calculate frame rate
        f = std::chrono::duration_cast <std::chrono::milliseconds> (Tend - Tbegin).count();
        if(f>0.0) FPS[((Fcnt++)&0x0F)]=1000.0/f;
        for(f=0.0, i=0;i<16;i++){ f+=FPS[i]; }
        putText(orig_img, cv::format("FPS %0.2f", f/16), cv::Point(10,20),cv::FONT_HERSHEY_SIMPLEX,0.6, cv::Scalar(0, 0, 255));

        // DEEP SORT
        test_deepsort(orig_img, results, mytracker);

        double end = cv::getTickCount();  // конец отсчета времени

        // Расчет fps
        double totalTime = (end - start) / cv::getTickFrequency();
        fps = 1.0 / totalTime;
        
        //show fps
        std::cout << "FPS --->" << fps << std::endl;
        
        imshow("Orange pi 5 - 2,5 GHz|16 Gb", orig_img);
        char esc = cv::waitKey(5);
        if(esc == 27) break;
        
        results.clear();
//      imwrite("./out.jpg", orig_img);
    }

    // release
    ret = rknn_destroy(ctx);

    if(model_data) free(model_data);
    if(resize_buf) free(resize_buf);

    cap.release();
    cv::destroyAllWindows();


    return 0;
}

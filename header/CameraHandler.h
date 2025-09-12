#ifndef CAMERA_HANDLER_H
#define CAMERA_HANDLER_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <arv.h>




#define CHECK_ERROR(error_ptr, msg) \
    if (error_ptr) { \
        std::cerr << msg << ": " << (error_ptr->message ? error_ptr->message : "Unknown error") << std::endl; \
        g_error_free(error_ptr); \
        error_ptr = NULL; \
    }




using namespace std;
using namespace cv;

struct ImageSources {
    string cam_sr_no = "";
    bool cameraMapped = false;
    Mat Img;
    int Width = 0;
    int Height = 0;
    int tickFrequency = 0;
    string folderpath = "";
    int currentIndex = 0;
    bool pathAssigned = false;
    int TriggerType = 0;
};

class CameraHandler {
public:
    CameraHandler();  
    void PrepareCamera();       
    void StopCamera();
    void SoftTrigger(int);
    void SetExposure(double,double,int,String);
    double GetExposure(string);
    double GetGamma(string);
    unsigned int NoOfCamera;
    vector<ImageSources> img_src_id;  
    vector<ArvCamera*> cameras;   
    vector<ArvBuffer*> buffers;
    vector<ArvStream*> streams;
    

};

#endif 


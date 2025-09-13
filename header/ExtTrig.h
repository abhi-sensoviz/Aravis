#ifndef CAMERA_HANDLER_H
#define CAMERA_HANDLER_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <arv.h>
#include <stdexcept> 
#include <signal.h>




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

struct CallBackData{
    ArvCamera* cam;
    ArvBuffer* buffer;
    ArvStream* stream;
    vector<ImageSources>* img_src;
    int index;
};

class CImageEventPrinter {
public:
    static void OnImageGrabbed(ArvStream* , gpointer );
    static void convertImage(ArvBuffer* ,ArvStream* ,unsigned int , int );
};


class ExtTrig {
public:
    ExtTrig();  
    int InitiallizeBuffers();
    int PrepareCamera();       
    void StopCamera();
    void SoftTrigger(int);
    void SetExposure(double,double,int,String);
    double GetExposure(string);
    double GetGamma(string);
    int InitCameraSerialDetails();
    int MapCamerainOrder();
    bool CheckCamConnection( int);
    double GetCameraTickCount(string );
    void ChangeTriggerToSoftwareType();
    // void OnImageGrabbed(ArvCamera* ,ArvBuffer*,ArvStream*);
    // void convertImage(ArvBuffer*,ArvStream*,unsigned int,int);

    unsigned int NoOfCamera=1;
    vector<ImageSources> img_src;  
    ArvCamera** camera;   
    vector<ArvBuffer*> buffers;
    vector<ArvStream*> streams;
    vector<String> cam_sr_no_str;
    
    
    vector<gint> CamWd;
    vector<gint> CamHt;
    vector<guint64> CamTickFreq;
    vector<bool> flag;


    int gImageCounter = 0;


    int ImgSourceMode=1;
    

};

#endif 


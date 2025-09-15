
#ifndef EXTTRIG_H
#define EXTTRIG_H


// Include files to use the PYLON API.
//#include <pylon/PylonIncludes.h>


#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <opencv.hpp>
#include <string>
#include <QTime>
#include "../Process/ossmprocessing.h"
#include <QMessageBox>
//#include <arv.h>

#if defined(_WIN32) // predefined Macro for ALL VC++ projects!
#include <windows.h>
#include <conio.h>
#else
#include <unistd.h>
#include <termios.h>
#endif


#define CHECK_ERROR(error_ptr, msg) \
    if (error_ptr) { \
        std::cerr << msg << ": " << (error_ptr->message ? error_ptr->message : "Unknown error") << std::endl; \
        g_error_free(error_ptr); \
        error_ptr = NULL; \
    }






// Namespace for using pylon objects.
//using namespace Pylon;


//#if INTERFACE
//    #include <pylon/gige/BaslerGigEInstantCamera.h>
//    typedef Pylon::CBaslerGigEInstantCamera Camera_t;
//    using namespace Basler_GigECameraParams;
//#else

//    #include <pylon/usb/BaslerUsbInstantCamera.h>
//    typedef Pylon::CBaslerUsbInstantCamera Camera_t;
//    using namespace Basler_UsbCameraParams;
//#endif

using namespace std;

extern vector<int> Cam_time_gap;
extern vector<double> str_str;
extern vector<bool> run_started;

void InitializeForNewPart(int partId);
extern int part_id_2;
//arv





class ExtTrig
{
public:
    ExtTrig();

    double GetCameraTickCount(string id);
    void StopCamera();
    void SetExposure(double exp,double gam,int gain,string id);
    double GetExposure(string id);
    double GetGamma(string id);
    int InitiallizeBuffers();
    int PrepareCamera();
    int MapCamerainOrder();
    int InitCameraSerialDetails();
    void SoftTrigger(int id);
    void convertImage(Mat Img, int index, int count);

    void OnImageGrabbed( string str, int index);
    bool  CheckCamConnection( int id);

    bool  RefreshCamera();
    void ChangeTriggerType();
void ChangeTriggerToSoftwareType();

    vector<int> CamWd;
    vector<int> CamHt;
    vector<int> CamTickFreq;
    vector<bool> flag;

    int TotalNoOfCamera;

    int returncode;
    int count;
    int img_count;
    bool bAutoTuning;


};

#endif // EXTTRIG_H


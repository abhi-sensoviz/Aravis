
/*=============================================================================
  Copyright (C) 2017 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        ActionCommands.cpp

  Description: see header file for description

-------------------------------------------------------------------------------

  THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF TITLE,
  NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR  PURPOSE ARE
  DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

=============================================================================*/

#include "ExtTrig.h"
#include <iostream>
#include <string.h>
#include <QDebug>
#include "../RS232/rs232main.h"
#include <unistd.h>

#ifdef signals
#undef signals
#endif
#define signals ARV_SIGNALS
#include <arv.h>
#undef signals


struct Camera_t{
    ArvCamera* cam;
    ArvStream* stream;
    ArvBuffer* buffer;
    string serial;
    unsigned int index;

}typedef Camera_t;
Camera_t** camera;


class CImageEventPrinter {
public:
    static void OnImageGrabbed(void *user_data, ArvStreamCallbackType type, ArvBuffer *buffer);
    static void convertImage(unsigned int ,int,ArvStream*,ArvBuffer* );
    static int FindPartIDinSyncWithCamImg(int, double);
};



vector<int> CamWd;
vector<int> CamHt;

vector<int> CamTickFreq;
vector<bool> flag;
vector<int> Cam_time_gap;
vector<double> str_str;
vector<bool> run_started;
vector<bool> part_received;
vector<int> part_id;




int gImageCounter = 0;
int gImageCounter1 = 0;
int part_id_2=0;



/*!
 * \brief ExtTrig Constructor Function to initiallize for certain variables in class
 */

ExtTrig::ExtTrig()
{

    bAutoTuning=true;


}

void InitializeForNewPart(int partId)
{


    log_doc.LogMsg() << "InitializeForNewPart id : " << partId << endl;
    cout << "InitializeForNewPart id : " << partId << endl;

    bool success=false;

    s_rec[partId].trigged=true;
    s_rec[partId].id=partId;
    s_rec[partId].timer=QTime::currentTime();
    s_rec[partId].triggTimer=QTime::currentTime();
    s_rec[partId].ejecTrigered=false;
    s_rec[partId].decision=false;
    s_rec[partId].processed=false;

    s_rec[partId].HolePresent=0;
    for(int j=0;j<NoOfCamera;j++)
    {
        s_rec[partId].cam_decision[j]=false;
        s_rec[partId].cam_patch_decision[j]=false;
        s_rec[partId].cam_processed[j]=false;
        s_rec[partId].cam_retrieved[j]=false;
        s_rec[partId].camTrigered[j]=false;
        s_rec[partId].threadCreated[j]=false;
        s_rec[partId].cam_triggTimer[j]=new QTime();
    }

    s_rec[partId].process_strated=false;
    //    lastPartEntryTime = QDateTime::currentDateTime();

}


/*!
 * \brief ExtTrig::convertImage This Function is for offline Mode. This function read and store the image in appropriate buffer
 * \param Img Image to be stored
 * \param index Camera Index
 * \param count var to count images
 */
void ExtTrig::convertImage(Mat Img, int index, int count) //(frame_store &frame)

{
    int         Width;
    int         Height;
    int         BufferSize;

//    cerr<<"convert image reached"<<endl;
    Width = Img.cols;
    Height=Img.rows;
    unsigned char* pBuffer = Img.data;
    //log_doc.LogMsg() << "Width: " << Width << "Height" << Height <<"Index:"<<index;
    qDebug() << "Width: " << Width << "Height" << Height <<"Index:"<<index<<endl;
    qDebug() << "Img Src Width: " << img_src[index].Width << "Img Src Height" << img_src[index].Height <<"Index:"<<index<<endl;

    //imshow("",Img);
    //waitKey(0);
    BufferSize= Width*Height;

    if(state==run_mode)
    {
        for(int j=0;j<NoOfCamera;j++)
        {
            if(index==j)
            {

                bool success=false;


                QTime str1=QTime::currentTime();
                int leasttime=10000;
                int id=-1000;

                int base_cam_index=0;
                for(int i=0;i<NoOfCamera;i++)
                {
                    if(cam_active[i])
                    {
                        base_cam_index=i;
                        break;
                    }
                }
                part_received[index]=true;

                if(part_id[index]==NoOfParts)
                {
                    part_id[index]=0;
                }

                if(index==base_cam_index)
                {
                    InitializeForNewPart(part_id[index]);

                }

                part_received[index]=false;
                id=part_id[index];
                part_id[index]++;



                int new_id=id;

                log_doc.LogMsg()<<"Camera Image Count for Cam "<<index;
                log_doc.LogMsg()<< "Frame Assigned for Cam"<<index<<"to Part ID "<<id;

                if(new_id!=-1000&&!s_rec[new_id].cam_retrieved[j])
                {




                    std::copy( pBuffer, pBuffer+BufferSize, s_rec[new_id].fr_Cam[j].buff );

                    s_rec[new_id].cam_retrieved[j]=true;
                    s_rec[new_id].trigged=true;


                }
                else{
                }


            }

        }


    }
    else
    {
        for(int j=0;j<NoOfCamera;j++)
        {
            if(index==j)
            {
                run_started[j]=false;
                fr[j].Active=true;
                fr[j].CameraIndex=j;
                //                if(fr[j].buff!=NULL)
                //                {
                //                    std::memset(fr[j].buff, 0, BufferSize);
                //                }
                if(m_ConvertImage[j].cols!=img_src[j].Width)
                {
                    m_ConvertImage[j]=cv::Mat(Height,Width,CV_8UC1);

                }

                std::memcpy(m_ConvertImage[j].data,pBuffer, BufferSize );

            }
        }
    }





}

/*!
 * \brief ExtTrig::OnImageGrabbed This Function is for Offline mode Image Grab simulation
 * \param str Path of Image
 * \param index Camera Index
 */
void ExtTrig:: OnImageGrabbed( string str, int index)
{
    //log_doc.LogMsg() << "On Image Grabbed Cam" << index; // device

    //qDebug() << "On Image Grabbed  Cam" << index<<QString::fromStdString(str); // device
    Mat Img=imread(str,0);
    if(Img.channels()==3)
    {
        cvtColor(Img,Img,CV_BGR2GRAY);
        qDebug()<<"Color Image "<<Img.cols<<" "<<Img.rows<<endl;
    }

    Mat ImgCropped;
    if(CropImageHalf==1)
    {

        ImgCropped=Img(Rect(0,0,Img.cols*0.5,Img.rows)).clone();
        qDebug()<<"Cropped Image"<<ImgCropped.cols<<" "<<ImgCropped.rows<<endl;
        convertImage(ImgCropped,  index,  gImageCounter);

    }
    else {
        qDebug()<<"1 Not Cropped Image"<<Img.cols<<" "<<Img.rows<<endl;
        convertImage(Img,  index,  gImageCounter);

    }

}


//Initiallizing all buffers
int ExtTrig::InitiallizeBuffers(){

    camera = new Camera_t*[NoOfCamera];
    for(unsigned int i=0;i<NoOfCamera;i++){

        camera[i]=new Camera_t();
        int CamWd_temp=0;
        int CamHt_temp=0;
        bool flag_temp=false;
        int Cam_time_gap_temp=0;
        double str_str_temp=0;
        bool run_started_temp=false;
        bool part_received_temp=false;
        int part_id_temp=0;
        int nCamImgRecieved_temp=0;
        int nCamImgMapped_temp=0;



        CamWd.push_back(CamWd_temp);
        CamHt.push_back(CamHt_temp);
        CamTickFreq.push_back(0);
        flag.push_back(flag_temp);
        Cam_time_gap.push_back(Cam_time_gap_temp);
        str_str.push_back(str_str_temp);
        run_started.push_back(run_started_temp);
        part_received.push_back(part_received_temp);
        part_id.push_back(part_id_temp);
        nCamImgRecieved.push_back(nCamImgRecieved_temp);
        nCamImgMapped.push_back(nCamImgMapped_temp);
        // /cout<< NoOfCamera<<endl;

    }
    for(unsigned int i=0;i<NoOfCamera;i++){
        str_str[i]=0;
        run_started[i]=false;;
        part_received[i]=false;
        part_id[i]=0;
        nCamImgRecieved[i]=0;
        nCamImgMapped[i]=0;





    }
    return 0;





}

//check camera connections -
bool ExtTrig:: CheckCamConnection( int id)
{
    unsigned int j=0;
    GError* error=NULL;
    if(ImgSourceMode==1){
        try{
            for(j=0;j<NoOfCamera;j++){
                if(camera[j]->cam){

                    string sr_no=String(arv_camera_get_device_serial_number(camera[j]->cam,&error));
                    CHECK_ERROR(error, "Camera "+to_string(j)+" Not Connected");
                    if(img_src[id].cam_sr_no==sr_no) break;
                }
            }

            if(j<NoOfCamera&&j>=0){
                //arv_update_device_list();//bottle neck

                arv_camera_get_device_serial_number(camera[j]->cam,&error);
                if(error!=NULL){
                    g_error_free(error);
                    cerr << "camera not connected"<<endl;
                    cam_sr_no_str[j]="";
                    return false;
                }
            }
        }catch (const runtime_error& e){
            // Error handling.
            cerr << "An exception occurred. 10" << endl
            << e.what() << endl;
            return false;
        }
        return true;
    }
    else{
        if(img_src[id].pathAssigned==true){
            return true;
        }
        else{
            return false;
        }

    }

}

//prepare camera
int ExtTrig::PrepareCamera(){
    try{
        //cout<<"PREPARE CAMERA"<<endl;
        arv_update_device_list();// refresh camera list
        //cout << "Prepare camera reached"<<endl;
        GError* error = NULL; //get number of camera
        unsigned int TotalConnectedCameras=arv_get_n_devices();

        //NoOfCamera= arv_get_n_devices(); //is configured in s/w comment this out
        if(TotalConnectedCameras==0){
            throw runtime_error("No Cameras Found");
        }
        if(NoOfCamera>TotalConnectedCameras){
            String text= "Make sure all camera are conneced only "+to_string(TotalConnectedCameras) + "detected";
            throw runtime_error("Error:"+text);
            //QMessageBox::information(nullptr, "Camera Error", QString::fromStdString(text));
        }

        cout << "Number of camera: " << NoOfCamera << endl;
        if (NoOfCamera == 0) {
            throw runtime_error("No camera found!");
        }

        for (unsigned int i = 0; i < TotalConnectedCameras; i++) {
            if(i<NoOfCamera){
                const char* model = arv_get_device_model(TotalConnectedCameras -i-1);
                const char* serial = arv_get_device_serial_nbr(TotalConnectedCameras -i-1);
                const char* vendor = arv_get_device_vendor(TotalConnectedCameras -i-1);

//				//creating source obj
//                ImageSource source;
//				source.cam_sr_no=serial;
//				source.cameraMapped=true;
//				img_src.push_back(source);
                //change to cam_sr_no_str


                //print info
                cout << "Camera " << i << ":" << endl;
                cout << "Vendor        : " << vendor << endl;
                cout << "  Model       : " << model << endl;
                cout << "  Serial      : " << serial<< endl;
                cout << "-----------------------------" << endl;



                //opeaning camera
                string camera_id = string(vendor)+"-"+ serial;
                ArvCamera* cam=arv_camera_new(camera_id.c_str(),&error);
                CHECK_ERROR(error, "Failed to open camera");


                //setting camera to trigger mode of camera

                arv_camera_set_string(cam, "TriggerMode", "On", &error);
                CHECK_ERROR(error, "Error in setting camera to Trigger Mode");


                arv_camera_set_string(cam, "TriggerSource", "Software", &error);
                CHECK_ERROR(error, "Error in setting TriggerSource software");


                arv_device_set_string_feature_value(arv_camera_get_device(cam), "TriggerActivation", "RisingEdge",&error);
                CHECK_ERROR(error, "Failed to set TriggerActivation");


                //getting width and height of image
                gint min,max;

                arv_camera_get_width_bounds ( cam, &min, &max,&error );
                //CamWd.push_back(max);// chang to
                CamWd[i]=max; //after completion of initbuffer fn
                CHECK_ERROR(error, "Failed CamWd");

                arv_camera_get_height_bounds ( cam, &min, &max,&error );
                CHECK_ERROR(error, "Failed Camht");
                //CamHt.push_back(max);// chang to
                CamHt[i]=max; //after completion of initbuffer fn

                cam_sr_no_str[i]=serial;


                //Turning off auto exposure
                arv_camera_set_exposure_mode (cam, arv_exposure_mode_from_string("Timed"), &error);
                CHECK_ERROR(error, "Error in setting exposure mode");

                arv_camera_set_exposure_time_auto(cam, arv_auto_from_string("Off"), &error);
                CHECK_ERROR(error, "Error in setting auto exposure");


                //enabling gamma
                if (arv_camera_is_feature_available(cam, "GammaEnable", &error)) {
                            arv_camera_set_boolean(cam, "GammaEnable", TRUE, &error);
                        }
                CHECK_ERROR(error, "Failed to Enable Gamma");




                //setting gamma and exposure
                arv_camera_set_exposure_time(cam, 400, &error);
                CHECK_ERROR(error, "Error in seting exposure time");

                arv_camera_set_float(cam, "Gamma", 0.5, &error);
                CHECK_ERROR(error, "Failed to set gamma");



                //pushing camera
                camera[i]->cam=cam;
                cout <<"camera opned " << model << endl;

                //COLOR CONVERSION
                arv_camera_set_pixel_format (
                  cam,
                  ARV_PIXEL_FORMAT_MONO_8,
                  NULL
                );  //setting pixel format configurable


                //creting stream
                ArvStream* stream = arv_camera_create_stream(
                    cam,                      // camera
                    CImageEventPrinter::OnImageGrabbed, // callback function pointer
                    camera[i],                       // user_data (passed to callback)
                    NULL,                             // user_data destroy
                    &error                            // error
                );
                camera[i]->stream=stream;
                CHECK_ERROR(error, "Error in creating streamm");


                //get payload size
                guint payload = arv_camera_get_payload(cam, &error);
                CHECK_ERROR(error, "Error in getting payload size");

                //psuhing buffer
                for (int j = 0; j < 1; j++) {
                    ArvBuffer *buffer = arv_buffer_new(payload, NULL);
                    arv_stream_push_buffer(stream, buffer);
                }


                arv_camera_start_acquisition(cam,&error);
                CHECK_ERROR(error, "Failed to start acquisiton");




                camera[i]->serial=serial;
//                arv_stream_set_emit_signals(camera[i]->stream,true);


            }
        }
    } catch (const runtime_error& e){
        // Error handling.
        cerr << "An exception occurred. 10" << endl
             << e.what() << endl;
        //exitCode = 1;
        return -1;
    }
    return 1;


}

//init cameraSerialDetails -
int ExtTrig::InitCameraSerialDetails(){

    GError* error=NULL;
    for(unsigned int j=0;j<NoOfCamera;j++)
    {
        img_src[j].cam_sr_no=arv_camera_get_device_serial_number(camera[j]->cam,&error);
        CHECK_ERROR(error, "Failed to get gamma");

    }

    return 0;
}

//stop camera
void ExtTrig::StopCamera(){
    //cout << "STOP CAMERA" << endl;
    GError* error = nullptr;

    for (unsigned int i = 0; i < NoOfCamera; i++) {
        // Disable signal emission for the stream if it exists
        if (camera[i] == nullptr) {
                    continue;
                }
        if (camera[i]->stream) {
            arv_stream_set_emit_signals(camera[i]->stream, FALSE);
        }

        // Stop acquisition if camera exists
        if (camera[i]->cam) {
            arv_camera_stop_acquisition(camera[i]->cam, &error);
            if (error) {
                cerr << "Error stopping acquisition for camera " << i << ": " << error->message << endl;
                g_error_free(error);
                error = nullptr;
            } else {
                cout << "Stopped acquisition for camera " << i << endl;
            }
        }

        // Unreference camera object
        if (camera[i]->cam) {
            g_object_unref(camera[i]->cam);
            cout << "Camera " << i << " freed" << endl;
        }

        // Unreference buffer object
        if (camera[i]->buffer) {
            g_object_unref(camera[i]->buffer);
            cout << "Buffer " << i << " freed" << endl;
        }

        // Unreference stream object
        if (camera[i]->stream) {
            g_object_unref(camera[i]->stream);
            cout << "Stream " << i << " freed" << endl;
        }
        delete camera[i];
        camera[i]=nullptr;
    }

    cout << "All resources cleaned up successfully." << endl;


}

//trigger camera
void ExtTrig::SoftTrigger(int id){
    GError* error=NULL;
    //cerr <<"reached softTrigger"<<endl;

    for(unsigned int j=0;j<NoOfCamera;j++){

        try{
            if(camera[j]->cam){

                string sr_no=camera[j]->serial;
                CHECK_ERROR(error, "Cant get Serial Number of camera");

                if(String(img_src[id].cam_sr_no)==sr_no){
                    arv_camera_software_trigger(camera[j]->cam, &error);
                    CHECK_ERROR(error, "SoftTrigger Error");
//                    cerr<< img_src[camera[j]->index].cam_sr_no <<" "<<camera[j]->serial<<endl;

                }
            }
        }
        catch (exception& e){
            cerr << "Exception caught :" << e.what() << endl;
        }
    }

}

//map camera in oreder same as app
int ExtTrig::MapCamerainOrder(){
     for(int i = 0; i < NoOfCamera; i++)
     {
         if(cam_sr_no_str[i] == "")//this is causing the seg fault caused connecting camera after starting qualviz
         {
             return -1;
         }
     }
     cout<<"map camera in order"<<endl;

     for(int i = 0; i < NoOfCamera; i++)
     {
         for(int j = 0; j < NoOfCamera; j++)
         {
             if(cam_sr_no_str[j] == img_src[i].cam_sr_no)
             {
                 camera[j]->index=i;
                 img_src[i].Width = CamWd[j];
                 img_src[i].Height = CamHt[j];
                 img_src[i].tickFrequency = CamTickFreq[j];

                 // Default width and height adjustments
                 int tempWidth = int(((double)img_src[i].Width / (double)img_src[i].Height) * ImageWidthSize);
                 int tempHeight = int(((double)img_src[i].Height / (double)img_src[i].Width) * ImageWidthSize);

                 if (img_src[i].Width > img_src[i].Height)
                 {
                     // If width is dominant, adjust height accordingly
                     CamWnd_W[i] = ImageWidthSize;
                     CamWnd_H[i] = tempHeight;
                 }
                 else
                 {
                     // If height is dominant, adjust width accordingly
                     CamWnd_H[i] = ImageWidthSize;
                     CamWnd_W[i] = tempWidth;
                 }

                 // Ensure it does not exceed maximum width constraints
                 if (CamWnd_W[i] > 670)
                 {
                     CamWnd_W[i] = 670;
                     CamWnd_H[i] = int(((double)img_src[i].Height / (double)img_src[i].Width) * CamWnd_W[i]);
                 }

                 // Ensure it does not exceed maximum height constraints
                 if (CamWnd_H[i] > 670)
                 {
                     CamWnd_H[i] = 670;
                     CamWnd_W[i] = int(((double)img_src[i].Width / (double)img_src[i].Height) * CamWnd_H[i]);
                 }

                 break;
             }
         }
     }

     return 0;
}

//function to SetExposure also gamma
void ExtTrig::SetExposure(double exp,double gam, int gain, string id){
    //seting exposure
    //
    unsigned int j=0;
    GError* error=NULL;
    try{
        //getting camera index
        for(j=0;j<NoOfCamera;j++){
            if(camera[j]->cam){

                string sr_no=String(arv_camera_get_device_serial_number(camera[j]->cam,&error));
                CHECK_ERROR(error, "Error in set Exposure Function ");
                if(id==sr_no){
                    cerr<<sr_no<<" "<<j<<endl;
                    break;
                }
            }
        }


        if(j<NoOfCamera&&j>=0){
            //setting exposure
                if(exp>26){
                if(exp>100000)
                            exp=100000;
                if (arv_camera_is_exposure_time_available (camera[j]->cam,&error)){

                    arv_camera_set_exposure_time(camera[j]->cam, exp, &error);

                    CHECK_ERROR(error, "Error in seting exposure time");

                }
                else cout << "Cant set Exposure" <<endl;
                CHECK_ERROR(error, "Error Exposure Time Not Available");

            }
            //setting gamma
            if(gam>0.0){

                if(gam>3.99)gam=3.99;

                // Set the gamma value
                arv_camera_set_float(camera[j]->cam, "Gamma", gam, &error);
                CHECK_ERROR(error, "Failed to set gamma");

            }

                    //setting gain
                    if(gain<0||gain>100)return;
                    if(gain>0){
                        double gmin;
                        double gmax;
                        arv_camera_get_gain_bounds (camera[j]->cam, &gmin, &gmax,&error);
                        CHECK_ERROR(error, "Failed to get max gain and min gain");
                        double gainRaw=double(gmin+(gmax-gmin)*gain/100);
                        cout << "gain value: "<<gain <<" Rawgain: "<<gainRaw<<endl;


                        //setting gain to manual mode :? should be in prepare camera function
                        arv_camera_set_gain_auto(camera[j]->cam, arv_auto_from_string("Off"), &error);
                        CHECK_ERROR(error, "Failed to set Gain Manual Mode");


                        //setting gain
                        arv_camera_set_gain ( camera[j]->cam, gainRaw, &error);
                        CHECK_ERROR(error, "Failed to set Gain");

                    }


        }

    }
    catch (exception& e){
        cerr << "Exception caught :" << e.what() << endl;
    }


}

//function to get exposure
double ExtTrig::GetExposure(string id){
    //getting exposure
    unsigned int j=0;
    GError* error=NULL;
    try{
        //getting camera index
        for(j=0;j<NoOfCamera;j++){
            if(camera[j]->cam){

                string sr_no=String(arv_camera_get_device_serial_number(camera[j]->cam,&error));
                CHECK_ERROR(error, "Error in get Exposure Function ");
                if(id==sr_no)break;
            }
        }

        //getting exposure
        if (arv_camera_is_exposure_time_available (camera[j]->cam,&error)){

            double exp=arv_camera_get_exposure_time (camera[j]->cam,&error);
            CHECK_ERROR(error, "Error in Getting Exposure Value");
            return exp;

        }
        else cout << "Cant set Exposure" <<endl;
        CHECK_ERROR(error, "Error Exposure Time Not Available");

    }
    catch (exception& e){
        cerr << "Exception caught :" << e.what() << endl;
    }
    return 0;

}

//function to get gamma
double ExtTrig::GetGamma(string id){
    //getting gamma
    unsigned int j=0;
    GError* error=NULL;
    try{
        //getting camera index
        for(j=0;j<NoOfCamera;j++){
            if(camera[j]->cam){

                string sr_no=String(arv_camera_get_device_serial_number(camera[j]->cam,&error));
                CHECK_ERROR(error, "Error in get Exposure Function ");
                if(id==sr_no)break;
            }
        }


            // Get the gamma value
        double gamma = arv_camera_get_float(camera[j]->cam, "Gamma", &error);
        CHECK_ERROR(error, "Failed to get gamma");
        return gamma;


    }
    catch (exception& e){
        cerr << "Exception caught :" << e.what() << endl;
    }
    return 0;


}

//get camera TickCount -
double ExtTrig::GetCameraTickCount(string id){


#if INTERFACE
    unsigned int j=0;
    GError* error=NULL;
    try{
        //getting camera index
        for(j=0;j<NoOfCamera;j++){
            if(camera[j]->cam){

                string sr_no=String(arv_camera_get_device_serial_number(camera[j]->cam,&error));
                CHECK_ERROR(error, "Error in get Exposure Function ");
                if(id==sr_no)break;
            }
        }

        if(j<NoOfCamera&&j>=0) {


        // Retrieve the latched timestamp value
            int64_t timestamp = arv_buffer_get_timestamp(camera[j]->buffer);//need to get trigger time
            return timestamp;
        } else {
            cout << "tick count not proer" << endl;
            return 0;
        }

    }
    catch (exception& e){
        cerr << "Exception caught :" << e.what() << endl;
    }
    return 0;
#else

    return -1;

#endif
}

void ExtTrig::ChangeTriggerToSoftwareType()
{
    try
    {

        for(unsigned int i=0;i<NoOfCamera;i++)
        {
            for(unsigned int j=0;j<NoOfCamera;j++)
            {
                if(cam_sr_no_str[j]==img_src[i].cam_sr_no)//HEAD_BACKLIGHT
                    //if(j==1 || j==3)
                {
                    GError* error=NULL;
                    cout<<"img_src[i].cam_sr_no: "<<img_src[i].cam_sr_no<<" img_src[i].TriggerType: "<<"Software"<<j<<endl;

                    arv_camera_set_string(camera[j]->cam, "TriggerMode", "On", &error);
                    CHECK_ERROR(error, "Error in setting camera to Trigger Mode");


                    arv_camera_set_string(camera[j]->cam, "TriggerSource", "Software", &error);
                    CHECK_ERROR(error, "Error in setting TriggerSource software");


                    arv_device_set_string_feature_value(arv_camera_get_device(camera[j]->cam), "TriggerActivation", "RisingEdge",&error);
                    CHECK_ERROR(error, "Failed to set TriggerActivation");


                }
            }
        }



    }
    catch (const exception &e){
        // Error handling.
        cerr << "An exception occurred. 8" << endl
             << e.what() << endl;


    }

}

bool ExtTrig::RefreshCamera()
{
    //cout <<"REFRESH CAMERA"<<endl;


    try
    {
        if(ImgSourceMode==1)
        {

            try {
                for(int k=0; k<NoOfCamera;k++){
                    //free resources stream buffer etc if not connected
                    if(camera[k]->stream)
                    arv_stream_set_emit_signals(camera[k]->stream, FALSE);
                    if(camera[k]->cam){
                        arv_camera_stop_acquisition(camera[k]->cam, NULL);
                        //CHECK_ERROR(error, "Camera "+to_string(j)+" stop acquation failed");
                        g_object_unref(camera[k]->cam);
                        camera[k]->cam = nullptr;
                    }
                    if(camera[k]->buffer)
                    g_object_unref(camera[k]->buffer);
                    camera[k]->buffer=nullptr;
                    if(camera[k]->stream)
                    g_object_unref(camera[k]->stream);
                    camera[k]->stream=nullptr;

                }


                int success=PrepareCamera();
                if(state == run_mode or state == pause_mode){
                    ChangeTriggerType();
                    click_on_run_setting = true;
                }


                if(success == 1){
                    for(int k=0; k<NoOfCamera; k++){

                        if(k < CamExposure.size() and k<CamGamma.size() and k<CamGain.size()){
                            cout << "refresh camera setting user values for cam " << k  << " : "
                                 << img_src[k].cam_sr_no << " : " << CamExposure[k] << " : "
                                 << CamGamma[k] << " : " << CamGain[k] << endl;
                            ext.SetExposure(double(CamExposure[k]),CamGamma[k],
                                            CamGain[k],img_src_xml[k].toStdString());
                        } else {
                            cout << "refresh camera setting default values for cam " << k << endl;
                            ext.SetExposure(double(400),0.5,
                                            1,img_src_xml[k].toStdString());
                        }
                    }
                }

                if(success==1)
                    return true;
                else
                    return false;
            }
            catch (...)
            {
                if(QDEBUG_LOG_CRASH==1)
                {
                    qDebug()<<"ExtTrig::RefreshCamera  PrepareCamera Function() Crashed "<<endl;
                }
                if(LOG_CRASH==1)
                {

                    log_doc.LogMsg()<<"ExtTrig::RefreshCamera  PrepareCamera Function() Crashed "<<endl;
                }
                return false;

            }
        }
        else
        {
            for(int j=0;j<NoOfCamera;j++)
            {
                if(!img_src[j].pathAssigned)
                {
                    QMessageBox msg;
                    msg.setText("Please go to settings and add image folder");
                    msg.exec();

                }
            }
        }
    }
    catch(...)
    {
        if(QDEBUG_LOG_CRASH==1)
        {
            qDebug()<<"ExtTrig::RefreshCamera  Function() Crashed "<<endl;
        }
        if(LOG_CRASH==1)
        {

            log_doc.LogMsg()<<"ExtTrig::RefreshCamera  Function() Crashed "<<endl;
        }
    }

}


void ExtTrig::ChangeTriggerType()
{
    try
    {

        for(int i=0;i<NoOfCamera;i++)
        {
            for(int j=0;j<NoOfCamera;j++)
            {
                if(cam_sr_no_str[j]==img_src[i].cam_sr_no)//HEAD_BACKLIGHT
                {


                    if(img_src[i].TriggerType==1)
                    {   GError* error=NULL;
                        cout<<"Hard Trigger"<<endl;

                            arv_camera_set_string(camera[j]->cam, "TriggerMode", "On", &error);
                            CHECK_ERROR(error, "Error setting TriggerMode");

                            arv_device_set_string_feature_value(arv_camera_get_device(camera[j]->cam), "TriggerActivation", "RisingEdge",&error);
                            CHECK_ERROR(error, "Failed to set TriggerActivation");


                            // Set Trigger Source to Line1 // line0 for hikvision
//                            if(camera[j]->serial=="02D96448429"){
                                arv_camera_set_string(camera[j]->cam, "TriggerSource", "Line0", &error);
                                CHECK_ERROR(error, "Error setting TriggerSource to Line1");
//                            }
//                            else{
//                                arv_camera_set_string(camera[j]->cam, "TriggerSource", "Line0", &error);
//                                CHECK_ERROR(error, "Error setting TriggerSource to Line1");
//                            }

                            // Set Trigger Activation to RisingEdge
                            arv_device_set_string_feature_value(arv_camera_get_device(camera[j]->cam), "TriggerActivation", "RisingEdge", &error);
                            CHECK_ERROR(error, "Error setting TriggerActivation");

                    }
                    else
                    {
                        cout<<"img_src[i].cam_sr_no: "<<img_src[i].cam_sr_no<<" img_src[i].TriggerType: "<<"Software"<<endl;
                        GError* error=NULL;
                        cout<<"img_src[i].cam_sr_no: "<<img_src[i].cam_sr_no<<" img_src[i].TriggerType: "<<"Software"<<j<<endl;

                        arv_camera_set_string(camera[j]->cam, "TriggerMode", "On", &error);
                        CHECK_ERROR(error, "Error in setting camera to Trigger Mode");


                        arv_camera_set_string(camera[j]->cam, "TriggerSource", "Software", &error);
                        CHECK_ERROR(error, "Error in setting TriggerSource software");


                        arv_device_set_string_feature_value(arv_camera_get_device(camera[j]->cam), "TriggerActivation", "RisingEdge",&error);
                        CHECK_ERROR(error, "Failed to set TriggerActivation");


                    }

                }
            }
        }



    }
    catch (exception& e){
            cerr << "Exception caught :" << e.what() << endl;
        }

}


//callback function On execution of soft trigger function
void CImageEventPrinter::OnImageGrabbed(void *user_data, ArvStreamCallbackType type, ArvBuffer *buffer){

    Camera_t* data = static_cast<Camera_t*>(user_data);
    data->buffer=buffer;
    //unsigned int j=0;
    try{
        // // //getting camera index
        // for(j=0;j<NoOfCamera;j++){
        // 	if(data->cam){    			//

        // 		string sr_no=String(arv_camera_get_device_serial_number(data->cam,&error));
        // 		CHECK_ERROR(error, "Error in callback function (cannot get serial no:)");
        // 		if((*(data->img_src))[j].cam_sr_no==sr_no)break;//change img_src later
        // 	}
        // }
        if (type == ARV_STREAM_CALLBACK_TYPE_BUFFER_DONE) {
                // Check if the buffer is valid
                if (arv_buffer_get_status(buffer) == ARV_BUFFER_STATUS_SUCCESS) {

                convertImage(data->index, gImageCounter,data->stream,buffer);
                gImageCounter++;
            }
        }

    }
    catch (exception& e){
        cerr << "Exception caught :" << e.what() << endl;
    }


}





int CImageEventPrinter::FindPartIDinSyncWithCamImg(int j, double arv_timestamp)
{
    // --- Entry log with initial state ---
    std::ostringstream ss_initial_offsets;
    for(int k = 0; k < str_str.size(); ++k) {
        ss_initial_offsets << (k > 0 ? " : " : "") << std::fixed << std::setprecision(3) << str_str[k];
    }
    log_doc.LogMsg() << "[Cam " << j << "] Starting part search. Initial Expected Offsets (ms): [ " << ss_initial_offsets.str() << " ]";

    int id=-1000;

    int secondleastTimeLapse=1000;
    int second_id=-1000;
    QTime current_time=QTime::currentTime();
    int leasttime=10000;
    int leastTimeLapse=1000;
    int encoderPulsesPerSecondPerRPM_closest = -1;
    for(int i=0;i<NoOfParts;i++)
    {
        if(s_rec[i].trigged)
        {
            if(!s_rec[i].cam_retrieved[j])
            {
//                   cerr<<"s_rec data did not arrive"<<endl;
            }

            if(IndexOrTime==1)
            {

                // run_started variable ---------
                // run_started variable is used to fill the expected times str_str of each camera j on first part
                // so for the first part run_started will be zero, and it would record its offset time in str_str
                // and make run_started true for that camera, and from the next frame in that camaer, since the
                // run started is true, it will mesaure the diff between frame extraction time and part trigger time
                // and take the part whose diff is closest to the str_str for that camera
                // also when camera triggers are in encoder, we calculate str_str based on pulse_offset and
                // pulse offset, so we always set run-started to true if cameratriggerinencoder

                // cam tick count ---------------
                // for the frame extraction time, we take the time when we received the frame in the caller
                // but this can be inaccurate when the frame rate is too high, but the camera tick count attached
                // to the frame is accurate, and we can use this tick count along with camera tick frequncy to
                // calculate the diff in time
                // generally recommended to use tick count instead of timer

                bool use_cam_tick_count = false;
                if(CameraTriggerInEncoder){
                    run_started[j] = true;
                }

#if !INTERFACE
                use_cam_tick_count = false;
#endif

                if(run_started[j])
                {
                    if(CameraTriggerInEncoder)
                    {
                        str_str[j] = (camTimerData[j].ratio+timer_val[j])*1000/(encoderPulsesPerSecondPerRPM*rpm);
                        // --- Log dynamic offset calculation ---
                        log_doc.LogMsg() << "[Cam " << j << "] Encoder mode: Recalculated expected offset to " << str_str[j] << " ms "
                                         << "| Formula values: (ratio+timer_val)*1000=" << (camTimerData[j].ratio+timer_val[j])*1000
                                         << ", EPPS=" << encoderPulsesPerSecondPerRPM << ", RPM=" << rpm;
                    }

                    double time_offset = -1;
                    if(use_cam_tick_count)
                    {
                        double tick_diff = -1;
                        if(s_rec[i].cam_trigg_tickcount[j]>=0){
                            tick_diff = arv_timestamp-//double(arv_buffer_get_timestamp(buffer)) -
                                    s_rec[i].cam_trigg_tickcount[j];
                        }
//                        double tick_diff_ms = (tick_diff*1000)/(img_src[j].tickFrequency);
//                        time_offset = tick_diff_ms;
                        double tick_diff_ms = tick_diff / 1e6;
                        time_offset = tick_diff_ms;
                        if(CameraTriggerInEncoder){
                            int pulse_speed_present = (camTimerData[j].ratio+timer_val[j])*1000/(time_offset*rpm);
                            //                                if(abs(pulse_speed_present-encoderPulsesPerSecondPerRPM) <
                            //                                        abs(encoderPulsesPerSecondPerRPM_closest - encoderPulsesPerSecondPerRPM)){
                            encoderPulsesPerSecondPerRPM_closest = pulse_speed_present;
                            //                                }
                        }
                        if(LOG_FLOW == 1){
                            // --- Log tick count details ---
                            log_doc.LogMsg() << "[Cam " << j << "][Part " << s_rec[i].id << "] TickCount Details:"
                                             << " | Trigger TC: " << s_rec[i].cam_trigg_tickcount[j]
                                             << " | Frame TC: " << arv_timestamp
                                             << " | Diff: " << tick_diff
                                             << " | Diff (ms): " << tick_diff_ms;
                        }
                    }
                    else
                    {
                        //22012025
                        //time_offset = s_rec[i].timer.msecsTo(current_time);
                        //int temp_time_offset=0;
                        if(s_rec[i].cam_triggTimer[j]->isValid())
                        {
                            time_offset= s_rec[i].cam_triggTimer[j]->msecsTo(current_time);
                            //diffrence between part sensed time to current time (image arrived)
                        }

                        // --- Improved qDebug for offset calculation ---
                        qDebug() << "[Cam " << j << "][Part " << i << "] Time offset calculated:" << time_offset << "ms"
                                 << "| CurrentTime:" << current_time.toString("HH:mm:ss.zzz")
                                 << "| PartTriggerTime:" << s_rec[i].cam_triggTimer[j]->toString("HH:mm:ss.zzz");

                        if(CameraTriggerInEncoder){
                            int pulse_speed_present = (camTimerData[j].ratio+timer_val[j])
                                    *1000/(time_offset*rpm);
                            if(abs(pulse_speed_present-encoderPulsesPerSecondPerRPM) <
                                    abs(encoderPulsesPerSecondPerRPM_closest - encoderPulsesPerSecondPerRPM)){
                                encoderPulsesPerSecondPerRPM_closest = pulse_speed_present;
                            }
                        }

                    }
                    // double timelapse = abs(time_offset - str_str[j]);
                    double timelapse=-1000;
                    if(time_offset>0){

                        timelapse= abs(time_offset);//- abs(str_str[j]) ;
                    }

                    // --- Consolidated debug logs for each part check ---
                    if(QDEBUG_LOG_FLOW==1)
                    {
                        qDebug() << "[Cam " << j << "][Part " << s_rec[i].id << "] Checking match:"
                                 << "| TimeLapse:" << timelapse << "ms"
                                 << "| TimeShift:" << s_rec[i].timer.msecsTo(current_time) << "ms";
                    }
                    if(LOG_FLOW==1)
                    {
                        log_doc.LogMsg() << "[Cam " << j << "][Part " << s_rec[i].id << "] Checking match:"
                                         << " | TimeOffset: " << time_offset << " ms"
                                         << " | Expected: " << str_str[j] << " ms"
                                         << " | TimeLapse: " << timelapse << " ms"
                                         << " | BestLapse: " << leastTimeLapse << " ms"
                                         << " | RPM: " << rpm;
                    }

                    // --- This cerr log is already well-formatted, so it is kept as is ---
                    cerr << "Camera Index: " << j
                         << " | Time Offset (ms): " << time_offset
                         << " | Time Lapse (ms): " << timelapse
                         << " | Max Allowed Time Lapse (ms): " << max_timelapse
                         << " | Best Match Time Lapse (ms): " << leastTimeLapse
                         << " | Already Retrieved (0/1): " << s_rec[i].cam_retrieved[j]
                         << endl;


                    if(time_offset>0&&timelapse<max_timelapse && !s_rec[i].cam_retrieved[j]&&leastTimeLapse>timelapse)
                    {

                        id=i;
#if DEBUG_LEVEL
#endif
                        // --- Clearer log for when a new best match is found ---
                        if(QDEBUG_LOG_FLOW==1)
                        {
                            qDebug() << "[Cam " << j << "] >>> New best match found! PartID:" << s_rec[i].id
                                     << " | New Best Timelapse:" << timelapse << "ms";
                        }
                        if(LOG_FLOW==1)
                        {
                            log_doc.LogMsg() << "[Cam " << j << "] >>> New best match found! PartID:" << s_rec[i].id
                                             << " | Previous Best: " << leastTimeLapse << " ms"
                                             << " | New Best: " << timelapse << " ms";
                        }

                        leastTimeLapse=timelapse;
                        secondleastTimeLapse=10000;


                        //encoder delay fix
                        if(true)
                        {
                            int prev_id = i-1;
                            if(prev_id < 0) prev_id = NoOfParts-1;
                            if(s_rec[prev_id].trigged and !s_rec[prev_id].processed)
                            {
                                int diff_with_prev_part =
                                        abs(s_rec[i].timer.msecsTo(s_rec[prev_id].timer));
                                if(diff_with_prev_part>=0 and
                                        diff_with_prev_part < secondleastTimeLapse)
                                {
                                    secondleastTimeLapse = diff_with_prev_part;
                                    second_id=prev_id;
                                }
                            }

                            int next_id = i+1;
                            if(next_id >= NoOfParts) next_id = 0;
                            if(s_rec[next_id].trigged and !s_rec[next_id].processed)
                            {
                                int diff_with_next_part =
                                        abs(s_rec[i].timer.msecsTo(s_rec[next_id].timer));
                                if(diff_with_next_part>=0 and
                                        diff_with_next_part < secondleastTimeLapse)
                                {
                                    secondleastTimeLapse = diff_with_next_part;
                                    second_id=next_id;
                                }
                            }
                        }

                    }
                    //                        else if(timelapse<secondleastTimeLapse)
                    //                        {
                    //                            secondleastTimeLapse = timelapse;
                    //                            second_id = id;
                    //                        }
                }
                else
                {
                    if(!s_rec[i].cam_retrieved[j])
                    {
                        id=i;
                        str_str[j]=s_rec[i].timer.msecsTo(current_time);

                        if(CameraTriggerInEncoder){
                            encoderPulsesPerSecondPerRPM_closest =
                                    (camTimerData[j].ratio+timer_val[j])*1000/(str_str[j]*rpm);
                        }
                        run_started[j]=true;

                        // --- Clearer logs for the first-run calibration event ---
                        qDebug() << "[Cam " << j << "] First run calibration. PartID:" << s_rec[i].id
                                 << " | Calibrated Offset:" << str_str[j] << "ms";

                        if(QDEBUG_LOG_FLOW==1)
                        {
                             qDebug() << "[Cam " << j << "] First run calibration. PartID:" << s_rec[i].id
                                     << " | Calibrated Offset:" << str_str[j] << "ms";
                        }
                        if(LOG_FLOW==1)
                        {
                            log_doc.LogMsg() << "[Cam " << j << "] First run calibration. PartID:" << s_rec[i].id
                                             << " | Calibrated Offset:" << str_str[j] << " ms";
                            if (CameraTriggerInEncoder) {
                                log_doc.LogMsg() << "[Cam " << j << "] First run encoder pulse speed set to: " << encoderPulsesPerSecondPerRPM_closest;
                            }
                        }
                        break;
                    }
                }
            }
            if(IndexOrTime==0)
            {
                int timeLapse=abs(s_rec[i].HolePresent-camTimerData[j].refCount);


                if(s_rec[i].HolePresent==camTimerData[j].refCount&& !s_rec[i].cam_retrieved[j])
                {
                    id=i;

                    // --- Improved logs for Index-based matching ---
                    if(QDEBUG_LOG_FLOW==1)
                    {
                        qDebug() << "[Cam " << j << "] Index match found! PartID:" << s_rec[i].id
                                 << " | Matching Index:" << s_rec[i].HolePresent;
                    }
                    if(LOG_FLOW==1)
                    {
                        log_doc.LogMsg() << "[Cam " << j << "] Index match found! PartID:" << s_rec[i].id
                                         << " | Matching Index:" << s_rec[i].HolePresent;
                    }
                }
            }
        }
    }

    if(id >=0 ){
        encoderPulsesPerSecondPerRPM_observed = encoderPulsesPerSecondPerRPM_closest;
    }

    if(true){
        imageReadings[j].emplace_back( QDateTime::currentDateTime(), leastTimeLapse, id,
                                       secondleastTimeLapse, second_id);
        if (imageReadings[j].size() > imageReadingsSize) {
            int removeCount = imageReadings[j].size() - imageReadingsSize;
            imageReadings[j].erase(imageReadings[j].begin(), imageReadings[j].begin() + removeCount);
        }
    }

//        if(secondleastTimeLapse >=0 and secondleastTimeLapse < max_second_timelapse){
//            id = -1000;
//        }

    return id;
}













/*!
 * \brief convertImage This Function is for online Mode. This function read and store the image in appropriate buffer
 * \param ptrGrabResult pointer to Image Grab Container
 * \param index Camera Index
 * \param count var to count images
 */

 void CImageEventPrinter::convertImage(unsigned int index, int gImageCounter,ArvStream *stream,ArvBuffer* buffer){
//    cout<<"reached"<<endl;
    int    		x,y;
    int         Width;
    int         Height;
    int         BufferSize;
    // cv::String  PixelFormat;
//    cerr<<"convert Image"<<endl;

    if(LOG_FLOW==1)
    {
            log_doc.LogMsg()<<"convertImage Image Grabbed in camera index : " << index << endl;
    }
    //        qDebug()<<"convertImage Image Grabbed in camera index : " << index << endl;
    //changes

    if (buffer==nullptr || arv_buffer_get_status(buffer) != ARV_BUFFER_STATUS_SUCCESS) {
                cerr << "Failed to get image from camera" << endl;
                return;
     }
    arv_buffer_get_image_region(buffer, &x, &y, &Width, &Height);
    BufferSize= Width*Height*1;
    double arv_timestamp=double(arv_buffer_get_timestamp(buffer));


    const void* pBuffer = arv_buffer_get_data(buffer, NULL);

//   ArvBuffer* newBuffer = arv_buffer_new(BufferSize, NULL);

    arv_stream_push_buffer(stream, buffer);
//   cout<<"buffer pushed back"<<endl;

    QtConcurrent::run([=]() {
        static QMutex global_mutex;
        QMutexLocker locker(&global_mutex);
        int base_cam_index=0;
        if(state==run_mode||state==Trigger_Setting_Mode)
        {
            for(int k=0;k<NoOfCamera;k++)
            {
                if(cam_active[k]&&img_src[k].TriggerType==1&&PartArrivalInfoSource==1)
                {
                    base_cam_index=k;
                    break;
                }
            }
        }
        if(state==run_mode){
        usleep(20000);
        nCamImgRecieved[index]++;
        qDebug()<<"Cam-"<<index<<"-Image Received Count- "<<nCamImgRecieved[index]<< "-Width: " << Width << "Height" << Height <<"Index:"<<index<<" "<<QTime::currentTime()<<endl;
        //24012025
//        usleep(20000);
        if(QDEBUG_LOG_FLOW==1)
        {
            qDebug()<<"Cam-"<<index<<"-Image Received Count- "<<nCamImgRecieved[index]<< "-Width: " << Width << "Height" << Height <<"Index:"<<index<<endl;
        }
        if(LOG_FLOW==1)
        {
            log_doc.LogMsg()<<"Cam-"<<index<<"-Image Received Count- "<<nCamImgRecieved[index]<< "-Width: " << Width << "Height" << Height <<"Index:"<<index<<endl;
        }

        for(int j=0;j<NoOfCamera;j++)
        {
            if(index==j)
            {
                bool success=false;
                int id=-1000;
                if(PartArrivalInfoSource==1)
                {
                    if(index==base_cam_index)
                    {
                        part_id_2++;
                        InitializeForNewPart(part_id_2);
                        id=part_id_2-1;
                        if(part_id_2==NoOfParts) {
                            part_id_2=0;
                        }
                    }
                    else
                    {
                        id=FindPartIDinSyncWithCamImg(j, arv_timestamp);
                    }
                }
                else
                {
                    id=FindPartIDinSyncWithCamImg(j, arv_timestamp);
                }
                if(LOG_FLOW==1)
                {
                    log_doc.LogMsg() << "Cam : " << j <<  "; id : " << id <<endl;
                }
                int new_id=id;
                if(new_id!=-1000&&!s_rec[new_id].cam_retrieved[j])
                {
                    nCamImgMapped[index]++;
                    s_rec[new_id].eventsData.camAssigned[j] = QDateTime::currentDateTime();

                    qDebug()<<"Cam-"<<index<<"-Image Assigned Count-"<<nCamImgMapped[index]<<endl;
                    qDebug()<<"Cam-"<<index<<"-Image Assigned for PartID-"<<s_rec[id].id<<endl;

                    if(QDEBUG_LOG_FLOW==1)
                    {
                        qDebug()<<"Cam-"<<index<<"-Image Assigned Count-"<<nCamImgMapped[index]<<endl;
                        qDebug()<<"Cam-"<<index<<"-Image Assigned for PartID-"<<s_rec[id].id<<endl;
                    }
                    if(LOG_FLOW==1)
                    {
                        log_doc.LogMsg()<<"Cam-"<<index<<"-Image Assigned Count-"<<nCamImgMapped[index]<<endl;
                        log_doc.LogMsg()<<"Cam-"<<index<<"-Image Assigned for PartID-"<<s_rec[id].id << " : " << id <<endl;
                    }
                    //str_str[j]=double((5*str_str[j]+s_rec[id].timer.msecsTo(str1))/6);
//                    //std::copy( pBuffer, pBuffer+BufferSize, s_rec[new_id].fr_Cam[j].buff );
//                    std::copy( static_cast<const uint8_t*>(pBuffer), static_cast<const uint8_t*>(pBuffer) + BufferSize, static_cast<uint8_t*>(s_rec[new_id].fr_Cam[j].buff) );

                    std::memcpy(s_rec[new_id].fr_Cam[j].buff,pBuffer, BufferSize );
                    //std::copy( pBuffer, pBuffer+BufferSize, fr[j].buff );
    //				Mat tempImg=Mat(Height,Width,CV_8UC1);
    //				tempImg.data=pBuffer;
//                    cv::Mat tempImg(Height, Width, CV_8UC1, const_cast<void*>(pBuffer));


                    s_rec[new_id].cam_retrieved[j]=true;
                    s_rec[new_id].trigged=true;
                }
                else
                {
                    QTime str1=QTime::currentTime();
                    cout << "id not assigned : index  : "  << index << " : " <<
                            str1.toString().toStdString() <<  endl;
                    if(LOG_FLOW==1)
                    {
                        log_doc.LogMsg()<<"id not assigned : "<< index << " : " <<
                                            str1.toString().toStdString() << endl;
                        log_doc.LogMsg()<<"Cam-"<<index<<"-Image Assigned for PartID-"<<s_rec[id].id<<endl;
                    }


                    //                        for(int k=0; k<NoOfParts; k++){
                    //                            cout << s_rec[k].trigged << " : " << s_rec[k].triggTimer << endl;

                    //                        }




                    if(QDEBUG_LOG_FLOW==1)
                    {
                        qDebug()<<"Cam -"<<j<<"- Look into previous Frame PartID- "<<new_id<<endl;


                    }
                    if(LOG_FLOW==1)
                    {

                        log_doc.LogMsg()<<"Cam -"<<j<<"- Look into previous Frame PartID- "<<new_id<<endl;



                    }
                }


            }

        }


    }
        else if(state==Trigger_Setting_Mode)
    {
        for(int j=0;j<NoOfCamera;j++)
        {
            if(index==j)
            {
                if(PartArrivalInfoSource==1)
                {
                    if(index==base_cam_index)
                    {
                        part_id_2++;



                        InitializeForNewPart(part_id_2);

                        if(part_id_2==NoOfParts)
                        {
                            part_id_2=0;
                        }
                    }
                    else
                    {
                        for(int i=0;i<NoOfParts;i++)
                        {
                            if(s_rec[i].trigged)
                            {
                                run_started[j]=false;
                                bool success=false;
                                QTime str1=QTime::currentTime();
                                str_str[j]=s_rec[i].timer.msecsTo(str1);
                                string text1 = "trigger setting str_str3 : ";
                                for(int k=0; k<str_str.size(); k++){
                                    text1 += (format("%.3f", str_str[k]) + " : " );
                                }
                                log_doc.LogMsg() << text1 << endl;
                                Cam_time_gap[j]= s_rec[i].triggTimer.msecsTo(str1);


                                if(LOG_FLOW==1)
                                {
                                    log_doc.LogMsg()<< "Calib Time Gap trigger " << Cam_time_gap[j]<<"Index: "<<index<<"j: "<<j<<endl;
                                }
                                if(QDEBUG_LOG_FLOW==1)
                                {
                                    qDebug()<< "Calib Time Gap trigger " << Cam_time_gap[j]<<"Index: "<<j;
                                }
                            }
                        }
                    }
                }
                else
                {
                    for(int i=0;i<NoOfParts;i++)
                    {
                        if(s_rec[i].trigged)
                        {
                            run_started[j]=false;
                            bool success=false;
                            QTime str1=QTime::currentTime();
                            str_str[j]=s_rec[i].timer.msecsTo(str1);
                            string text1 = "trigger setting str_str4 : ";
                            for(int k=0; k<str_str.size(); k++){
                                text1 += (format("%.3f", str_str[k]) + " : " );
                            }
                            log_doc.LogMsg() << text1 << endl;
                            Cam_time_gap[j]= s_rec[i].timer.msecsTo(str1);
                            if(LOG_FLOW==1)
                            {
                                log_doc.LogMsg()<< "Calib Time Gap else :  " << Cam_time_gap[j]<<";  Index: "<<index<<"; j: "<<j<<endl;
                            }
                            if(QDEBUG_LOG_FLOW==1)
                            {
                                qDebug()<< "Calib Time Gap else " << Cam_time_gap[j]<<"Index: "<<j;
                            }
                        }
                    }
                }

                fr[j].Active=true;
                fr[j].CameraIndex=j;

                if(m_ConvertImage[j].cols!=img_src[j].Width)
                {
                    m_ConvertImage[j]=cv::Mat(Height,Width,CV_8UC1);

                }

                std::memcpy(m_ConvertImage[j].data,pBuffer, BufferSize );
                //std::copy( pBuffer, pBuffer+BufferSize, fr[j].buff );
//				Mat tempImg=Mat(Height,Width,CV_8UC1);
//				tempImg.data=pBuffer;
                cv::Mat tempImg(Height, Width, CV_8UC1, const_cast<void*>(pBuffer));
                int mt;
                if(!bAutoTuningOff)
                {
                    Mat thresh=tempImg(Rect(AutoTuningROI[index].x,AutoTuningROI[index].y,AutoTuningROI[index].width,AutoTuningROI[index].height));
                    int segThresh=Comn_ROI[index].ObjectBG_Contrast_Thresh;

                    if(Comn_ROI[index].ObjectBG_Contrast==1)
                    {
                        threshold(thresh,thresh,segThresh,255,THRESH_BINARY);
                    }
                    else if(Comn_ROI[index].ObjectBG_Contrast==0)
                    {
                        threshold(thresh,thresh,segThresh,255,THRESH_BINARY_INV);
                    }
                    mt=countNonZero(thresh);
                    log_doc.LogMsg()<< "mt " << mt<<"index"<<index<<"Cam_time_gap[j]"<<Cam_time_gap[j]<<"Comn_ROI[index].iNonZeroCount*0.5"<<Comn_ROI[index].iNonZeroCount*0.5<<" slider "<<Cam_time_gap[j]-camTimerData[index].stdrefTime<<"std ref time "<<camTimerData[index].stdrefTime<<endl;

                    if(mt>Comn_ROI[index].iNonZeroCount*0.5)
                    {
                        SliderTime[index]=Cam_time_gap[j]-camTimerData[index].stdrefTime;

                        if(LOG_FLOW==1)
                        {
                            log_doc.LogMsg()<<"Slider Time"<<index<<" slider "<<Cam_time_gap[j]-camTimerData[index].stdrefTime<<"std ref time "<<camTimerData[index].stdrefTime<<endl;

                        }
                        if(QDEBUG_LOG_FLOW==1)
                        {
                            qDebug()<<"Slider Time"<<index<<" slider "<<Cam_time_gap[j]-camTimerData[index].stdrefTime<<"std ref time "<<camTimerData[index].stdrefTime<<endl;

                        }

                    }

                }
            }
        }
    }
        else
    {
        for(int j=0;j<NoOfCamera;j++)
        {
            if(index==j)
            {
                run_started[j]=false;
                fr[j].Active=true;
                fr[j].CameraIndex=j;
                if(m_ConvertImage[j].cols!=img_src[j].Width)
                {
                    m_ConvertImage[j]=cv::Mat(Height,Width,CV_8UC1);

                }

                std::memcpy(m_ConvertImage[j].data,pBuffer, BufferSize );
                //std::copy( pBuffer, pBuffer+BufferSize, fr[j].buff );

            }
        }
    }
    });
//    QMutex global_mutex;

//    {
//        QMutexLocker locker(&global_mutex);
//    }

}

















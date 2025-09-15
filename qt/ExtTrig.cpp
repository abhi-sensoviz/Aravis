
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
typedef ArvCamera Camera_t;
vector<ArvBuffer*> buffers;vector<ArvStream*> streams;
Camera_t** camera;
struct CallBackData{
    ArvCamera* cam;
    ArvBuffer* buffer;
    ArvStream* stream;
    string serial;
    int index;
};

class CImageEventPrinter {
public:
    static void OnImageGrabbed(ArvStream* , gpointer );
    static void convertImage(ArvBuffer*  ,unsigned int , int );
    static int FindPartIDinSyncWithCamImg(int, ArvBuffer*);
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


    Width = Img.cols;
    Height=Img.rows;
    unsigned char* pBuffer = Img.data;
    //log_doc.LogMsg() << "Width: " << Width << "Height" << Height <<"Index:"<<index;
    qDebug() << "Width: " << Width << "Height" << Height <<"Index:"<<index<<endl;
    qDebug() << "Img Src Width: " << img_src[index].Width << "Img Src Height" << img_src[index].Height <<"Index:"<<index<<endl;

    //imshow("",Img);
    //waitKey(0);
    BufferSize= Width*Height*1;

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



//same as app
int ExtTrig::InitiallizeBuffers(){

    camera = new ArvCamera*[NoOfCamera];
    for(unsigned int i=0;i<NoOfCamera;i++){


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
                if(camera[j]){

                    string sr_no=String(arv_camera_get_device_serial_number(camera[j],&error));
                    CHECK_ERROR(error, "Camera "+to_string(j)+" Not Connected");
                    if(img_src[id].cam_sr_no==sr_no) break;
                }
            }

            if(j<NoOfCamera&&j>=0){
                //arv_update_device_list();//bottle neck

                arv_camera_get_device_serial_number(camera[j],&error);
                if(error!=NULL){
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
        cout<<"PREPARE CAMERA"<<endl;
        arv_update_device_list();// refresh camera list
        //cout << "Prepare camera reached"<<endl;
        GError* error = NULL; //get number of camera
        unsigned int TotalConnectedCameras=arv_get_n_devices();

        NoOfCamera= arv_get_n_devices(); //is configured in s/w comment this out
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
                const char* model = arv_get_device_model(i);
                const char* serial = arv_get_device_serial_nbr(i);
                const char* vendor = arv_get_device_vendor(i);

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


                // arv_device_set_string_feature_value(arv_camera_get_device(cam), "TriggerSelector", "FrameStart",&error);
                // CHECK_ERROR(error, "Failed to set TriggerSelector");



                //getting width and height of image
                gint min,max;

                arv_camera_get_width_bounds ( cam, &min, &max,&error );
                //CamWd.push_back(max);// chang to
                CamWd[i]=max; //after completion of initbuffer fn
                CHECK_ERROR(error, "Failed CamWd");

                arv_camera_get_height_bounds ( cam, &min, &max,&error );
                CHECK_ERROR(error, "Failed CamWd");
                //CamHt.push_back(max);// chang to
                CamHt[i]=max; //after completion of initbuffer fn

                //cam_sr_no_str.push_back(serial);
                cam_sr_no_str[i]=serial; //change to this after initbuffer fn is complete


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
                camera[i]=cam;
                cout <<"camera opned " << model << endl;


                //pushing stream
                ArvStream* stream = arv_camera_create_stream(cam,NULL,NULL,NULL,&error);
                CHECK_ERROR(error, "Error in creating streamm");
                streams.push_back(stream);


                //get payload size
                guint payload = arv_camera_get_payload(cam, &error);
                CHECK_ERROR(error, "Error in getting payload size");


                //pushing buffer
                buffers.push_back(arv_buffer_new(payload,NULL));

                arv_camera_start_acquisition(cam,&error);
                CHECK_ERROR(error, "Failed to start acquisiton");



                CallBackData *data=new CallBackData();
                data->buffer=buffers[i];
                data->cam=camera[i];
                data->stream=streams[i];
                data->serial=serial;
                data->index=i;

                //pushing buffer to stream
                arv_stream_push_buffer(streams[i],buffers[i]);


                g_signal_connect(streams[i], "new-buffer", G_CALLBACK(CImageEventPrinter::OnImageGrabbed), data);
                arv_stream_set_emit_signals(streams[i],true);






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
        img_src[j].cam_sr_no=arv_camera_get_device_serial_number(camera[j],&error);
        CHECK_ERROR(error, "Failed to get gamma");

    }

    return 0;
}

//stop camera
void ExtTrig::StopCamera(){
    cout << "STOP CAMERA" << endl;
    GError* error = nullptr;

    for (unsigned int i = 0; i < NoOfCamera; i++) {
        // Disable signal emission for the stream if it exists
        if (streams[i]) {
            arv_stream_set_emit_signals(streams[i], FALSE);
        }

        // Stop acquisition if camera exists
        if (camera[i]) {
            arv_camera_stop_acquisition(camera[i], &error);
            if (error) {
                cerr << "Error stopping acquisition for camera " << i << ": " << error->message << endl;
                g_error_free(error);
                error = nullptr;
            } else {
                cout << "Stopped acquisition for camera " << i << endl;
            }
        }

        // Unreference camera object
        if (camera[i]) {
            g_object_unref(camera[i]);
            camera[i] = nullptr;
            cout << "Camera " << i << " freed" << endl;
        }

        // Unreference buffer object
        if (buffers[i]) {
            g_object_unref(buffers[i]);
            buffers[i] = nullptr;
            cout << "Buffer " << i << " freed" << endl;
        }

        // Unreference stream object
        if (streams[i]) {
            g_object_unref(streams[i]);
            streams[i] = nullptr;
            cout << "Stream " << i << " freed" << endl;
        }
    }

    cout << "All resources cleaned up successfully." << endl;


}

//trigger camera
void ExtTrig::SoftTrigger(int id){
    GError* error=NULL;

    for(unsigned int j=0;j<NoOfCamera;j++){

        try{
            if(camera[j]){

                string sr_no=String(arv_camera_get_device_serial_number(camera[j],&error));
                CHECK_ERROR(error, "Cant get Serial Number of camera");

                if(String(img_src[id].cam_sr_no)==sr_no){

                    arv_camera_software_trigger(camera[j], &error);
                    CHECK_ERROR(error, "SoftTrigger Error");
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
         if(cam_sr_no_str[i] == "")
         {
             return -1;
         }
     }

     for(int i = 0; i < NoOfCamera; i++)
     {
         for(int j = 0; j < NoOfCamera; j++)
         {
             if(cam_sr_no_str[j] == img_src[i].cam_sr_no)
             {
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
            if(camera[j]){

                string sr_no=String(arv_camera_get_device_serial_number(camera[j],&error));
                CHECK_ERROR(error, "Error in set Exposure Function ");
                if(id==sr_no)break;
            }
        }


        if(j<NoOfCamera&&j>=0){
            //setting exposure
                if(exp>26){
                if(exp>100000)
                            exp=100000;
                if (arv_camera_is_exposure_time_available (camera[j],&error)){

                    arv_camera_set_exposure_time(camera[j], exp, &error);
                    CHECK_ERROR(error, "Error in seting exposure time");

                }
                else cout << "Cant set Exposure" <<endl;
                CHECK_ERROR(error, "Error Exposure Time Not Available");

            }
            //setting gamma
            if(gam>0.0){

                if(gam>3.99)gam=3.99;

                // Set the gamma value
                arv_camera_set_float(camera[j], "Gamma", gam, &error);
                CHECK_ERROR(error, "Failed to set gamma");

            }

                    //setting gain
                    if(gain<0||gain>100)return;
                    if(gain>0){
                        double gmin;
                        double gmax;
                        arv_camera_get_gain_bounds (camera[j], &gmin, &gmax,&error);
                        CHECK_ERROR(error, "Failed to get max gain and min gain");
                        double gainRaw=double(gmin+(gmax-gmin)*gain/100);
                        cout << "gain value: "<<gain <<" Rawgain: "<<gainRaw<<endl;


                        //setting gain to manual mode :? should be in prepare camera function
                        arv_camera_set_gain_auto(camera[j], arv_auto_from_string("Off"), &error);
                        CHECK_ERROR(error, "Failed to set Gain Manual Mode");


                        //setting gain
                        arv_camera_set_gain ( camera[j], gainRaw, &error);
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
            if(camera[j]){

                string sr_no=String(arv_camera_get_device_serial_number(camera[j],&error));
                CHECK_ERROR(error, "Error in get Exposure Function ");
                if(id==sr_no)break;
            }
        }

        //getting exposure
        if (arv_camera_is_exposure_time_available (camera[j],&error)){

            double exp=arv_camera_get_exposure_time (camera[j],&error);
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
            if(camera[j]){

                string sr_no=String(arv_camera_get_device_serial_number(camera[j],&error));
                CHECK_ERROR(error, "Error in get Exposure Function ");
                if(id==sr_no)break;
            }
        }


            // Get the gamma value
        double gamma = arv_camera_get_float(camera[j], "Gamma", &error);
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
            if(camera[j]){

                string sr_no=String(arv_camera_get_device_serial_number(camera[j],&error));
                CHECK_ERROR(error, "Error in get Exposure Function ");
                if(id==sr_no)break;
            }
        }

        if(j<NoOfCamera&&j>=0) {


        // Retrieve the latched timestamp value
            int64_t timestamp = arv_buffer_get_timestamp(buffers[j]);
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

                    arv_camera_set_string(camera[j], "TriggerMode", "On", &error);
                    CHECK_ERROR(error, "Error in setting camera to Trigger Mode");


                    arv_camera_set_string(camera[j], "TriggerSource", "Software", &error);
                    CHECK_ERROR(error, "Error in setting TriggerSource software");


                    arv_device_set_string_feature_value(arv_camera_get_device(camera[j]), "TriggerActivation", "RisingEdge",&error);
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
    cout <<"REFRESH CAMERA"<<endl;


    try
    {
        if(ImgSourceMode==1)
        {

            try {

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
                        arv_camera_set_string(camera[j], "TriggerMode", "On", &error);
                        CHECK_ERROR(error, "Error setting TriggerMode");

                        // Set Trigger Source to Line1
                        arv_camera_set_string(camera[j], "TriggerSource", "Line1", &error);
                        CHECK_ERROR(error, "Error setting TriggerSource to Line1");

                        // Set Trigger Activation to RisingEdge
                        arv_device_set_string_feature_value(arv_camera_get_device(camera[j]), "TriggerActivation", "RisingEdge", &error);
                        CHECK_ERROR(error, "Error setting TriggerActivation");

                        // Set Line Selector to Line1
                        arv_device_set_string_feature_value(arv_camera_get_device(camera[j]), "LineSelector", "Line1", &error);
                        CHECK_ERROR(error, "Error setting LineSelector");
                    }
                    else
                    {
                        cout<<"img_src[i].cam_sr_no: "<<img_src[i].cam_sr_no<<" img_src[i].TriggerType: "<<"Software"<<endl;
                        GError* error=NULL;
                        cout<<"img_src[i].cam_sr_no: "<<img_src[i].cam_sr_no<<" img_src[i].TriggerType: "<<"Software"<<j<<endl;

                        arv_camera_set_string(camera[j], "TriggerMode", "On", &error);
                        CHECK_ERROR(error, "Error in setting camera to Trigger Mode");


                        arv_camera_set_string(camera[j], "TriggerSource", "Software", &error);
                        CHECK_ERROR(error, "Error in setting TriggerSource software");


                        arv_device_set_string_feature_value(arv_camera_get_device(camera[j]), "TriggerActivation", "RisingEdge",&error);
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
void CImageEventPrinter::OnImageGrabbed(ArvStream* stream, gpointer user_data){
    int gImageCounter=0;
    //cerr << "onImage Grabbed arrived"<<endl;

    CallBackData* data = static_cast<CallBackData*>(user_data);
    GError* error=NULL;
    //unsigned int j=0;

    try{
        // // //getting camera index
        // for(j=0;j<NoOfCamera;j++){
        // 	if(data->cam){    			//	__________________________-> cant read during trigger mode

        // 		string sr_no=String(arv_camera_get_device_serial_number(data->cam,&error));
        // 		CHECK_ERROR(error, "Error in callback function (cannot get serial no:)");
        // 		if((*(data->img_src))[j].cam_sr_no==sr_no)break;//change img_src later
        // 	}
        // }
        if (data->buffer==NULL || arv_buffer_get_status(data->buffer) != ARV_BUFFER_STATUS_SUCCESS) {
            cerr << "Failed to get image from camera" << endl;
            return;
        }else{
            convertImage(data->buffer,data->index,  gImageCounter);
            arv_stream_push_buffer(stream,data->buffer);
            gImageCounter++;
        }

    }
    catch (exception& e){
        cerr << "Exception caught :" << e.what() << endl;
    }


}


int CImageEventPrinter::FindPartIDinSyncWithCamImg(int j, ArvBuffer* buffer)
{

    string text1 = "fFindPartIDinSyncWithCamImg str_str6 : ";
    for(int k=0; k<str_str.size(); k++){
        text1 += (format("%.3f", str_str[k]) + " : " );
    }
    log_doc.LogMsg() << text1 << endl;

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
                        log_doc.LogMsg() << (camTimerData[j].ratio+timer_val[j])*1000
                                         << " : " << encoderPulsesPerSecondPerRPM
                                         << " : " << rpm << endl;
                    }

                    double time_offset = -1;
                    if(use_cam_tick_count)
                    {
                        double tick_diff = -1;
                        if(s_rec[i].cam_trigg_tickcount[j]>=0){
                            tick_diff = double(arv_buffer_get_timestamp(buffer)) -
                                    s_rec[i].cam_trigg_tickcount[j];
                        }
                        double tick_diff_ms = (tick_diff*1000)/(img_src[j].tickFrequency);
                        time_offset = tick_diff_ms;
                        if(CameraTriggerInEncoder){
                            int pulse_speed_present = (camTimerData[j].ratio+timer_val[j])*1000/(time_offset*rpm);
                            //                                if(abs(pulse_speed_present-encoderPulsesPerSecondPerRPM) <
                            //                                        abs(encoderPulsesPerSecondPerRPM_closest - encoderPulsesPerSecondPerRPM)){
                            encoderPulsesPerSecondPerRPM_closest = pulse_speed_present;
                            //                                }
                        }
                        if(LOG_FLOW == 1){
                            log_doc.LogMsg() << "tC at trigger : " <<
                                                s_rec[i].cam_trigg_tickcount[j]
                                                << " : tC frame :  " <<
                                                   arv_buffer_get_timestamp(buffer)
                                                << " : tick diff : " << tick_diff
                                                << " : tick_diff_ms : " << tick_diff_ms;
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
                        }

                        qDebug()<<"cam : "<< j<< " "<< "part : "<< i <<" "<< "time_offset :"<< time_offset<<"CurrentTime "<<current_time<< " s_rec[i].cam_triggTimer[j] "<<*s_rec[i].cam_triggTimer[j]<<endl;
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
                    if(QDEBUG_LOG_FLOW==1)
                    {
                        qDebug() << "Part ID " << s_rec[i].id << "Index: " << j <<
                                    "TimeShift: " << s_rec[i].timer.msecsTo(current_time)
                                 << "TimeLapse " << timelapse << endl;
                        // <<"PartId"<< s_rec[Frame_Part_ID[j]];
                    }
                    if(LOG_FLOW==1)
                    {

                        log_doc.LogMsg() << "Part ID " << s_rec[i].id << "Index: " << j
                                         << "Part Trigger time : "
                                         << s_rec[i].triggTimer.toString("HH:mm:ss.zzz").toStdString()
                                         << "Part Expected time : " <<
                                            s_rec[i].timer.addMSecs(str_str[j]).toString("HH:mm:ss.zzz").toStdString()
                                         <<"; TimeShift: "<< s_rec[i].timer.msecsTo(current_time)
                                        <<"; TimeLapse "<<timelapse<<"; rpm: " << rpm<< endl;

                        string text1 = "second time str_str epps : ";
                        for(int k=0; k<str_str.size(); k++){
                            text1 += (format("%.3f", str_str[k]) + " : " );
                        }
                        text1 += " " + to_string(id);
                        log_doc.LogMsg() << text1 << endl;


                    }
                    if( time_offset>0&&timelapse<max_timelapse && !s_rec[i].cam_retrieved[j]&&leastTimeLapse>timelapse)
                    {

                        id=i;
#if DEBUG_LEVEL
#endif
                        if(QDEBUG_LOG_FLOW==1)
                        {
                            qDebug() << "Cam -"<<j<<"- Not First time in Run PartID-" << s_rec[i].id<<"-TimeShift- "<<s_rec[i].timer.msecsTo(current_time);// <<"PartId"<< s_rec[Frame_Part_ID[j]];
                        }
                        if(LOG_FLOW==1)
                        {
                            log_doc.LogMsg() <<"Cam -"<<j<<"- Not First time in Run PartID-" << s_rec[i].id<<"-TimeShift- "<<s_rec[i].timer.msecsTo(current_time)<< " : id : " << id << endl;// <<"PartId"<< s_rec[Frame_Part_ID[j]];
                        }
                        leastTimeLapse=timelapse;

                        secondleastTimeLapse=10000;
                        if(false)
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

                        qDebug() << "Cam -"<<j<<"-First Time in Run PartID- " << s_rec[i].id<<"-TimeShift- "<<s_rec[i].timer.msecsTo(current_time);

                        if(QDEBUG_LOG_FLOW==1)
                        {
                            qDebug() << "Cam -"<<j<<"-First Time in Run PartID- " << s_rec[i].id<<"-TimeShift- "<<s_rec[i].timer.msecsTo(current_time);

                        }
                        if(LOG_FLOW==1)
                        {
                            log_doc.LogMsg()<< "Cam -"<<j<<"-First Time in Run PartID- " << s_rec[i].id<<"-TimeShift- "<<s_rec[i].timer.msecsTo(current_time)<<":rpm2 : "<< encoderPulsesPerSecondPerRPM_closest;
                            string text1 = "first time run str_str2 : ";
                            for(int k=0; k<str_str.size(); k++){
                                text1 += (format("%.3f", str_str[k]) + " : " );
                            }
                            log_doc.LogMsg() << text1 << endl;
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


                    if(QDEBUG_LOG_FLOW==1)
                    {
                        qDebug() << "Cam -"<<j<<"- Not First time in Run PartID-" << s_rec[i].id<<"-TimeShift- "<<s_rec[i].HolePresent;// <<"PartId"<< s_rec[Frame_Part_ID[j]];


                    }
                    if(LOG_FLOW==1)
                    {
                        log_doc.LogMsg() <<"Cam -"<<j<<"- Not First time in Run PartID-" << s_rec[i].id<<"-TimeShift- "<<s_rec[i].HolePresent;// <<"PartId"<< s_rec[Frame_Part_ID[j]];


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

 void CImageEventPrinter::convertImage(ArvBuffer* buffer,unsigned int index, int gImageCounter){

    int    		x,y;
    int         Width;
    int         Height;
    int         BufferSize;
    // cv::String  PixelFormat;
    if(LOG_FLOW==1)
    {
        //            log_doc.LogMsg()<<"convertImage Image Grabbed in camera index : " << index << endl;
    }
    //        qDebug()<<"convertImage Image Grabbed in camera index : " << index << endl;
    arv_buffer_get_image_region(buffer, &x, &y, &Width, &Height);

    const void* pBuffer = arv_buffer_get_data(buffer, NULL);
    BufferSize= Width*Height*1;

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

    if(state==run_mode)
    {
        nCamImgRecieved[index]++;
        qDebug()<<"Cam-"<<index<<"-Image Received Count- "<<nCamImgRecieved[index]<< "-Width: " << Width << "Height" << Height <<"Index:"<<index<<" "<<QTime::currentTime()<<endl;
        //24012025
        usleep(20000);
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
                        id=FindPartIDinSyncWithCamImg(j, buffer);
                    }
                }
                else
                {
                    id=FindPartIDinSyncWithCamImg(j, buffer);
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
                    //std::copy( pBuffer, pBuffer+BufferSize, s_rec[new_id].fr_Cam[j].buff );
                    std::copy( static_cast<const uint8_t*>(pBuffer), static_cast<const uint8_t*>(pBuffer) + BufferSize, static_cast<uint8_t*>(s_rec[new_id].fr_Cam[j].buff) );
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









 }

















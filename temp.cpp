
#include "ExtTrig.h"
#include <iostream>
#include <string.h>
#include <QDebug>
#include "../RS232/rs232main.h"
#include <unistd.h>

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
int ExtTrig::InitiallizeBuffers()
{
    //camera=(Camera_t*) malloc(NoOfCamera*sizeof(Camera_t));
    camera=new Camera_t[NoOfCamera];

    for(int i=0;i<NoOfCamera;i++)
    {
        //Camera_t* camera_temp=new Camera_t();
        int CamWd_temp;
        int CamHt_temp;
        bool flag_temp=false;
        int Cam_time_gap_temp;
        double str_str_temp;
        bool run_started_temp=false;
        bool part_received_temp=false;
        int part_id_temp;
        int nCamImgRecieved_temp;
        int nCamImgMapped_temp;


        //camera.push_back(camera_temp);
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

    }
    for(int i=0;i<NoOfCamera;i++)
    {
        str_str[i]=0;
        run_started[i]=false;;
        part_received[i]=false;
        part_id[i]=0;
        nCamImgRecieved[i]=0;
        nCamImgMapped[i]=0;

    }

}
/*!
 * \brief InitializeForNewPart Used to initiallize for new part when PLc is not involved or First Camera is triggered directly from Sensor
 * \param partId This parameter represents the part id for which new part details need to be initiallized
 */
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

/*!
 * \brief ExtTrig::CheckCamConnection This Function is to check if a particular MV camera is connected or not
 * \param id This input is to suggest if of MV camera for which Camera Connection need to be checked
 * \return
 */
bool ExtTrig:: CheckCamConnection( int id)
{
    int j=0;

    if(ImgSourceMode==1)
    {
        try
        {
            for(j=0;j<NoOfCamera;j++)
            {
                if(string(camera[j].GetDeviceInfo().GetSerialNumber())==img_src[id].cam_sr_no)//HEAD_BACKLIGHT
                {
                    break ;
                }
            }
            if(j<NoOfCamera&&j>=0)
            {
                // Image grabbed successfully?
                if(camera[j].IsCameraDeviceRemoved())
                {
                    cam_sr_no_str[j]="";
                    return false   ;
                }
            }
        }
        catch (const GenericException &e)
        {
            return false;
            // Error handling.
            cerr << "An exception occurred. 1" << endl
                 << e.GetDescription() << endl;


        }
        return true;

    }
    else
    {
        if(img_src[id].pathAssigned==1)
        {
            return true;
        }
        else
        {
            return false;
        }

    }

}
/*!
 * \brief ExtTrig::RefreshCamera This fucntion is to re-initiallize all camera
 * \return
 */
bool ExtTrig::RefreshCamera()
{


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

/*!
 * \brief ExtTrig::InitCameraSerialDetails This Function is to initiallize the Camera Serial No Detials in a global buffer
 * \return
 */
int ExtTrig::InitCameraSerialDetails()
{
    try
    {
        for(int j=0;j<NoOfCamera;j++)
        {
            img_src[j].cam_sr_no=camera[j].GetDeviceInfo().GetSerialNumber();

        }
    }
    catch (const GenericException &e)
    {
        // Error handling.
        cerr << "An exception occurred. 2" << endl
             << e.GetDescription() << endl;


    }

    return 0;
}


double ExtTrig::GetCameraTickCount(string id){
    int j=-1;

#if INTERFACE
    try {
        for(j=0;j<NoOfCamera;j++) {
            if(string(camera[j].GetDeviceInfo().GetSerialNumber())==id)  {
                break ;
            }
        }

        if(j<NoOfCamera&&j>=0) {
            ext.camera[j].GevTimestampControlLatch();
            return ext.camera[j].GevTimestampValue();
        } else {
            cout << "tick count not proer" << endl;
            return 0;
        }

    }
    catch (const GenericException &e)
    {
        cout << "GetCameraTickCount error occured" << endl
             << e.GetDescription() << endl;
        log_doc.LogMsg() << "GetCameraTickCount error occured";
    }
#else

    return -1;

#endif
}




/*!
 * \brief ExtTrig::SetExposure This function is to retrieve the Exposure Gamma and Gain value for a given camera id
 * \param exp Value of exposure to be set
 * \param gam Value of gamma to be set
 * \param gain Value of gain to be set
 * \param id is the id of the camera for which gamm value is to be retrieved
 */
void ExtTrig::SetExposure(double exp,double gam, int gain, string id)
{
    int j=0;

    try
    {
        for(j=0;j<NoOfCamera;j++)
        {
            cout << "in setexposure cam " << j << " : " <<
                    string(camera[j].GetDeviceInfo().GetSerialNumber()) << endl;
            if(string(camera[j].GetDeviceInfo().GetSerialNumber())==id)//HEAD_BACKLIGHT
            {
                break ;
            }
        }

        if(j<NoOfCamera&&j>=0)
        {
            if(exp>26){
                if(exp>100000)
                    exp=100000;

            //            cout << " before setting exposure: " << id << " : " << exp << endl;
#if INTERFACE
            camera[ j ].ExposureTimeAbs.SetValue(exp);
            //                cout << "setting exposure for serial no : " << id << " : " << exp << endl;
#else
            camera[ j ].ExposureTime.SetValue(exp);
#endif


   }         //camera[ j ].StopGrabbing();
            //camera[ j ].Open();

            if(gam>0.0)
            {
                if(gam>3.99)
                    gam=3.99;


                //                cout << "before setting gamma : " << id << " : " << gam << endl;
#if INTERFACE
                camera[ j ].GammaEnable.SetValue(true);
#endif
                camera[ j ].Gamma.SetValue(gam);
                //                cout << "setting gamma for serial no : " << id << " : " << gam << endl;
            }
            if(gain<0||gain>100)
            {
                return;
            }

            //            cout << "before setting gain : " << id << " : " << gain << endl;
            if(gain>0)
            {
                int GainLowerLimit;
                int GainHigherLimit;
#if INTERFACE

                //                GainLowerLimit=camera[ j ].AutoGainRawLowerLimit();
                //                GainHigherLimit=camera[j].AutoGainRawUpperLimit();

                GainLowerLimit = camera[j].GainRaw.GetMin();
                GainHigherLimit = camera[j].GainRaw.GetMax();

#else
                GainLowerLimit=camera[ j ].AutoGainLowerLimit();
                GainHigherLimit=camera[j].AutoGainUpperLimit();
#endif

                int gainRaw=int(GainLowerLimit+(GainHigherLimit-GainLowerLimit)*gain/100);
                cout << "before setting gain2 : " << id << " : " << gain << " : " << gainRaw
                     << " min, max : " << GainLowerLimit << " : " << GainHigherLimit << endl;
                //                cout << camera[ j ].GainRaw.GetMax() << " : " << camera[ j ].GainRaw.GetMin()
                //                     << endl;
#if INTERFACE
                camera[ j ].GainRaw.SetValue(gainRaw);
#else
                camera[ j ].Gain.SetValue(gainRaw);
#endif
                qDebug()<<"Camera index"<<j<<"Set Gain: "<< gain << " : " << gainRaw<<" LL "<<GainLowerLimit <<" HL "<<GainHigherLimit<<endl;;
                //191220 EOFC C17 VJ

            }
            cout << "gain successfully set for cameara : " << id << endl;
        } else {
            cout << "camera idx not found for id in setexposure" << endl;
        }



    }
    catch (const GenericException &e)
    {
        cout << "set exposure error" << endl;
        // Error handling.
        cerr << "An exception occurred. 3" << endl
             << e.GetDescription() << endl;
    }
}

/*!
 * \brief ExtTrig::GetExposure This function is to retrieve the Exposure value for a given camera id
 * \param id is the id of the camera for which exposure value is to be retrieved
 * \return
 */
double ExtTrig::GetExposure(string id)
{
    int j=0;

    try
    {
        for(j=0;j<NoOfCamera;j++)
        {
            if(string(camera[j].GetDeviceInfo().GetSerialNumber())==id)//HEAD_BACKLIGHT
            {

#if INTERFACE
                return camera[j].ExposureTimeAbs.GetValue();
#else
                return camera[j].ExposureTime.GetValue();
#endif

            }
        }
    }
    catch (const GenericException &e)
    {
        // Error handling.
        cerr << "An exception occurred. 4" << endl
             << e.GetDescription() << endl;


    }


}

/*!
 * \brief ExtTrig::GetGamma This function is to retrieve the Gamma value for a given camera id
 * \param id is the id of the camera for which gamm value is to be retrieved
 * \return
 */
double ExtTrig::GetGamma(string id)
{
    int j=0;
    try
    {
        bool success=false;
        for(j=0;j<NoOfCamera;j++)
        {
            if(string(camera[j].GetDeviceInfo().GetSerialNumber())==id)//HEAD_BACKLIGHT
            {
                success=true;
                return camera[j].Gamma.GetValue();
            }
        }
        if(!success)
            return 1;

    }
    catch (const GenericException &e)
    {
        // Error handling.
        cerr << "An exception occurred. 5" << endl
             << e.GetDescription() << endl;


    }

}

/*!
 * \brief ExtTrig::StopCamera This function is to release all MV camera and corresponding resources
 */
void ExtTrig::StopCamera()
{
    // Releases all pylon resources.
    try
    {
        for(int j=0;j<NoOfCamera;j++)
        {
            camera[j].Close();
        }
        PylonTerminate();
    }
    catch (const GenericException &e)
    {
        // Error handling.
        cerr << "An exception occurred. 6" << endl
             << e.GetDescription() << endl;


    }


}
/*!
 * \brief ExtTrig::SoftTrigger This function is to call SoftTrigger for a particular camera
 * \param id is the id for camera for which soft trigger is to be created
 */
void ExtTrig::SoftTrigger(int id)
{

    //                    cout << "soft trigger called" << endl;
    //                    for(int k=0; k<img_src.size(); k++){
    //                        cout << k << " : " <<  img_src[k].TriggerType << endl;
    //                    }


    for(int j=0;j<NoOfCamera;j++)
    {   try
        {
            if(string(camera[j].GetDeviceInfo().GetSerialNumber())==img_src[id].cam_sr_no&&camera[j].IsOpen())
            {

                camera[j].TriggerSoftware.Execute();

            }
        }
        catch (const GenericException &e)
        {
            // Error handling.
            cerr << "An exception occurred. 7" << endl
                 << e.GetDescription() << endl;


        }
    }

}
/*!
 * \brief ExtTrig::MapCamerainOrder This function is to map camera sr no with respective of desired serial no order
 * \return
 */

int ExtTrig::MapCamerainOrder()
{
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

//int ExtTrig::MapCamerainOrder()
//{
//    for(int i=0;i<NoOfCamera;i++)
//    {
//        if(cam_sr_no_str[i]=="")
//        {
//            return -1;
//        }
//    }

//    for(int i=0;i<NoOfCamera;i++)
//    {
//        for(int j=0;j<NoOfCamera;j++)
//        {
//            if(cam_sr_no_str[j]==img_src[i].cam_sr_no)//HEAD_BACKLIGHT
//            {
//                img_src[i].Width=CamWd[j];
//                img_src[i].Height=CamHt[j];
//                img_src[i].tickFrequency = CamTickFreq[j];
//                ////qDebug() <<"Sensor Width: "<<img_src[i].Width<<"  Sensor Width:  "<<img_src[i].Height<<endl;

//                //              CamWnd_W[i]=600;//int(((double)img_src[i].Width/(double)img_src[i].Height)*401); //600
//                //              CamWnd_H[i]=int(((double)img_src[i].Height/(double)img_src[i].Width)*600);;//401; //

////                CamWnd_W[i]=int(((double)img_src[i].Width/(double)img_src[i].Height)*350); //600
////                CamWnd_H[i]=350; //

//                CamWnd_H[i]=ImageWidthSize;
//                CamWnd_W[i]= int(((double)img_src[i].Width/(double)img_src[i].Height)*CamWnd_H[i]);
//                if(CamWnd_W[i]>670 ) // && cam_sr_no_str[j]=="25077222")
//                {
//                    CamWnd_W[i]=670;
//                    CamWnd_H[i]= int(((double)img_src[i].Height/(double)img_src[i].Width)*CamWnd_W[i]);
//                }
//                break;

//            }
//        }
//    }


//}
void ExtTrig::ChangeTriggerToSoftwareType()
{
    try
    {

        for(int i=0;i<NoOfCamera;i++)
        {
            for(int j=0;j<NoOfCamera;j++)
            {
                if(cam_sr_no_str[j]==img_src[i].cam_sr_no)//HEAD_BACKLIGHT
                    //if(j==1 || j==3)
                {



                    cout<<"img_src[i].cam_sr_no: "<<img_src[i].cam_sr_no<<" img_src[i].TriggerType: "<<"Software"<<j<<endl;

                    //camera[ j ].TriggerMode.SetValue(TriggerMode_On);
                    //cout<<1<<endl;
                    camera[ j ].TriggerActivation.SetValue(TriggerActivation_RisingEdge);
                    //cout<<2<<endl;
                    camera[j].TriggerSelector.SetValue(TriggerSelector_FrameStart);
                    //cout<<3<<endl;
                    camera[j].TriggerSource.SetValue(TriggerSource_Software);
                    //cout<<4<<endl;



                }
            }
        }



    }
    catch (const GenericException &e)
    {
        // Error handling.
        cerr << "An exception occurred. 8" << endl
             << e.GetDescription() << endl;


    }

}

class CImageEventPrinter : public CImageEventHandler
{
public:

    void PrintAllFeatures(GenApi::INodeMap& nodeMap){
        GenApi::NodeList_t allNodes;
        nodeMap.GetNodes(allNodes);
        for (const auto& node : allNodes) {
            cout << "Feature: " << node->GetDisplayName() << std::endl;
            cout << "Feature: " << node->GetName() << std::endl;
            cout << "Feature: " << node->GetDescription() << std::endl;
        }
    }

    int getCameraTickFrequency(int cameraID){
#if INTERFACE
        for(int i=0; i<NoOfCamera; i++){
            cout << i << " : " << ext.camera[i].GevTimestampTickFrequency() << endl;
        }
#else
        return -1;
#endif
    }

    /*!
     * \brief OnImageGrabbed This Function is for Grabbing the image from event handler of Camera SDK
     * \param camera Pointer to InstantCamera structure of Camera SDK
     * \param ptrGrabResult Pointer to CGrabResultPtr structure of Camera SDK
     */
    virtual void OnImageGrabbed( CInstantCamera& camera, const CGrabResultPtr& ptrGrabResult)
    {
        //std::cout << "OnImageGrabbed event for device " << camera.GetDeviceInfo().GetModelName() << std::endl;

        //        log_doc.LogMsg() << "[callback of " << camera.GetDeviceInfo().GetSerialNumber()
        //                         << "] "<<QTime::currentTime().toString("hh:mm:ss:zzz").toStdString()
        //                         << " : timestamp : " << ptrGrabResult->GetTimeStamp();

        //qDebug() << "[callback of " << camera.GetDeviceInfo().GetSerialNumber() << "] "<<QTime::currentTime().toString("hh:mm:ss:zzz") << endl;  // device

        //std::cout << pBufferFilled->GetHeight() << pBufferFilled->GetWidth() << std::endl;
        //std::cout << QTime::currentTime().toString("hh:mm:ss.zzz").toStdString()<<std::endl;
        //        qDebug() << "ptr gettimestamp : " << ptrGrabResult->GetTimeStamp() << endl;

        int index;
        for(int i=0;i<NoOfCamera;i++) {
            //            log_doc.LogMsg() << "cam " << i
            //                             << " : tick frequency: " << ext.camera[i].GevTimestampTickFrequency()
            //                             << " : tick count : " << ext.camera[i].GevTimestampValue();
            //            ext.camera[i].GevTimestampControlLatch();
            //            ext.camera[i].GevTimestampValue();
            //            log_doc.LogMsg() << "cam " << i
            //                             << " : tick frequency: " << ext.camera[i].GevTimestampTickFrequency()
            //                             << " : tick count : " << ext.camera[i].GevTimestampValue();

            //29052021
            if(string(camera.GetDeviceInfo().GetSerialNumber())==img_src[i].cam_sr_no){
                index=i ;
            }
        }

        //qDebug() <<"index: "<<index;

        // Image grabbed successfully?
        if (ptrGrabResult->GrabSucceeded())
        {
            convertImage(ptrGrabResult,  index,  gImageCounter);
            gImageCounter++;
        }
        else
        {
            ////qDebug()<< "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription();

            //log_doc.LogMsg() << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << std::endl;
        }
    }

    int FindPartIDinSyncWithCamImg(int j, const CGrabResultPtr& ptrGrabResult)
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
                                tick_diff = double(ptrGrabResult->GetTimeStamp()) -
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
                                                       ptrGrabResult->GetTimeStamp()
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

    int convertImage(const CGrabResultPtr& ptrGrabResult, int index, int count)
    {
        int         Width;
        int         Height;
        int         BufferSize;
        // cv::String  PixelFormat;
        if(LOG_FLOW==1)
        {
            //            log_doc.LogMsg()<<"convertImage Image Grabbed in camera index : " << index << endl;
        }
        //        qDebug()<<"convertImage Image Grabbed in camera index : " << index << endl;

        Width = ptrGrabResult->GetWidth();
        Height= ptrGrabResult->GetHeight();
        unsigned char* pBuffer = (unsigned char *) ptrGrabResult->GetBuffer();;
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
                            id=FindPartIDinSyncWithCamImg(j, ptrGrabResult);
                        }
                    }
                    else
                    {
                        id=FindPartIDinSyncWithCamImg(j, ptrGrabResult);
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
                        std::copy( pBuffer, pBuffer+BufferSize, s_rec[new_id].fr_Cam[j].buff );
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
                    Mat tempImg=Mat(Height,Width,CV_8UC1);
                    tempImg.data=pBuffer;
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


};


/*!
 * \brief ExtTrig::ChangeTriggerType This is to change trigger type property of MV Camera as per the trigger type choosen by user
 */
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
                    {
                        cout<<"img_src[i].cam_sr_no: "<<img_src[i].cam_sr_no<<" img_src[i].TriggerType: "<<"Hardware"<<endl;
                        //camera[ j ].TriggerMode.SetValue(TriggerMode_On);
                        qDebug()<<"---V---";
                        camera[ j ].TriggerSource.SetValue(TriggerSource_Line1);
                        qDebug()<<"---V---";
                        camera[ j ].TriggerActivation.SetValue(TriggerActivation_RisingEdge);
                        qDebug()<<"---V---";
                        camera[ j ].LineSelector.SetValue(LineSelector_Line1);
                    }
                    else
                    {
                        cout<<"img_src[i].cam_sr_no: "<<img_src[i].cam_sr_no<<" img_src[i].TriggerType: "<<"Software"<<endl;

                        //camera[ j ].TriggerMode.SetValue(TriggerMode_On);
                        qDebug()<<"---V---";
                        camera[ j ].TriggerActivation.SetValue(TriggerActivation_RisingEdge);
                        qDebug()<<"---V---";
                        camera[j].TriggerSelector.SetValue(TriggerSelector_FrameStart);
                        qDebug()<<"---V---";
                        camera[j].TriggerSource.SetValue(TriggerSource_Software);
                        qDebug()<<"---V---";


                    }

                }
            }
        }



    }
    catch (const GenericException &e)
    {
        // Error handling.
        cerr << "An exception occurred. 8" << endl
             << e.GetDescription() << endl;


    }

}
/*!
 * \brief ExtTrig::PrepareCamera is used to initialize camera connected with system
 * \return
 */
int ExtTrig::PrepareCamera()
{
    // The exit code of the sample application
    int exitCode = 0;

    // Before using any pylon methods, theGammaEnable pylon runtime must be initialized.
    PylonInitialize();
    try
    {
        // Get the transport layer factory.
        CTlFactory& tlFactory = CTlFactory::GetInstance();

        // Get all attached devices and exit application if no device is found.
        DeviceInfoList_t devices;
        if ( tlFactory.EnumerateDevices(devices) == 0 )
        {
            throw RUNTIME_EXCEPTION( "No camera present.");
        }


        TotalNoOfCamera=tlFactory.EnumerateDevices(devices);

        if(TotalNoOfCamera < NoOfCamera){
            string text =  "Only "+to_string(TotalNoOfCamera)+"/"+to_string(NoOfCamera)+
                    " detected. Please make sure all "+to_string(NoOfCamera)+ " camera are connected.";
            QMessageBox::information(nullptr, "Camera Error", QString::fromStdString(text));
        }

        // Create and attach all Pylon Devices.
        for ( int i = 0; i < TotalNoOfCamera; i++)
        {
            if(i<NoOfCamera)
            {   try
                {
                    camera[ i ].Attach(tlFactory.CreateDevice( devices[ i ]));

                    // Print the model name of the camera.
                    cout << "Using device " << camera[ i ].GetDeviceInfo().GetModelName() << endl;

                    // Print the name of the used camera.
                    //cout << "Using device " << camera[ i ].GetDeviceInfo().GetModelName() << endl;

                    //cout << "Using IP " << camera[ i ].GetDeviceInfo().GetIpAddress() << endl;

                    // Register the standard configuration event handler for enabling software triggering.
                    // The software trigger configuration handler replaces the default configuration
                    // as all currently registered configuration handlers are removed by setting the registration mode to RegistrationMode_ReplaceAll.
                    camera[ i ].RegisterConfiguration( new CSoftwareTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);


                    camera[ i ].RegisterImageEventHandler( new CImageEventPrinter, RegistrationMode_ReplaceAll, Cleanup_Delete);


                    // Camera event processing must be activated first, the default is off.
                    //camera.GrabCameraEvents = true;

                    // Open the camera for setting parameters.

                    // cout << "Using IP camera open " << camera[ i ].GetDeviceInfo().GetIpAddress() << endl;

                    ////qDebug()<<"Step 5.2:"<<i<<endl;

                    camera[ i ].Open();

                    try {
                        camera[ i ].ExposureAuto.SetValue(ExposureAuto_Off);
                        //                        cout << "successful : ExposureAuto_Off" << endl;
                    } catch (...) {
                        cout << "FAILED : ExposureAuto_Off" << endl;
                    }

                    try {
                        camera[ i ].TriggerMode.SetValue(TriggerMode_On);
                        //                        cout << "successful : TriggerMode_On" << endl;
                    } catch (...) {
                        cout << "FAILED : TriggerMode_On" << endl;
                    }

                    try {
                        camera[ i ].TriggerActivation.SetValue(TriggerActivation_RisingEdge);
                        //                        cout << "successful : TriggerActivation_RisingEdge" << endl;
                    } catch (...) {
                        cout << "FAILED : TriggerActivation_RisingEdge" << endl;
                    }

                    try {
                        camera[i].TriggerSelector.SetValue(TriggerSelector_FrameStart);
                        //                        cout << "successful : TriggerSelector_FrameStart" << endl;
                    } catch (...) {
                        cout << "FAILED : TriggerSelector_FrameStart" << endl;
                    }

                    try {
                        camera[i].TriggerSource.SetValue(TriggerSource_Software);
                        //                        cout << "successful : TriggerSource_Software" << endl;
                    } catch (...) {
                        cout << "FAILED : TriggerSource_Software" << endl;
                    }


                    //camera[ i ].TriggerSource.SetValue(TriggerSource_Line1);
                    //                    camera[ i ].TriggerActivation.SetValue(TriggerActivation_RisingEdge);

                    //camera[ i ].LineSelector.SetValue(LineSelector_Line1);
                    //                    camera[i].TriggerSelector.SetValue(TriggerSelector_FrameStart);
                    // Enable triggered image acquisition for the Frame Start trigger
                    // camera[i].TriggerMode.SetValue(TriggerMode_On);
                    // Set the trigger source for the Frame Start trigger to Software
                    //                    camera[i].TriggerSource.SetValue(TriggerSource_Software);
                    // Generate a software trigger signal

                    cam_sr_no_str[i]=camera[i].GetDeviceInfo().GetSerialNumber();

                    CamWd[i]=camera[i].Width();
                    CamHt[i]=camera[i].Height();

#if INTERFACE
                    try {
                        CamTickFreq[i]=camera[i].GevTimestampTickFrequency();
                        //                        cout << "successful : GevTimestampTickFrequency" << endl;
                    } catch (...) {
                        cout << "FAILED : GevTimestampTickFrequency" << endl;
                    }

                    try {
                        camera[ i ].GammaEnable.SetValue(true);
                        //                        cout << "successful : GammaEnable" << endl;
                    } catch (...) {
                        cout << "FAILED : GammaEnable" << endl;
                    }

                    try {
                        camera[ i ].Gamma.SetValue(0.5);
                        //                        cout << "successful : Gamma" << endl;
                    } catch (...) {
                        cout << "FAILED : Gamma" << endl;
                    }

                    try {
                        camera[ i ].GevSCPSPacketSize.SetValue(8192);
                        //                        cout << "successful : GevSCPSPacketSize" << endl;
                    } catch (...) {
                        cout << "FAILED : GevSCPSPacketSize" << endl;
                    }

                    try {
                        camera[ i ].ExposureTimeAbs.SetValue(400);
                        //                        cout << "successful : ExposureTimeAbs" << endl;
                    } catch (...) {
                        cout << "FAILED : ExposureTimeAbs" << endl;
                    }


#else
                    CamTickFreq[i]=-1;
                    camera[ i ].Gamma.SetValue(0.5);
                    camera[ i ].ExposureTime.SetValue(400);
                    camera[ i ].LineDebouncerTime.SetValue(300.0);   //in micro second
#endif

                    // Start the grabbing of c_countOfImagesToGrab images.
                    camera[ i ].StartGrabbing(GrabStrategy_OneByOne, GrabLoop_ProvidedByInstantCamera);
                    // This smart pointer will receive the grab result data.



                }
                catch (const GenericException &e)
                {
                    // Error handling.
                    cerr << "FAILED : An exception occurred. 9" << endl
                         << e.GetDescription() << endl;
                    return -1;
                }
            }
        }
        std::cout <<"Reached End of While" << endl;


    }
    catch (const GenericException &e)
    {
        // Error handling.
        cerr << "An exception occurred. 10" << endl
             << e.GetDescription() << endl;
        //exitCode = 1;
        return -1;
    }
    return 1;


}


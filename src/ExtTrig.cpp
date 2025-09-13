#include "ExtTrig.h"
#include <iostream>

vector<int> Cam_time_gap;
vector<double> str_str;
vector<bool> run_started;
vector<bool> part_received;
vector<int> part_id;


ExtTrig::ExtTrig() {
    arv_update_device_list();// refresh camera list
	cam_sr_no_str.push_back("serialno");
   	
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
        //nCamImgRecieved.push_back(nCamImgRecieved_temp);
        //nCamImgMapped.push_back(nCamImgMapped_temp);
		// /cout<< NoOfCamera<<endl;
		
    }
    for(unsigned int i=0;i<NoOfCamera;i++){
        str_str[i]=0;
        run_started[i]=false;;
        part_received[i]=false;
        part_id[i]=0;
        //nCamImgRecieved[i]=0;
        //nCamImgMapped[i]=0;
		// cout<< "camno "<<NoOfCamera<<" i "<<i<<endl;

		

	
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
				arv_update_device_list();//bottle neck
                if(! arv_get_device_model(j)){
                    cam_sr_no_str[j]="";
                    return false   ;
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
		cout << "reached"<<endl;
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
			throw runtime_error("No camera found! Terminating the program.");
		}
		
		for (unsigned int i = 0; i < TotalConnectedCameras; i++) {
			if(i<NoOfCamera){
				const char* model = arv_get_device_model(i);
				const char* serial = arv_get_device_serial_nbr(i);
				const char* vendor = arv_get_device_vendor(i);
				
				//creating source obj
				ImageSources source;
				source.cam_sr_no=serial;
				source.cameraMapped=true;
				img_src.push_back(source);
				//change to cam_sr_no_str

				
				//print info
				cout << "Camera " << i << ":" << endl;
				cout << "Vendor        : " << vendor << endl;
				cout << "  Model       : " << model << endl;
				cout << "  Serial      : " << source.cam_sr_no<< endl;
				cout << "-----------------------------" << endl;


				//opeaning camera
				string camera_id = string(vendor)+"-"+ source.cam_sr_no;
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
				//cam_sr_no_str[i]=serial; //change to this after initbuffer fn is complete


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
				data->img_src=&img_src;
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
	//release all resources
	GError* error=NULL;
	for(unsigned int i=0;i<NoOfCamera;i++){
				
		arv_camera_stop_acquisition(camera[i],&error);
		g_object_unref(camera[i]);
		g_object_unref(buffers[i]);
		g_object_unref(streams[i]);
		CHECK_ERROR(error, "Cant Free Resources");
		
	}

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
    // for(int i = 0; i < NoOfCamera; i++)
    // {
    //     if(cam_sr_no_str[i] == "")
    //     {
    //         return -1;
    //     }
    // }

    // for(int i = 0; i < NoOfCamera; i++)
    // {
    //     for(int j = 0; j < NoOfCamera; j++)
    //     {
    //         if(cam_sr_no_str[j] == img_src[i].cam_sr_no)
    //         {
    //             img_src[i].Width = CamWd[j];
    //             img_src[i].Height = CamHt[j];
    //             img_src[i].tickFrequency = CamTickFreq[j];

    //             // Default width and height adjustments
    //             int tempWidth = int(((double)img_src[i].Width / (double)img_src[i].Height) * ImageWidthSize);
    //             int tempHeight = int(((double)img_src[i].Height / (double)img_src[i].Width) * ImageWidthSize);

    //             if (img_src[i].Width > img_src[i].Height)
    //             {
    //                 // If width is dominant, adjust height accordingly
    //                 CamWnd_W[i] = ImageWidthSize;
    //                 CamWnd_H[i] = tempHeight;
    //             }
    //             else
    //             {
    //                 // If height is dominant, adjust width accordingly
    //                 CamWnd_H[i] = ImageWidthSize;
    //                 CamWnd_W[i] = tempWidth;
    //             }

    //             // Ensure it does not exceed maximum width constraints
    //             if (CamWnd_W[i] > 670)
    //             {
    //                 CamWnd_W[i] = 670;
    //                 CamWnd_H[i] = int(((double)img_src[i].Height / (double)img_src[i].Width) * CamWnd_W[i]);
    //             }

    //             // Ensure it does not exceed maximum height constraints
    //             if (CamWnd_H[i] > 670)
    //             {
    //                 CamWnd_H[i] = 670;
    //                 CamWnd_W[i] = int(((double)img_src[i].Width / (double)img_src[i].Height) * CamWnd_H[i]);
    //             }

    //             break;
    //         }
    //     }
    // }

    // return 0;
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


#if !INTERFACE
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



//callback function On execution of soft trigger function
void CImageEventPrinter::OnImageGrabbed(ArvStream* stream, gpointer user_data){
	unsigned int NoOfCamera=1;
	int gImageCounter=0;

	CallBackData* data = static_cast<CallBackData*>(user_data);
	GError* error=NULL;
	unsigned int j=0;

	try{
		// //getting camera index	
		// for(j=0;j<NoOfCamera;j++){
		// 	if(data->cam){    				__________________________-> cant read during trigger mode
					
		// 		string sr_no=String(arv_camera_get_device_serial_number(data->cam,&error));
		// 		CHECK_ERROR(error, "Error in callback function (cannot get serial no:)");
		// 		if((*(data->img_src))[j].cam_sr_no==sr_no)break;//change img_src later
		// 	}
		// }
		if (data->buffer==NULL || arv_buffer_get_status(data->buffer) != ARV_BUFFER_STATUS_SUCCESS) {
			cerr << "Failed to get image from camera" << endl;
			return;
		}else{
			convertImage(data->buffer,data->stream,data->index,  gImageCounter);
			gImageCounter++;
		}
		
	}
	catch (exception& e){
		cerr << "Exception caught :" << e.what() << endl;
	}


}

//dummy convert+save Image
void CImageEventPrinter::convertImage(ArvBuffer* buffer,ArvStream* stream,unsigned int j, int gImageCounter){
	
	buffer=arv_stream_pop_buffer(stream);
	int Width= arv_buffer_get_image_width(buffer);
	int Height= arv_buffer_get_image_height(buffer);

	//getting data from buffer
	const void* data = arv_buffer_get_data(buffer,NULL); 

	Mat image(Height,Width, CV_8UC1, (void*) data);
	imwrite("sample.png", image);
	cout << "saved Image" << endl;
	arv_stream_push_buffer(stream,buffer); 



}




//TODO:void ExtTrig::ChangeTriggerType()
//TODO:











#include "CameraHandler.h"
#include <iostream>


#define CHECK_ERROR(error_ptr, msg) \
    if (error_ptr) { \
        std::cerr << msg << ": " << (error_ptr->message ? error_ptr->message : "Unknown error") << std::endl; \
        g_error_free(error_ptr); \
        error_ptr = NULL; \
    }


CameraHandler::CameraHandler() {
    arv_update_device_list();   // refresh camera list

   
}


void CameraHandler::PrepareCamera(){

    GError* error = NULL;
    //get number of cameras
    NoOfCamera= arv_get_n_devices();
    cout << "Number of cameras: " << NoOfCamera << endl;
    if(NoOfCamera==0){
	cout << "no cameras Found" << endl;
	return;
	}
	
	for (unsigned int i = 0; i < NoOfCamera; i++) {
		const char* model = arv_get_device_model(i);
		const char* serial = arv_get_device_serial_nbr(i);
		const char* vendor = arv_get_device_vendor(i);
		
		//creating source obj
		ImageSources source;
		source.cam_sr_no=serial;
		source.cameraMapped=true;
		img_src_id.push_back(source);

		
		//print info
		cout << "Camera " << i << ":" << endl;
		cout << "Vendor        : " << vendor << endl;
		cout << "  Model       : " << model << endl;
		cout << "  Serial      : " << source.cam_sr_no<< endl;
		cout << "-----------------------------" << endl;



		//opeaning camera
		GError* error =NULL;
		string camera_id = string(vendor)+"-"+ source.cam_sr_no;
		ArvCamera* cam=arv_camera_new(camera_id.c_str(),&error);
		CHECK_ERROR(error, "Failed to open camera")
		else{
		
			//setting camera to trigger mode
			arv_camera_set_string(cam, "TriggerMode", "On", &error);
			arv_camera_set_string(cam, "TriggerSource", "Software", &error);
			CHECK_ERROR(error, "Error in setting camera to Trigger Mode")
			
			
			//pushing camera 
			cameras.push_back(cam);
			cout <<"camera opned " << model << endl;
			

			//pushing stream 
			ArvStream* stream = arv_camera_create_stream(cam,NULL,NULL,NULL,&error);
			CHECK_ERROR(error, "Error in creating stream")
			streams.push_back(stream);
			
			
			//get payload size
			guint payload = arv_camera_get_payload(cam, &error);
			CHECK_ERROR(error, "Error in getting payload size")
			
			
			//pushing buffer
			buffers.push_back(arv_buffer_new(payload,NULL));
			
			//setting camera to acuquisition mode
			arv_camera_start_acquisition(cam,&error);
			CHECK_ERROR(error, "Error in starting acquisition In prepare camera")
			
		
			

				
		}
	    }


}

void CameraHandler::StopCamera(){
	cout << "entered stopCamera()" <<endl;
	//release all resources
	GError* error=NULL;
	for(int i=0;i<NoOfCamera;i++){
				
		arv_camera_stop_acquisition(cameras[i],&error);
		g_object_unref(cameras[i]);
		g_object_unref(buffers[i]);
		g_object_unref(streams[i]);
		CHECK_ERROR(error, "Cant Free Resources")

	}
	cout << "stopped camera" << endl;


}



					
//trigger camera
void CameraHandler::SoftTrigger(int id){
	GError* error=NULL;
	for(int j=0;j<NoOfCamera;j++){

		try{
			if(cameras[j]){
					
				string sr_no=String(arv_camera_get_device_serial_number(cameras[j],&error));
				CHECK_ERROR(error, "Cant get Serial Number of camera")
					
				if(String(img_src_id[id].cam_sr_no)==sr_no){
						
					arv_camera_software_trigger(cameras[j], &error);
					CHECK_ERROR(error, "SoftTrigger Error")
				}
			}
		}
		catch (exception& e){
			cerr << "Exception caught :" << e.what() << endl;
		}
	}

}

//function to SetExposure also gamma
void CameraHandler::SetExposure(double exp,double gam, int gain, string id){
	//seting exposure
	//
	int j=0;
	GError* error=NULL;
	try{
		//getting camer index	
		for(j=0;j<NoOfCamera;j++){
			if(cameras[j]){
					
				string sr_no=String(arv_camera_get_device_serial_number(cameras[j],&error));
				CHECK_ERROR(error, "Error in set Exposure Function ")
				if(id==sr_no)break;
			}
		}
		
	
		if(j<NoOfCamera&&j>=0){
		    	if(exp>26){
		       	if(exp>100000)
		            		exp=100000;
				if (arv_camera_is_exposure_time_available (cameras[j],&error)){
					//cout << "Exposure set to " <<arv_camera_get_exposure_time (cameras[j],&error) << endl ;
					//CHECK_ERROR(error, "Error in Getting Exposure")
					//cout << "eposure mode " << arv_exposure_mode_from_string ("Timed")<< endl;
				
					//memory write erro in the commented code below
					//arv_camera_set_exposure_mode (cameras[j],arv_exposure_mode_from_string ("Timed"), &error);	
					//CHECK_ERROR(error, "Error in setting exposure mode")	
				
					arv_camera_set_exposure_time(cameras[j], exp, &error);
					CHECK_ERROR(error, "Error in seting exposure time")
				
				
				
				}
				else cout << "Cant set Exposure" <<endl;
			}
			if(gam>0.0){
           
               	 	if(gam>3.99)
              		      		gam=3.99;
              		      		//TODO Set Gamma
              		}
						
		}	

	}
	catch (exception& e){
		cerr << "Exception caught :" << e.what() << endl;
	}
	

}




//functio to get exposure
int CameraHandler::GetExposure(string id){
	//getting exposure
	//
	int j=0;
	GError* error=NULL;
	try{
		//getting camer index	
		for(j=0;j<NoOfCamera;j++){
			if(cameras[j]){
					
				string sr_no=String(arv_camera_get_device_serial_number(cameras[j],&error));
				CHECK_ERROR(error, "Error in get Exposure Function ")
				if(id==sr_no)break;
			}
		}
		
		int exp=arv_camera_get_exposure_time (cameras[j],&error);
		CHECK_ERROR(error, "Error in Getting Exposure Value")
		return exp;
			
		
	}
	catch (exception& e){
		cerr << "Exception caught :" << e.what() << endl;
	}
		

}




//TODO: Get Gamma














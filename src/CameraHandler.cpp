#include "CameraHandler.h"
#include <iostream>



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



		string camera_id = string(vendor)+"-"+ source.cam_sr_no;
		ArvCamera* cam=arv_camera_new(camera_id.c_str(),&error);
		CHECK_ERROR(error, "Failed to open camera")
		else{
		
			//setting camera to trigger mode
			arv_camera_set_string(cam, "TriggerMode", "On", &error);
			arv_camera_set_string(cam, "TriggerSource", "Software", &error);
			CHECK_ERROR(error, "Error in setting camera to Trigger Mode");
			
			
			//pushing camera 
			cameras.push_back(cam);
			cout <<"camera opned " << model << endl;
			

			//pushing stream 
			ArvStream* stream = arv_camera_create_stream(cam,NULL,NULL,NULL,&error);
			CHECK_ERROR(error, "Error in creating stream");
			streams.push_back(stream);
			
			
			//get payload size
			guint payload = arv_camera_get_payload(cam, &error);
			CHECK_ERROR(error, "Error in getting payload size");
			
			
			//pushing buffer
			buffers.push_back(arv_buffer_new(payload,NULL));
			
					

				
		}
	    }


}

void CameraHandler::StopCamera(){
	//release all resources
	GError* error=NULL;
	for(int i=0;i<NoOfCamera;i++){
				
		arv_camera_stop_acquisition(cameras[i],&error);
		g_object_unref(cameras[i]);
		g_object_unref(buffers[i]);
		g_object_unref(streams[i]);
		CHECK_ERROR(error, "Cant Free Resources");

	}

}



					
//trigger camera
void CameraHandler::SoftTrigger(int id){
	GError* error=NULL;
	for(int j=0;j<NoOfCamera;j++){

		try{
			if(cameras[j]){
					
				string sr_no=String(arv_camera_get_device_serial_number(cameras[j],&error));
				CHECK_ERROR(error, "Cant get Serial Number of camera");
					
				if(String(img_src_id[id].cam_sr_no)==sr_no){
						
					arv_camera_software_trigger(cameras[j], &error);
					CHECK_ERROR(error, "SoftTrigger Error");
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
		//getting camera index	
		for(j=0;j<NoOfCamera;j++){
			if(cameras[j]){
					
				string sr_no=String(arv_camera_get_device_serial_number(cameras[j],&error));
				CHECK_ERROR(error, "Error in set Exposure Function ");
				if(id==sr_no)break;
			}
		}
		
	
		if(j<NoOfCamera&&j>=0){
			//setting exposure
		    	if(exp>26){
		       	if(exp>100000)
		            		exp=100000;
				if (arv_camera_is_exposure_time_available (cameras[j],&error)){
					
			
					//memory write error in the commented code below
					// ?should be in prepare camera??
					arv_camera_set_exposure_mode (cameras[j], arv_exposure_mode_from_string("Timed"), &error);
					CHECK_ERROR(error, "Error in setting exposure mode");
					arv_camera_set_exposure_time_auto(cameras[j], arv_auto_from_string("Off"), &error);
					CHECK_ERROR(error, "Error in setting auto exposure");
				
					arv_camera_set_exposure_time(cameras[j], exp, &error);
					CHECK_ERROR(error, "Error in seting exposure time");
					
			
				
				
				
				}
				else cout << "Cant set Exposure" <<endl;
				CHECK_ERROR(error, "Error Exposure Time Not Available");
				
			}
			//setting gamma
			if(gam>0.0){
           
               	 	if(gam>3.99)
              		      		gam=3.99;
              		      	
              		      	//? call on prepare camera??
			    	if (arv_camera_is_feature_available(cameras[j], "GammaEnable", &error)) {
					arv_camera_set_boolean(cameras[j], "GammaEnable", TRUE, &error);
					CHECK_ERROR(error, "Failed to Enable Gamma");
			    	}	    	
			    	
				// Set the gamma value
				arv_camera_set_float(cameras[j], "Gamma", gam, &error);
				CHECK_ERROR(error, "Failed to set gamma");
		    	
			
              		}
              		
              		//setting gain
              		if(gain<0||gain>100)return;
              		if(gain>0){
              			double gmin;
              			double gmax;
              			arv_camera_get_gain_bounds (cameras[j], &gmin, &gmax,&error);
              			CHECK_ERROR(error, "Failed to get max gain and min gain");
              			double gainRaw=double(gmin+(gmax-gmin)*gain/100);
              			cout << "gain value: "<<gain <<" Rawgain: "<<gainRaw<<endl;
              			
              			
              			//setting gain to manual mode :? should be in prepare camera function
              			arv_camera_set_gain_auto(cameras[j], arv_auto_from_string("Off"), &error);
              			CHECK_ERROR(error, "Failed to set Gain Manual Mode"); 
              			
              			
              			//setting gain
              			arv_camera_set_gain ( cameras[j], gainRaw, &error);
              			CHECK_ERROR(error, "Failed to set Gain"); 
              			
              		}
              		
								
		}	

	}
	catch (exception& e){
		cerr << "Exception caught :" << e.what() << endl;
	}
	

}




//functio to get exposure
double CameraHandler::GetExposure(string id){
	//getting exposure
	int j=0;
	GError* error=NULL;
	try{
		//getting camera index	
		for(j=0;j<NoOfCamera;j++){
			if(cameras[j]){
					
				string sr_no=String(arv_camera_get_device_serial_number(cameras[j],&error));
				CHECK_ERROR(error, "Error in get Exposure Function ");
				if(id==sr_no)break;
			}
		}
				
		//getting exposure
		if (arv_camera_is_exposure_time_available (cameras[j],&error)){
		
			double exp=arv_camera_get_exposure_time (cameras[j],&error);
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





//functio to get gamma 
double CameraHandler::GetGamma(string id){
	//getting gamma 
	//
	int j=0;
	GError* error=NULL;
	try{
		//getting camera index	
		for(j=0;j<NoOfCamera;j++){
			if(cameras[j]){
					
				string sr_no=String(arv_camera_get_device_serial_number(cameras[j],&error));
				CHECK_ERROR(error, "Error in get Exposure Function ");
				if(id==sr_no)break;
			}
		}

	    	
	        // Get the gamma value
	   	double gamma = arv_camera_get_float(cameras[j], "Gamma", &error);
		CHECK_ERROR(error, "Failed to get gamma");
		return gamma;
			
		
	}
	catch (exception& e){
		cerr << "Exception caught :" << e.what() << endl;
	}
	return 0;
		

}















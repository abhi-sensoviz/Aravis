#include<stdlib.h>
#include<ExtTrig.h>


int main(){
	int id=0;
	GError* error=NULL;
	ExtTrig* cam=new ExtTrig();	
	cam->InitiallizeBuffers();
	cam->PrepareCamera();
	if(cam->NoOfCamera==0)return 1;
	
	//cam->SetExposure(2000,2.5,50,cam->img_src_id[id].cam_sr_no);
	cout << cam->GetExposure(cam->img_src[id].cam_sr_no)<< endl;
	cout << cam->GetGamma(cam->img_src[id].cam_sr_no)<<endl;
	
	//printing gain
	cout << arv_camera_get_gain(cam->camera[id], &error)<< endl;
	CHECK_ERROR(error, "Error in getting camera gain");
	




	// arv_stream_push_buffer(cam->streams[id], cam->buffers[id]); 
	// g_signal_connect(stream[], "new-buffer", G_CALLBACK(new_buffer_callback), NULL);


	cam->SoftTrigger(id);  //triggring camera here
	cout << "Triggered camera"<< endl;

	
	cout << String(arv_camera_get_device_serial_number(cam->camera[id],&error))<<endl;
	
	


	//cout << cam->GetCameraTickCount(cam->img_src[id].cam_sr_no) <<endl;
	if(!cam->CheckCamConnection(id)){
		cam->StopCamera();
	}
	
	
	
	return 0;


}

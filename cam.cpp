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
	




	arv_stream_push_buffer(cam->streams[id], cam->buffers[id]); //pushing buffer to stream 1
	CHECK_ERROR(error, "Error starting acquisiton mode");


	cam->SoftTrigger(id);  //triggring camera here
	cout << "Triggered camera"<< endl;

		
		
	cam->buffers[id]=arv_stream_pop_buffer(cam->streams[id]); //poping buffer from stream
	
	
	if (!cam->buffers[id] || arv_buffer_get_status(cam->buffers[id]) != ARV_BUFFER_STATUS_SUCCESS) {
    		cerr << "Failed to get image from camera" << endl;
    		return 1;
	}
	
	
	//converting to cv::MAT
	
	cam->img_src[id].Width= arv_buffer_get_image_width(cam->buffers[id]);
	cam->img_src[id].Height= arv_buffer_get_image_height(cam->buffers[id]);
	
	//getting data from buffer
	const void* data = arv_buffer_get_data(cam->buffers[id],NULL); 


	Mat image(cam->img_src[id].Height,cam->img_src[id].Width, CV_8UC1, (void*) data);
	imwrite("sample.png", image);
	cout << "saved Image" << endl;
	
	
	cam->StopCamera();
	
	
	
	return 0;


}

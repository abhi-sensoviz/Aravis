#include<stdlib.h>
#include<CameraHandler.h>

int main(){
	int id=0;
	CameraHandler* cam=new CameraHandler();	
	cam->PrepareCamera();

	cam->SetExposure(1500,0.5,4,"DA7078263");
	cout << cam->GetExposure("DA7078263")<< endl;
	
	

	arv_stream_push_buffer(cam->streams[id], cam->buffers[id]); //pushing buffer to stream
	

	cam->SoftTrigger(id);  //triggring camera here
	cout << "Triggered camera"<< endl;

	cam->buffers[id]=arv_stream_pop_buffer(cam->streams[id]); //poping buffer from stream
	
	if (!cam->buffers[id] || arv_buffer_get_status(cam->buffers[id]) != ARV_BUFFER_STATUS_SUCCESS) {
    		cerr << "Failed to get image from camera" << endl;
    		return 1;
	}
	
	
	
	//converting to cv::MAT
	
	cam->img_src_id[id].Width= arv_buffer_get_image_width(cam->buffers[id]);
	cam->img_src_id[id].Height= arv_buffer_get_image_height(cam->buffers[id]);
	
	const void* data = arv_buffer_get_data(cam->buffers[id],NULL); 
	
	Mat image(cam->img_src_id[id].Height,cam->img_src_id[id].Width, CV_8UC1, (void*) data);
	imwrite("sample.png", image);
	cout << "saved Image" << endl;
	
	
	cam->StopCamera();
	
	
	
	return 0;


}

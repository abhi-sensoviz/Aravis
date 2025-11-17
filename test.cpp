#include <aravis-0.8/arv.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

using namespace std;
int main () {

    GError *error = NULL;

    // /* 1. Find a camera */
    // const char *camera_id = arv_get_device_id(0);
    // if (!camera_id) {
    //     printf("No camera found\n");
    //     return -1;
    // }
    const char* cam_sr_no = arv_get_device_serial_nbr(0);
	const char* vendor = arv_get_device_vendor(0);

    string camera_id = string(vendor)+"-"+ cam_sr_no;
    ArvCamera* cam=arv_camera_new(camera_id.c_str(),&error);

    /* 2. Open camera */
    // ArvCamera *cam = arv_camera_new(camera_id, &error);
    if (!cam) {
        printf("Cannot open camera\n");
        return -1;
    }

    /* 3. Set pixel format (use Mono8 for simplicity) */
    arv_camera_set_pixel_format(cam, ARV_PIXEL_FORMAT_MONO_8, &error);

    /* 4. Configure trigger */
    arv_camera_set_string(cam, "TriggerMode", "On", &error);
    arv_camera_set_string(cam, "TriggerSource", "Software", &error);
    arv_camera_set_string(cam, "TriggerSelector", "ExposureStart", &error);

    /* 5. Create stream */
    ArvStream *stream = arv_camera_create_stream(cam, NULL, NULL, &error);

    /* 6. Insert required buffers */
    guint payload = arv_camera_get_payload(cam, &error);
    for (int i = 0; i < 4; i++)
        arv_stream_push_buffer(stream, arv_buffer_new(payload, NULL));

    /* 7. Start acquisition */
    arv_camera_start_acquisition(cam, &error);

    /* 8. Fire software trigger */
    arv_camera_software_trigger(cam, &error);

    /* 9. Wait for buffer */
    ArvBuffer *buffer = arv_stream_timeout_pop_buffer(stream, 2000); // 2 sec timeout

    if (buffer && arv_buffer_get_status(buffer) == ARV_BUFFER_STATUS_SUCCESS) {

        /* 10. Get image data */
        size_t size;
        const void *data = arv_buffer_get_data(buffer, &size);

        /* 11. Save PGM image (Mono8) */
        FILE *f = fopen("image.pgm", "wb");
        // fprintf(f, "P5\n%u %u\n255\n",
        //         arv_camera_get_width(cam, NULL),
        //         arv_camera_get_height(cam, NULL));
        fwrite(data, 1, size, f);
        fclose(f);

        printf("Image saved as image.pgm (%zu bytes)\n", size);
    } else {
        printf("Failed to receive buffer\n");
    }

    /* 12. Requeue buffer */
    if (buffer)
        arv_stream_push_buffer(stream, buffer);

    /* 13. Cleanup */
    arv_camera_stop_acquisition(cam, NULL);
    g_object_unref(stream);
    g_object_unref(cam);

    return 0;
}

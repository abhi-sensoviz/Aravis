#include <arv.h>
#include <stdlib.h>
#include <signal.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <sys/types.h>

typedef struct {
    GMainLoop *main_loop;
    int buffer_count;
    int frame_index;
} ApplicationData;

static gboolean cancel = FALSE;

static void set_cancel(int signal) {
    cancel = TRUE;
}

static void save_buffer_to_file(ArvBuffer *buffer, int frame_index) {
    gint width = arv_buffer_get_image_width(buffer);
    gint height = arv_buffer_get_image_height(buffer);
    const guint8 *data = (const guint8*)arv_buffer_get_data(buffer, NULL);
    if (!data) return;

    // Ensure folder exists
    struct stat st = {0};
    if (stat("image", &st) == -1) {
        mkdir("image", 0755);
    }

    char filename[256];
    snprintf(filename, sizeof(filename), "image/frame_%04d.png", frame_index);

    cv::Mat img(height, width, CV_8UC1, (void*)data);
    cv::imwrite(filename, img.clone());

    printf("Saved %s\n", filename);
}

static void new_buffer_cb(ArvStream *stream, ApplicationData *data) {
    ArvBuffer *buffer = arv_stream_try_pop_buffer(stream);
    if (!buffer) return;

    if (arv_buffer_get_status(buffer) == ARV_BUFFER_STATUS_SUCCESS) {
        data->buffer_count++;
        save_buffer_to_file(buffer, data->frame_index++);
    }
    arv_stream_push_buffer(stream, buffer);
}

static gboolean periodic_task_cb(void *abstract_data) {
    ApplicationData *data = (ApplicationData*)abstract_data;
    printf("Frame rate = %d Hz\n", data->buffer_count);
    data->buffer_count = 0;

    if (cancel) {
        g_main_loop_quit(data->main_loop);
        return FALSE;
    }
    return TRUE;
}

static void control_lost_cb(ArvGvDevice *gv_device) {
    printf("Control lost\n");
    cancel = TRUE;
}

int main(int argc, char **argv) {
    ApplicationData data = {};
    GError *error = NULL;

    arv_update_device_list();
    int n = arv_get_n_devices();
    if (n == 0) {
        printf("No camera found\n");
        return 1;
    }

    const char *camera_id = arv_get_device_id(0);
    printf("Using camera: %s\n", camera_id);

    ArvCamera *camera = arv_camera_new(camera_id, &error);
    g_free((gpointer)camera_id);

    if (!ARV_IS_CAMERA(camera)) {
        printf("Failed to create camera: %s\n", error ? error->message : "Unknown error");
        g_clear_error(&error);
        return 1;
    }

    // Set maximum packet size for GigE cameras
    if (ARV_IS_GV_DEVICE(arv_camera_get_device(camera))) {
        ArvGvDevice *gv_device = ARV_GV_DEVICE(arv_camera_get_device(camera));
        guint max_packet_size = arv_gv_device_get_packet_size(gv_device, NULL);
        if (max_packet_size == 0) {
    		// Safe fallback for most GigE cameras
    		max_packet_size = 1400;
	}
        arv_gv_device_set_packet_size(gv_device, max_packet_size,NULL);
        g_signal_connect(gv_device, "control-lost", G_CALLBACK(control_lost_cb), NULL);
    }

    gint payload = arv_camera_get_payload(camera, &error);
    if (payload <= 0) {
        printf("Invalid payload: %d\n", payload);
        g_object_unref(camera);
        return 1;
    }

    // Create stream
    ArvStream *stream = arv_camera_create_stream(camera, NULL, NULL, NULL, &error);
    if (!ARV_IS_STREAM(stream)) {
        printf("Can't create stream: %s\n", error ? error->message : "Unknown error");
        g_clear_error(&error);
        g_object_unref(camera);
        return 1;
    }

    // Push buffers
    for (int i = 0; i < 50; i++)
        arv_stream_push_buffer(stream, arv_buffer_new(payload, NULL));

    // Start acquisition
    arv_camera_start_acquisition(camera, &error);

    g_signal_connect(stream, "new-buffer", G_CALLBACK(new_buffer_cb), &data);
    arv_stream_set_emit_signals(stream, TRUE);

    g_timeout_add_seconds(1, periodic_task_cb, &data);
    data.main_loop = g_main_loop_new(NULL, FALSE);
    signal(SIGINT, set_cancel);

    g_main_loop_run(data.main_loop);

    // Clean up
    arv_camera_stop_acquisition(camera, NULL);
    arv_stream_set_emit_signals(stream, FALSE);
    g_object_unref(stream);
    g_object_unref(camera);
    g_main_loop_unref(data.main_loop);

    return 0;
}

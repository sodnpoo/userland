/*
 * File:   opencv_modect.c
 * Author: lee@sodnpoo.com
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
//#include <opencv2/video/video.hpp>

#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"

#include "RaspiCamControl.h"
#include "RaspiPreview.h"
#include "RaspiCLI.h"

//#include "vgfont.h"

#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

#define CALC_FPS 1


#define DEFAULT_VIDEO_FPS 30
#define DEFAULT_VIDEO_WIDTH 1280
#define DEFAULT_VIDEO_HEIGHT 720


//FPS: OpenCV = 15.05, Video = 30.51, ~60% CPU
/*
#define VIDEO_FPS 30
#define VIDEO_WIDTH 1280
#define VIDEO_HEIGHT 720
*/
int still_interval = 60;
const char* STILL_TMPFN = "/tmp/opencv_modect.jpg";

/*
//FPS: OpenCV = 14.90, Video = 30.02, ~75% CPU
#define VIDEO_FPS 30
#define VIDEO_WIDTH 1920
#define VIDEO_HEIGHT 1080
*/
/*
//FPS: OpenCV = 21.57, Video = 91.12, CPU ~90%
#define VIDEO_FPS 90
#define VIDEO_WIDTH 640
#define VIDEO_HEIGHT 480
*/

typedef struct {
    int width;
    int height;
    int fps;

    MMAL_COMPONENT_T *camera;
    MMAL_COMPONENT_T *encoder;
    MMAL_COMPONENT_T *preview;

    MMAL_PORT_T *camera_video_port;
    MMAL_POOL_T *camera_video_port_pool;
    MMAL_PORT_T *encoder_input_port;
    MMAL_POOL_T *encoder_input_pool;
    MMAL_PORT_T *encoder_output_port;
    MMAL_POOL_T *encoder_output_pool;

    int opencv_width;
    int opencv_height;
    VCOS_SEMAPHORE_T complete_semaphore;

    signed int motion;
    int grabframe;

    float video_fps;
    float opencv_fps;

    IplImage* small_image; // resized image
    IplImage* stub; // stub

    char *stillfn; //place to write stills to
    int rotation;
} PORT_USERDATA;

int fill_port_buffer(MMAL_PORT_T *port, MMAL_POOL_T *pool) {
    int q;
    int num = mmal_queue_length(pool->queue);

    for (q = 0; q < num; q++) {
        MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(pool->queue);
        if (!buffer) {
            fprintf(stderr, "Unable to get a required buffer %d from pool queue\n", q);
        }

        if (mmal_port_send_buffer(port, buffer) != MMAL_SUCCESS) {
            fprintf(stderr, "Unable to send a buffer to port (%d)\n", q);
        }
    }
}

static void camera_video_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    //fprintf(stderr, "%s %d\n", __func__, buffer->length);
    PORT_USERDATA *userdata = (PORT_USERDATA *) port->userdata;

    static struct timespec t1;
    struct timespec t2;

    static int frame_count = 0;
    static int frame_post_count = 0;

    if (frame_count == 0) {
        clock_gettime(CLOCK_MONOTONIC, &t1);
    }
    frame_count++;

    //if(1){
    if( (CALC_FPS) && (frame_count % (userdata->fps*2) == 0) ){ //every 2 seconds
      // print framerate every n frame
      clock_gettime(CLOCK_MONOTONIC, &t2);
      float d = (t2.tv_sec + t2.tv_nsec / 1000000000.0) - (t1.tv_sec + t1.tv_nsec / 1000000000.0);
      float fps = 0.0;

      if (d > 0) {
          fps = frame_count / d;
      } else {
          fps = frame_count;
      }
      userdata->video_fps = fps;
      //fprintf(stderr, "  Frame = %d, Frame Post %d, Framerate = %.0f fps \n", frame_count, frame_post_count, fps);
    }

    //if(1){
    if(userdata->grabframe){
      mmal_buffer_header_mem_lock(buffer);

      //monkey with the imageData pointer, to avoid a memcpy
      char* oldImageData = userdata->stub->imageData;
      userdata->stub->imageData = buffer->data;
      cvResize(userdata->stub, userdata->small_image, CV_INTER_LINEAR);
      userdata->stub->imageData = oldImageData;

      mmal_buffer_header_mem_unlock(buffer);

      if (vcos_semaphore_trywait(&(userdata->complete_semaphore)) != VCOS_SUCCESS) {
        vcos_semaphore_post(&(userdata->complete_semaphore));
        frame_post_count++;
      }
    }

    if(0){
    //if( (userdata->stillfn) && (frame_count % (userdata->fps * still_interval) == 0) ){ //every 60 seconds
      mmal_buffer_header_mem_lock(buffer);

      fprintf(stderr, "WRITING STILL (%d)\n", frame_count);
/*
      //Just grab the Y and write it out ASAP
      //monkey with the imageData pointer, to avoid a memcpy
      char* oldImageData = userdata->stub->imageData;
      userdata->stub->imageData = buffer->data;

      //grab a still for export to www
      cvSaveImage("/home/pi/image.tmp.jpg", userdata->stub, 0);

      userdata->stub->imageData = oldImageData;
*/
/**/
      //TODO some of this can probably be collapsed down, but as we only do this once a minute I don't care so much....
      //so here we're going to attempt a new method to get full YUV
      unsigned char* pointer = (unsigned char *)(buffer -> data);
      //get Y U V as CvMat()s
      CvMat y = cvMat(userdata->height, userdata->width, CV_8UC1, pointer);
      pointer = pointer + (userdata->height*userdata->width);
      CvMat u = cvMat(userdata->height/2, userdata->width/2, CV_8UC1, pointer);
      pointer = pointer + (userdata->height*userdata->width/4);
      CvMat v = cvMat(userdata->height/2, userdata->width/2, CV_8UC1, pointer);
      //resize U and V and convert Y U and V into IplImages
      IplImage* uu = cvCreateImage(cvSize(userdata->width, userdata->height), IPL_DEPTH_8U, 1);
      cvResize(&u, uu, CV_INTER_LINEAR);
      IplImage* vv = cvCreateImage(cvSize(userdata->width, userdata->height), IPL_DEPTH_8U, 1);
      cvResize(&v, vv, CV_INTER_LINEAR);
      IplImage* yy = cvCreateImage(cvSize(userdata->width, userdata->height), IPL_DEPTH_8U, 1);
      cvResize(&y, yy, CV_INTER_LINEAR);
      //Create the final, 3 channel image
      IplImage* image = cvCreateImage(cvSize(userdata->width, userdata->height), IPL_DEPTH_8U, 3);
      CvArr * output[] = { image };
      //map Y to the 1st channel
      int from_to[] = {0, 0};
      const CvArr * inputy[] = { yy };
      cvMixChannels(inputy, 1, output, 1, from_to, 1);
      //map V to the 2nd channel
      from_to[1] = 1;
      const CvArr * inputv[] = { vv };
      cvMixChannels(inputv, 1, output, 1, from_to, 1);
      //map U to the 3rd channel
      from_to[1] = 2;
      const CvArr * inputu[] = { uu };
      cvMixChannels(inputu, 1, output, 1, from_to, 1);
      //convert the colour space
      cvCvtColor(image, image, CV_YCrCb2BGR);
      //save the image
      cvSaveImage(STILL_TMPFN, image, 0);
      //cleanup the images
      cvReleaseImage(&yy);
      cvReleaseImage(&vv);
      cvReleaseImage(&uu);
      cvReleaseImage(&image);
/**/

      mmal_buffer_header_mem_unlock(buffer);

      rename(STILL_TMPFN, userdata->stillfn);

      if (vcos_semaphore_trywait(&(userdata->complete_semaphore)) != VCOS_SUCCESS) {
        vcos_semaphore_post(&(userdata->complete_semaphore));
        frame_post_count++;
      }
    }

    if(1){
    //if(userdata->motion > 0){
      MMAL_BUFFER_HEADER_T *output_buffer = mmal_queue_get(userdata->encoder_input_pool->queue);
      if(output_buffer){
        mmal_buffer_header_mem_lock(buffer);
        memcpy(output_buffer->data, buffer->data, buffer->length);
        output_buffer->length = buffer->length;
        mmal_buffer_header_mem_unlock(buffer);

        if (mmal_port_send_buffer(userdata->encoder_input_port, output_buffer) != MMAL_SUCCESS) {
          fprintf(stderr, "ERROR: Unable to send buffer \n");
        }
      }
      userdata->motion--;
    }
    mmal_buffer_header_release(buffer);

    // and send one back to the port (if still open)
    if (port->is_enabled) {
        MMAL_STATUS_T status;

        MMAL_BUFFER_HEADER_T *new_buffer;
        MMAL_POOL_T *pool = userdata->camera_video_port_pool;
        new_buffer = mmal_queue_get(pool->queue);

        if (new_buffer) {
            status = mmal_port_send_buffer(port, new_buffer);
        }

        if (!new_buffer || status != MMAL_SUCCESS) {
            fprintf(stderr, "[%s]Unable to return a buffer to the video port\n", __func__);
        }
    }
}

static void encoder_input_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    //fprintf(stderr, "%s %d\n", __func__, buffer->length);
    mmal_buffer_header_release(buffer);
}

static void encoder_output_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    //fprintf(stderr, "%s %d\n", __func__, buffer->length);
    MMAL_BUFFER_HEADER_T *new_buffer;
    PORT_USERDATA *userdata = (PORT_USERDATA *) port->userdata;
    MMAL_POOL_T *pool = userdata->encoder_output_pool;

    if(1){//(userdata->motion){
      //write h264 stream to stdout
      mmal_buffer_header_mem_lock(buffer);
      fwrite(buffer->data, 1, buffer->length, stdout);
      mmal_buffer_header_mem_unlock(buffer);
      /*
      //write out epoch:framenum
      static int frame_count = 0;
      if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_KEYFRAME)
      {
        time_t now = time(NULL);
        fprintf(stderr, "KEYFRAME (%d:%d)\n", now, frame_count);
      }

      if(buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END)
      {
        frame_count++;
      }
      */
    }

    mmal_buffer_header_release(buffer);

    if (port->is_enabled) {
        MMAL_STATUS_T status;

        new_buffer = mmal_queue_get(pool->queue);

        if (new_buffer) {
            status = mmal_port_send_buffer(port, new_buffer);
        }

        if (!new_buffer || status != MMAL_SUCCESS) {
            fprintf(stderr, "[%s]Unable to return a buffer to the video port\n", __func__);
        }
    }
}

/**
 * Set the rotation of the image
 * @param camera Pointer to camera component
 * @param rotation Degree of rotation (any number, but will be converted to 0,90,180 or 270 only)
 * @return 0 if successful, non-zero if any parameters out of range
 */
/*
int raspicamcontrol_set_rotation(MMAL_COMPONENT_T *camera, int rotation)
{
   int ret;
   int my_rotation = ((rotation % 360 ) / 90) * 90;

   ret = mmal_port_parameter_set_int32(camera->output[0], MMAL_PARAMETER_ROTATION, my_rotation);
   mmal_port_parameter_set_int32(camera->output[1], MMAL_PARAMETER_ROTATION, my_rotation);
   mmal_port_parameter_set_int32(camera->output[2], MMAL_PARAMETER_ROTATION, my_rotation);

   return ret;
}
*/
//TODO remove the preview port
int setup_camera(PORT_USERDATA *userdata) {
    MMAL_STATUS_T status;
    MMAL_COMPONENT_T *camera = 0;
    MMAL_ES_FORMAT_T *format;
    MMAL_PORT_T * camera_preview_port;
    MMAL_PORT_T * camera_video_port;
    MMAL_PORT_T * camera_still_port;
    MMAL_POOL_T * camera_video_port_pool;

    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: create camera %x\n", status);
        return -1;
    }
    userdata->camera = camera;
    userdata->camera_video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];

    camera_preview_port = camera->output[MMAL_CAMERA_PREVIEW_PORT];
    camera_video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
    camera_still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

    {
        MMAL_PARAMETER_CAMERA_CONFIG_T cam_config = {
            { MMAL_PARAMETER_CAMERA_CONFIG, sizeof (cam_config)},
            .max_stills_w = userdata->width,
            .max_stills_h = userdata->height,
            .stills_yuv422 = 0,
            .one_shot_stills = 1,
            .max_preview_video_w = userdata->width,
            .max_preview_video_h = userdata->height,
            .num_preview_video_frames = 3,
            .stills_capture_circular_buffer_height = 0,
            .fast_preview_resume = 0,
            .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
        };
        mmal_port_parameter_set(camera->control, &cam_config.hdr);
    }

    // Setup camera preview port format
    format = camera_preview_port->format;
    //format->encoding = MMAL_ENCODING_I420;
    format->encoding = MMAL_ENCODING_OPAQUE;
    format->encoding_variant = MMAL_ENCODING_I420;
    format->es->video.width = userdata->width;
    format->es->video.height = userdata->height;
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = userdata->width;
    format->es->video.crop.height = userdata->height;

    status = mmal_port_format_commit(camera_preview_port);

    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: camera viewfinder format couldn't be set\n");
        return -1;
    }

    // Setup camera video port format
    mmal_format_copy(camera_video_port->format, camera_preview_port->format);

    format = camera_video_port->format;
    format->encoding = MMAL_ENCODING_I420;
    format->encoding_variant = MMAL_ENCODING_I420;
    format->es->video.width = userdata->width;
    format->es->video.height = userdata->height;
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = userdata->width;
    format->es->video.crop.height = userdata->height;
    format->es->video.frame_rate.num = userdata->fps;
    format->es->video.frame_rate.den = 1;

    camera_video_port->buffer_num = 2;
    camera_video_port->buffer_size = (format->es->video.width * format->es->video.height * 12 / 8 ) * camera_video_port->buffer_num;

    fprintf(stderr, "camera video buffer_size = %d\n", camera_video_port->buffer_size);
    fprintf(stderr, "camera video buffer_num = %d\n", camera_video_port->buffer_num);

    status = mmal_port_format_commit(camera_video_port);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to commit camera video port format (%u)\n", status);
        return -1;
    }

    camera_video_port_pool = (MMAL_POOL_T *) mmal_port_pool_create(camera_video_port, camera_video_port->buffer_num, camera_video_port->buffer_size);
    userdata->camera_video_port_pool = camera_video_port_pool;
    camera_video_port->userdata = (struct MMAL_PORT_USERDATA_T *) userdata;


    status = mmal_port_enable(camera_video_port, camera_video_buffer_callback);

    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to enable camera video port (%u)\n", status);
        return -1;
    }

    status = mmal_component_enable(camera);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to enable camera (%u)\n", status);
        return -1;
    }

    fill_port_buffer(userdata->camera_video_port, userdata->camera_video_port_pool);

    if (mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS) {
        printf("%s: Failed to start capture\n", __func__);
    }

    raspicamcontrol_set_rotation(camera, userdata->rotation);

    fprintf(stderr, "camera created\n");
    return 0;
}

int setup_encoder(PORT_USERDATA *userdata) {
    MMAL_STATUS_T status;
    MMAL_COMPONENT_T *encoder = 0;
    MMAL_PORT_T *preview_input_port = NULL;

    MMAL_PORT_T *encoder_input_port = NULL, *encoder_output_port = NULL;
    MMAL_POOL_T *encoder_input_port_pool;
    MMAL_POOL_T *encoder_output_port_pool;

    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_ENCODER, &encoder);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to create preview (%u)\n", status);
        return -1;
    }

    encoder_input_port = encoder->input[0];
    encoder_output_port = encoder->output[0];
    userdata->encoder_input_port = encoder_input_port;
    userdata->encoder_output_port = encoder_input_port;

    mmal_format_copy(encoder_input_port->format, userdata->camera_video_port->format);
    encoder_input_port->buffer_size = encoder_input_port->buffer_size_recommended;
    encoder_input_port->buffer_num = 2;

    mmal_format_copy(encoder_output_port->format, encoder_input_port->format);

    encoder_output_port->buffer_size = encoder_output_port->buffer_size_recommended;
    encoder_output_port->buffer_num = 2;
    // Commit the port changes to the input port
    status = mmal_port_format_commit(encoder_input_port);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to commit encoder input port format (%u)\n", status);
        return -1;
    }

    // Only supporting H264 at the moment
    encoder_output_port->format->encoding = MMAL_ENCODING_H264;
    encoder_output_port->format->bitrate = 2000000;

    encoder_output_port->buffer_size = encoder_output_port->buffer_size_recommended;

    if (encoder_output_port->buffer_size < encoder_output_port->buffer_size_min) {
        encoder_output_port->buffer_size = encoder_output_port->buffer_size_min;
    }

    encoder_output_port->buffer_num = encoder_output_port->buffer_num_recommended;

    if (encoder_output_port->buffer_num < encoder_output_port->buffer_num_min) {
        encoder_output_port->buffer_num = encoder_output_port->buffer_num_min;
    }

    // Commit the port changes to the output port
    status = mmal_port_format_commit(encoder_output_port);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to commit encoder output port format (%u)\n", status);
        return -1;
    }

    fprintf(stderr, "encoder input buffer_size = %d\n", encoder_input_port->buffer_size);
    fprintf(stderr, "encoder input buffer_num = %d\n", encoder_input_port->buffer_num);

    fprintf(stderr, "encoder output buffer_size = %d\n", encoder_output_port->buffer_size);
    fprintf(stderr, "encoder output buffer_num = %d\n", encoder_output_port->buffer_num);

    encoder_input_port_pool = (MMAL_POOL_T *) mmal_port_pool_create(encoder_input_port, encoder_input_port->buffer_num, encoder_input_port->buffer_size);
    userdata->encoder_input_pool = encoder_input_port_pool;
    encoder_input_port->userdata = (struct MMAL_PORT_USERDATA_T *) userdata;
    status = mmal_port_enable(encoder_input_port, encoder_input_buffer_callback);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to enable encoder input port (%u)\n", status);
        return -1;
    }
    fprintf(stderr, "encoder input pool has been created\n");

    encoder_output_port_pool = (MMAL_POOL_T *) mmal_port_pool_create(encoder_output_port, encoder_output_port->buffer_num, encoder_output_port->buffer_size);
    userdata->encoder_output_pool = encoder_output_port_pool;
    encoder_output_port->userdata = (struct MMAL_PORT_USERDATA_T *) userdata;

    status = mmal_port_enable(encoder_output_port, encoder_output_buffer_callback);
    if (status != MMAL_SUCCESS) {
        fprintf(stderr, "Error: unable to enable encoder output port (%u)\n", status);
        return -1;
    }
    fprintf(stderr, "encoder output pool has been created\n");

    fill_port_buffer(encoder_output_port, encoder_output_port_pool);

    fprintf(stderr, "encoder has been created\n");
    return 0;
}

void signal_start(){
  char *cmd = "/usr/bin/pkill -USR1 -f snapcat.py";
  system(cmd);
}

void signal_stop(){
  char *cmd = "/usr/bin/pkill -USR2 -f snapcat.py";
  system(cmd);
}

static void update_annotation_data(MMAL_COMPONENT_T *camera_component, char *hostname)
{
  time_t t = time(NULL);
  struct tm tm = *localtime(&t);
  char tmp[MMAL_CAMERA_ANNOTATE_MAX_TEXT_LEN_V3];
  strftime(tmp, MMAL_CAMERA_ANNOTATE_MAX_TEXT_LEN_V3, "%F %T", &tm );

  char *text;
  asprintf(&text, " %s@ %s ", hostname, tmp);

  int settings = 0x0;
  settings |= ANNOTATE_USER_TEXT;
  settings |= ANNOTATE_BLACK_BACKGROUND;

  int bg_colour = -1;
  int fg_colour = -1;

  raspicamcontrol_set_annotate(camera_component, settings, text, 32, fg_colour, bg_colour);

  free(text);
}

int main(int argc, char** argv) {
    PORT_USERDATA userdata;
    MMAL_STATUS_T status;
    memset(&userdata, 0, sizeof (PORT_USERDATA));

    userdata.width = DEFAULT_VIDEO_WIDTH;
    userdata.height = DEFAULT_VIDEO_HEIGHT;
    userdata.fps = DEFAULT_VIDEO_FPS;

    int c;
    opterr = 0;
    while ((c = getopt (argc, argv, "r:w:h:f:s:")) != -1){
      switch (c) {
        case 'r': //rotation
          userdata.rotation = atoi(optarg);
          break;
        case 'w':
          userdata.width = atoi(optarg);
          break;
        case 'h':
          userdata.height = atoi(optarg);
          break;
        case 'f':
          userdata.fps = atoi(optarg);
          break;
        case 's':
          userdata.stillfn = optarg;
          break;
        case '?':
          if ((optopt == 's') || (optopt == 'r'))
          //if (optopt == 's')
            fprintf (stderr, "Option -%c requires an argument.\n", optopt);
          else if (isprint (optopt))
            fprintf (stderr, "Unknown option `-%c'.\n", optopt);
          else
            fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
          return 1;
        default:
          return 1;
      }
    }

    if(userdata.stillfn){
      fprintf(stderr, "Writing still images to %s\n", userdata.stillfn);
    }

    userdata.opencv_width = 160;//userdata.width/4;
    userdata.opencv_height = 120;//userdata.height/4;
    userdata.motion = 0;
    userdata.grabframe = 1;

    fprintf(stderr, "VIDEO_WIDTH : %i\n", userdata.width );
    fprintf(stderr, "VIDEO_HEIGHT: %i\n", userdata.height );
    fprintf(stderr, "VIDEO_FPS   : %i\n", userdata.fps);

    bcm_host_init();

    if (1 && setup_camera(&userdata) != 0) {
        fprintf(stderr, "Error: setup camera %x\n", status);
        return -1;
    }

    if (1 && setup_encoder(&userdata) != 0) {
        fprintf(stderr, "Error: setup encoder %x\n", status);
        return -1;
    }

    vcos_semaphore_create(&userdata.complete_semaphore, "mmal_opencv_video", 0);

    IplImage* back = NULL;
    IplImage* fore = NULL;
    IplImage* sub = NULL;
    IplImage* gray = NULL;
    IplImage* mask = NULL;

    IplImage* tmp = NULL;

    sub = cvCreateImage(cvSize(userdata.opencv_width, userdata.opencv_height), IPL_DEPTH_8U, 1);
    back = cvCreateImage(cvSize(userdata.opencv_width, userdata.opencv_height), IPL_DEPTH_8U, 1);
    gray = cvCreateImage(cvSize(userdata.opencv_width, userdata.opencv_height), IPL_DEPTH_8U, 1);
    tmp = cvCreateImage(cvSize(userdata.opencv_width, userdata.opencv_height), IPL_DEPTH_8U, 1);

    char *maskfn = "/home/pi/mask.png";
    mask = cvLoadImage(maskfn, CV_LOAD_IMAGE_GRAYSCALE );
    if(!mask){
        fprintf(stderr, "couldn't load mask");
        return -1;
    }

    userdata.small_image = cvCreateImage(cvSize(userdata.opencv_width, userdata.opencv_height), IPL_DEPTH_8U, 1);
    userdata.stub = cvCreateImage(cvSize(userdata.width, userdata.height), IPL_DEPTH_8U, 1);

    int count = 0;

    int opencv_frames = 0;
    struct timespec t1;
    struct timespec t2;
    clock_gettime(CLOCK_MONOTONIC, &t1);

    struct timespec s;
    s.tv_sec = 0;
    s.tv_nsec = 30000000;

    char hostname[1024];
    gethostname(hostname, 1024);

    while (1) {

      //nanosleep(&s, NULL);

      if(1){
        if (vcos_semaphore_wait(&(userdata.complete_semaphore)) == VCOS_SUCCESS) {
          userdata.grabframe = 0;

          opencv_frames++;
          //if (1) {
          if( (CALC_FPS) && (opencv_frames % (userdata.fps*1) == 0) ){
            clock_gettime(CLOCK_MONOTONIC, &t2);
            float d = (t2.tv_sec + t2.tv_nsec / 1000000000.0) - (t1.tv_sec + t1.tv_nsec / 1000000000.0);
            if (d > 0) {
              userdata.opencv_fps = opencv_frames / d;
            } else {
              userdata.opencv_fps = opencv_frames;
            }

            fprintf(stderr, "FPS: OpenCV = %.2f, Video = %.2f\n", userdata.opencv_fps, userdata.video_fps);

            update_annotation_data(userdata.camera, hostname);

          }

          fore = userdata.small_image;

          if(!back){
            cvCopy(fore, back, NULL);
          }

          //cv2.blur(fore, tmp);

          cvSmooth(fore, fore, CV_GAUSSIAN, 3, 0, 0, 0);

          cvSub(back, fore, sub, NULL);
          cvCopy(fore, back, NULL);

          cvErode(sub, sub, NULL, 1);

          cvCanny(sub, sub, 20, 60, 3);

          cvCopy(sub, tmp, mask);
/*
          //DUMP steps to files
          char fn[256];
          sprintf(fn, "/home/pi/test_fore_%d.jpg", count);
          cvSaveImage(fn, fore, 0);
          sprintf(fn, "/home/pi/test_sub_%d.jpg", count);
          cvSaveImage(fn, sub, 0);
          sprintf(fn, "/home/pi/test_back_%d.jpg", count);
          cvSaveImage(fn, back, 0);
          count++;
*/

          int n = cvCountNonZero(tmp);
          //fprintf(stderr, "N (%d)\n", n);
          //if(n>0){
          int threshold = 10;


          if(n>threshold){

            if(userdata.motion < 0){
              fprintf(stderr, "START (%d)\n", n);
              signal_start();
            }

            userdata.motion = (userdata.fps * 15) + threshold; //number of seconds to capture after detection
            fprintf(stderr, "MOTION DETECTED (%d)\n", n);
          }else{
            //fprintf(stderr, "userdata.motion (%d)\n", userdata.motion);


            if(userdata.motion > 0 && userdata.motion < threshold){
              fprintf(stderr, "STOP (%d)\n", userdata.motion);
              userdata.motion = 0;
              signal_stop();
            }
            //userdata.motion = 0;
          }

          userdata.grabframe = 1;
        }
      }
    }

    return 0;
}

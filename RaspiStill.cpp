/*
 Copyright (c) 2013, Broadcom Europe Ltd
 Copyright (c) 2013, James Hughes
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file RaspiStill.c
 * Command line program to capture a still frame and encode it to file.
 * Also optionally display a preview/viewfinder of current camera input.
 *
 * \date 31 Jan 2013
 * \Author: James Hughes
 *
 * Description
 *
 * 3 components are created; camera, preview and JPG encoder.
 * Camera component has three ports, preview, video and stills.
 * This program connects preview and stills to the preview and jpg
 * encoder. Using mmal we don't need to worry about buffers between these
 * components, but we do need to handle buffers from the encoder, which
 * are simply written straight to the file in the requisite buffer callback.
 *
 * We use the RaspiCamControl code to handle the specific camera settings.
 */

extern "C" {
#include "types.h"
}

#include "FlashCam.h"
#include <string>

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <memory.h>
#include <unistd.h>
#include <errno.h>
#include <sysexits.h>

#define VERSION_STRING "v1.3.11"

#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/mmal_parameters_camera.h"

//#include "RaspiCamControl.h"

#include <semaphore.h>

// Standard port setting for the camera component
//#define MMAL_CAMERA_PREVIEW_PORT 0
//#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

// Stills format information
// 0 implies variable
#define STILLS_FRAME_RATE_NUM 0
#define STILLS_FRAME_RATE_DEN 1

/// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM 3

/// Frame advance method
#define FRAME_NEXT_SINGLE        0
#define FRAME_NEXT_TIMELAPSE     1
#define FRAME_NEXT_KEYPRESS      2
#define FRAME_NEXT_FOREVER       3
#define FRAME_NEXT_GPIO          4
#define FRAME_NEXT_SIGNAL        5
#define FRAME_NEXT_IMMEDIATELY   6


/** Structure containing all state information for the current run
 */
typedef struct
{
    int timeout;                        /// Time taken before frame is grabbed and app then shuts down. Units are milliseconds
    unsigned int width;                          /// Requested width of image
    unsigned int height;                         /// requested height of image
    char camera_name[MMAL_PARAMETER_CAMERA_INFO_MAX_STR_LEN]; // Name of the camera sensor
    int quality;                        /// JPEG quality setting (1-100)
    int wantRAW;                        /// Flag for whether the JPEG metadata also contains the RAW bayer image
    char *filename;                     /// filename of output file
    char *linkname;                     /// filename of output file
    int frameStart;                     /// First number of frame output counter
    int verbose;                        /// !0 if want detailed run information
    int demoMode;                       /// Run app in demo mode
    int demoInterval;                   /// Interval between camera settings changes
    MMAL_FOURCC_T encoding;             /// Encoding to use for the output file.
    int timelapse;                      /// Delay between each picture in timelapse mode. If 0, disable timelapse
    int frameNextMethod;                /// Which method to use to advance to next frame
    int glCapture;                      /// Save the GL frame-buffer instead of camera output
    int settings;                       /// Request settings from the camera
    int cameraNum;                      /// Camera number
    int burstCaptureMode;               /// Enable burst mode
    int sensor_mode;                     /// Sensor mode. 0=auto. Check docs/forum for modes selected by other values.
    int datetime;                       /// Use DateTime instead of frame#
    int timestamp;                      /// Use timestamp instead of frame#
    int restart_interval;               /// JPEG restart interval. 0 for none.
    
    FLASHCAM_PARAMS_T camera_parameters; /// Camera setup parameters
    
    MMAL_COMPONENT_T *camera_component;    /// Pointer to the camera component
    MMAL_COMPONENT_T *encoder_component;   /// Pointer to the encoder component
    MMAL_CONNECTION_T *encoder_connection; /// Pointer to the connection from camera to encoder
    
    MMAL_POOL_T *encoder_pool; /// Pointer to the pool of buffers used by encoder output port
    
} RASPISTILL_STATE;

/** Struct used to pass information in encoder port userdata to callback
 */
typedef struct
{
    FILE *file_handle;                   /// File handle to write buffer data to.
    VCOS_SEMAPHORE_T complete_semaphore; /// semaphore which is posted when we reach end of frame (indicates end of capture or fault)
    RASPISTILL_STATE *pstate;            /// pointer to our state in case required in callback
} PORT_USERDATA;


static void set_sensor_defaults(RASPISTILL_STATE *state)
{
    MMAL_COMPONENT_T *camera_info;
    MMAL_STATUS_T status;
    
    // Default to the OV5647 setup
    strncpy(state->camera_name, "OV5647", MMAL_PARAMETER_CAMERA_INFO_MAX_STR_LEN);
    
    // Try to get the camera name and maximum supported resolution
    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA_INFO, &camera_info);
    if (status == MMAL_SUCCESS)
    {
        MMAL_PARAMETER_CAMERA_INFO_T param;
        param.hdr.id = MMAL_PARAMETER_CAMERA_INFO;
        param.hdr.size = sizeof(param)-4;  // Deliberately undersize to check firmware veresion
        status = mmal_port_parameter_get(camera_info->control, &param.hdr);
        
        if (status != MMAL_SUCCESS)
        {
            // Running on newer firmware
            param.hdr.size = sizeof(param);
            status = mmal_port_parameter_get(camera_info->control, &param.hdr);
            if (status == MMAL_SUCCESS && param.num_cameras > state->cameraNum)
            {
                // Take the parameters from the first camera listed.
                if (state->width == 0)
                    state->width = param.cameras[state->cameraNum].max_width;
                if (state->height == 0)
                    state->height = param.cameras[state->cameraNum].max_height;
                strncpy(state->camera_name,  param.cameras[state->cameraNum].camera_name, MMAL_PARAMETER_CAMERA_INFO_MAX_STR_LEN);
                state->camera_name[MMAL_PARAMETER_CAMERA_INFO_MAX_STR_LEN-1] = 0;
            }
            else
                vcos_log_error("Cannot read camera info, keeping the defaults for OV5647");
        }
        else
        {
            // Older firmware
            // Nothing to do here, keep the defaults for OV5647
        }
        
        mmal_component_destroy(camera_info);
    }
    else
    {
        vcos_log_error("Failed to create camera_info component");
    }
    //Command line hasn't specified a resolution, and we failed to
    //get a default resolution from camera_info. Assume OV5647 full res
    if (state->width == 0)
        state->width = 2592;
    if (state->height == 0)
        state->height = 1944;
    
    
    state->width  = 720;
    state->height = 480;
    
}

/**
 * Assign a default set of parameters to the state passed in
 *
 * @param state Pointer to state structure to assign defaults to
 */
static void default_status(RASPISTILL_STATE *state)
{
    if (!state)
    {
        vcos_assert(0);
        return;
    }
    
    state->timeout = 1000; // 5s delay before take image
    state->quality = 85;
    state->wantRAW = 0;
    state->filename = NULL;
    state->linkname = NULL;
    state->frameStart = 0;
    state->verbose = 1;
    state->demoMode = 0;
    state->demoInterval = 250; // ms
    state->camera_component = NULL;
    state->encoder_component = NULL;
    state->encoder_connection = NULL;
    state->encoder_pool = NULL;
    state->encoding = MMAL_ENCODING_JPEG;
    state->timelapse = 0;
    state->frameNextMethod = FRAME_NEXT_SINGLE;
    state->glCapture = 0;
    state->settings = 0;
    state->cameraNum = 0;
    state->burstCaptureMode=0;
    state->sensor_mode = 0;
    state->datetime = 0;
    state->timestamp = 0;
    state->restart_interval = 0;
    
    // Set up the camera_parameters to default
    //raspicamcontrol_set_defaults(&state->camera_parameters);
}

/**
 * Dump image state parameters to stderr. Used for debugging
 *
 * @param state Pointer to state structure to assign defaults to
 */
static void dump_status(RASPISTILL_STATE *state)
{
    int i;
    
    if (!state)
    {
        vcos_assert(0);
        return;
    }
    
    fprintf(stderr, "Width %d, Height %d, quality %d, filename %s\n", state->width,
            state->height, state->quality, state->filename);
    fprintf(stderr, "Time delay %d, Raw %s\n", state->timeout,
            state->wantRAW ? "yes" : "no");
    fprintf(stderr, "Link to latest frame enabled ");
    if (state->linkname)
    {
        fprintf(stderr, " yes, -> %s\n", state->linkname);
    }
    else
    {
        fprintf(stderr, " no\n");
    }
    
    fprintf(stderr, "\n\n");
    
}


/**
 * Handler for sigint signals
 *
 * @param signal_number ID of incoming signal.
 *
 */
static void signal_handler(int signal_number)
{
    if (signal_number == SIGUSR1 || signal_number == SIGUSR2)
    {
        // Handle but ignore - prevents us dropping out if started in none-signal mode
        // and someone sends us the USR1 or USR2 signal anyway
    }
    else
    {
        // Going to abort on all other signals
        vcos_log_error("Aborting program\n");
        exit(130);
    }
}

/**
 * Asked GPU how much memory it has allocated
 *
 * @return amount of memory in MB
 */
static int get_mem_gpu(void)
{
    char response[80] = "";
    int gpu_mem = 0;
    if (vc_gencmd(response, sizeof response, "get_mem gpu") == 0)
        vc_gencmd_number_property(response, "gpu", &gpu_mem);
    return gpu_mem;
}

/**
 *  buffer header callback function for camera control
 *
 *  No actions taken in current version
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void camera_control_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
    if (buffer->cmd == MMAL_EVENT_PARAMETER_CHANGED)
    {
        MMAL_EVENT_PARAMETER_CHANGED_T *param = (MMAL_EVENT_PARAMETER_CHANGED_T *)buffer->data;
        switch (param->hdr.id) {
            case MMAL_PARAMETER_CAMERA_SETTINGS:
            {
                MMAL_PARAMETER_CAMERA_SETTINGS_T *settings = (MMAL_PARAMETER_CAMERA_SETTINGS_T*)param;
                vcos_log_error("Exposure now %u, analog gain %u/%u, digital gain %u/%u",
                               settings->exposure,
                               settings->analog_gain.num, settings->analog_gain.den,
                               settings->digital_gain.num, settings->digital_gain.den);
                vcos_log_error("AWB R=%u/%u, B=%u/%u",
                               settings->awb_red_gain.num, settings->awb_red_gain.den,
                               settings->awb_blue_gain.num, settings->awb_blue_gain.den
                               );
            }
                break;
        }
    }
    else if (buffer->cmd == MMAL_EVENT_ERROR)
    {
        vcos_log_error("No data received from sensor. Check all connections, including the Sunny one on the camera board");
    }
    else
        vcos_log_error("Received unexpected camera control callback event, 0x%08x", buffer->cmd);
    
    mmal_buffer_header_release(buffer);
}

/**
 *  buffer header callback function for encoder
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void encoder_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
    
    vcos_log_error("callback");
    
    int complete = 0;
    
    // We pass our file handle and other stuff in via the userdata field.
    
    PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;
    
    if (pData)
    {
        int bytes_written = buffer->length;
        
        if (buffer->length && pData->file_handle)
        {
            mmal_buffer_header_mem_lock(buffer);
            
            bytes_written = fwrite(buffer->data, 1, buffer->length, pData->file_handle);
            
            mmal_buffer_header_mem_unlock(buffer);
        }
        
        // We need to check we wrote what we wanted - it's possible we have run out of storage.
        if (bytes_written != buffer->length)
        {
            vcos_log_error("Unable to write buffer to file - aborting");
            complete = 1;
        }
        
        // Now flag if we have completed
        if (buffer->flags & (MMAL_BUFFER_HEADER_FLAG_FRAME_END | MMAL_BUFFER_HEADER_FLAG_TRANSMISSION_FAILED))
            complete = 1;
    }
    else
    {
        vcos_log_error("Received a encoder buffer callback with no state");
    }
    
    // release buffer back to the pool
    mmal_buffer_header_release(buffer);
    
    // and send one back to the port (if still open)
    if (port->is_enabled)
    {
        MMAL_STATUS_T status = MMAL_SUCCESS;
        MMAL_BUFFER_HEADER_T *new_buffer;
        
        new_buffer = mmal_queue_get(pData->pstate->encoder_pool->queue);
        
        if (new_buffer)
        {
            status = mmal_port_send_buffer(port, new_buffer);
        }
        if (!new_buffer || status != MMAL_SUCCESS)
            vcos_log_error("Unable to return a buffer to the encoder port");
    }
    
    if (complete)
        vcos_semaphore_post(&(pData->complete_semaphore));
}

/**
 * Create the camera component, set up its ports
 *
 * @param state Pointer to state control struct. camera_component member set to the created camera_component if successful.
 *
 * @return MMAL_SUCCESS if all OK, something else otherwise
 *
 */
static MMAL_STATUS_T create_camera_component(RASPISTILL_STATE *state)
{
    MMAL_COMPONENT_T *camera = 0;
    MMAL_ES_FORMAT_T *format;
    MMAL_PORT_T *still_port = NULL;
    MMAL_STATUS_T status;
    MMAL_PARAMETER_INT32_T camera_num;
    
    FlashCam flashcam = FlashCam();
    FLASHCAM_PARAMS_T camera_parameters_copy;
    
    /* Create the component */
    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);
    
    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("Failed to create camera component");
        goto error;
    }
    
    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("Could not set stereo mode : error %d", status);
        goto error;
    }
    
    camera_num = {{MMAL_PARAMETER_CAMERA_NUM, sizeof(camera_num)}, state->cameraNum};
    
    status = mmal_port_parameter_set(camera->control, &camera_num.hdr);
    
    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("Could not select camera : error %d", status);
        goto error;
    }
    
    if (!camera->output_num)
    {
        status = MMAL_ENOSYS;
        vcos_log_error("Camera doesn't have output ports");
        goto error;
    }
    
    status = mmal_port_parameter_set_uint32(camera->control, MMAL_PARAMETER_CAMERA_CUSTOM_SENSOR_CONFIG, state->sensor_mode);
    
    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("Could not set sensor mode : error %d", status);
        goto error;
    }
    
    still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];
    
    if (state->settings)
    {
        MMAL_PARAMETER_CHANGE_EVENT_REQUEST_T change_event_request =
        {{MMAL_PARAMETER_CHANGE_EVENT_REQUEST, sizeof(MMAL_PARAMETER_CHANGE_EVENT_REQUEST_T)},
            MMAL_PARAMETER_CAMERA_SETTINGS, 1};
        
        status = mmal_port_parameter_set(camera->control, &change_event_request.hdr);
        if ( status != MMAL_SUCCESS )
        {
            vcos_log_error("No camera settings events");
        }
    }
    
    // Enable the camera, and tell it its control callback function
    status = mmal_port_enable(camera->control, camera_control_callback);
    
    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("Unable to enable control port : error %d", status);
        goto error;
    }
    
    //  set up the camera configuration
    {
        MMAL_PARAMETER_CAMERA_CONFIG_T cam_config =
        {
            { MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) },
            .max_stills_w = state->width,
            .max_stills_h = state->height,
            .stills_yuv422 = 0,
            .one_shot_stills = 1,
            .max_preview_video_w = state->width,
            .max_preview_video_h = state->height,
            .num_preview_video_frames = 3,
            .stills_capture_circular_buffer_height = 0,
            .fast_preview_resume = 0,
            .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
        };
        
        mmal_port_parameter_set(camera->control, &cam_config.hdr);
    }
    
    
    //FLASHCAM START    
    
    flashcam.setCamera(camera);
    
    //get params
    
    if ( flashcam.getAllParams(&state->camera_parameters) != 0) {
        vcos_log_error("Error getting camera params");
    } else {
        flashcam.printParams(&state->camera_parameters);
    }
    
    
    /*
     state->camera_parameters.awb            = MMAL_PARAM_AWBMODE_AUTO;
     state->camera_parameters.exposure       = MMAL_PARAM_EXPOSUREMODE_AUTO;
     state->camera_parameters.brightness     = 50;
     state->camera_parameters.denoise        = 1;
     
     state->camera_parameters.flash          = MMAL_PARAM_FLASH_ON;    
     //state->camera_parameters.awb            = MMAL_PARAM_AWBMODE_FLASH;
     
     
     //set updated values
     if ( flashcam.setAllParams(&state->camera_parameters) != 0) {
     vcos_log_error("Error setting camera params");
     } else {
     flashcam.printParams(&state->camera_parameters);
     }
     //check vluae
     if ( flashcam.getAllParams(&camera_parameters_copy) != 0) {
     vcos_log_error("Error getting camera params");
     } else {
     flashcam.printParams(&camera_parameters_copy);
     }
     */
    
    // Now set up the port formats
    
    
    format = still_port->format;
    
    /*
     if(state->camera_parameters.speed > 6000000)
     {
     MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
     { 50, 1000 }, {166, 1000}};
     mmal_port_parameter_set(still_port, &fps_range.hdr);
     }
     else if(state->camera_parameters.speed > 1000000)
     {
     MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
     { 167, 1000 }, {999, 1000}};
     mmal_port_parameter_set(still_port, &fps_range.hdr);
     }
     */
    
    // Set our stills format on the stills (for encoder) port
    format->encoding = MMAL_ENCODING_OPAQUE;
    format->es->video.width = VCOS_ALIGN_UP(state->width, 32);
    format->es->video.height = VCOS_ALIGN_UP(state->height, 16);
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = state->width;
    format->es->video.crop.height = state->height;
    format->es->video.frame_rate.num = STILLS_FRAME_RATE_NUM;
    format->es->video.frame_rate.den = STILLS_FRAME_RATE_DEN;
    
    
    status = mmal_port_format_commit(still_port);
    
    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("camera still format couldn't be set");
        goto error;
    }
    
    /* Ensure there are enough buffers to avoid dropping frames */
    if (still_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
        still_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;
    
    /* Enable component */
    status = mmal_component_enable(camera);
    
    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("camera component couldn't be enabled");
        goto error;
    }
    
    state->camera_component = camera;
    
    if (state->verbose)
        fprintf(stderr, "Camera component done\n");
    
    return status;
    
error:
    
    if (camera)
        mmal_component_destroy(camera);
    
    return status;
}


/**
 * Destroy the camera component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_camera_component(RASPISTILL_STATE *state)
{
    if (state->camera_component)
    {
        mmal_component_destroy(state->camera_component);
        state->camera_component = NULL;
    }
}

/**
 * Create the encoder component, set up its ports
 *
 * @param state Pointer to state control struct. encoder_component member set to the created camera_component if successful.
 *
 * @return a MMAL_STATUS, MMAL_SUCCESS if all OK, something else otherwise
 */
static MMAL_STATUS_T create_encoder_component(RASPISTILL_STATE *state)
{
    MMAL_COMPONENT_T *encoder = 0;
    MMAL_PORT_T *encoder_input = NULL, *encoder_output = NULL;
    MMAL_STATUS_T status;
    MMAL_POOL_T *pool;
    
    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_IMAGE_ENCODER, &encoder);
    
    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("Unable to create JPEG encoder component");
        goto error;
    }
    
    if (!encoder->input_num || !encoder->output_num)
    {
        status = MMAL_ENOSYS;
        vcos_log_error("JPEG encoder doesn't have input/output ports");
        goto error;
    }
    
    encoder_input = encoder->input[0];
    encoder_output = encoder->output[0];
    
    // We want same format on input and output
    mmal_format_copy(encoder_output->format, encoder_input->format);
    
    // Specify out output format
    encoder_output->format->encoding = state->encoding;
    
    encoder_output->buffer_size = encoder_output->buffer_size_recommended;
    
    if (encoder_output->buffer_size < encoder_output->buffer_size_min)
        encoder_output->buffer_size = encoder_output->buffer_size_min;
    
    encoder_output->buffer_num = encoder_output->buffer_num_recommended;
    
    if (encoder_output->buffer_num < encoder_output->buffer_num_min)
        encoder_output->buffer_num = encoder_output->buffer_num_min;
    
    // Commit the port changes to the output port
    status = mmal_port_format_commit(encoder_output);
    
    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("Unable to set format on video encoder output port");
        goto error;
    }
    
    // Set the JPEG quality level
    status = mmal_port_parameter_set_uint32(encoder_output, MMAL_PARAMETER_JPEG_Q_FACTOR, state->quality);
    
    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("Unable to set JPEG quality");
        goto error;
    }
    
    // Set the JPEG restart interval
    status = mmal_port_parameter_set_uint32(encoder_output, MMAL_PARAMETER_JPEG_RESTART_INTERVAL, state->restart_interval);
    
    if (state->restart_interval && status != MMAL_SUCCESS)
    {
        vcos_log_error("Unable to set JPEG restart interval");
        goto error;
    }
    
    //  Enable component
    status = mmal_component_enable(encoder);
    
    if (status  != MMAL_SUCCESS)
    {
        vcos_log_error("Unable to enable video encoder component");
        goto error;
    }
    
    /* Create pool of buffer headers for the output port to consume */
    pool = mmal_port_pool_create(encoder_output, encoder_output->buffer_num, encoder_output->buffer_size);
    
    if (!pool)
    {
        vcos_log_error("Failed to create buffer header pool for encoder output port %s", encoder_output->name);
    }
    
    state->encoder_pool = pool;
    state->encoder_component = encoder;
    
    if (state->verbose)
        fprintf(stderr, "Encoder component done\n");
    
    return status;
    
error:
    
    if (encoder)
        mmal_component_destroy(encoder);
    
    return status;
}

/**
 * Destroy the encoder component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_encoder_component(RASPISTILL_STATE *state)
{
    // Get rid of any port buffers first
    if (state->encoder_pool)
    {
        mmal_port_pool_destroy(state->encoder_component->output[0], state->encoder_pool);
    }
    
    if (state->encoder_component)
    {
        mmal_component_destroy(state->encoder_component);
        state->encoder_component = NULL;
    }
}


/**
 * Connect two specific ports together
 *
 * @param output_port Pointer the output port
 * @param input_port Pointer the input port
 * @param Pointer to a mmal connection pointer, reassigned if function successful
 * @return Returns a MMAL_STATUS_T giving result of operation
 *
 */
static MMAL_STATUS_T connect_ports(MMAL_PORT_T *output_port, MMAL_PORT_T *input_port, MMAL_CONNECTION_T **connection)
{
    MMAL_STATUS_T status;
    
    status =  mmal_connection_create(connection, output_port, input_port, MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
    
    if (status == MMAL_SUCCESS)
    {
        status =  mmal_connection_enable(*connection);
        if (status != MMAL_SUCCESS)
            mmal_connection_destroy(*connection);
    }
    
    return status;
}

/**
 * Checks if specified port is valid and enabled, then disables it
 *
 * @param port  Pointer the port
 *
 */
static void check_disable_port(MMAL_PORT_T *port)
{
    if (port && port->is_enabled)
        mmal_port_disable(port);
}


/**
 * main
 */
int old_main()
{
    // Our main data storage vessel..
    RASPISTILL_STATE state = {0};
    int exit_code = EX_OK;
    
    MMAL_STATUS_T status = MMAL_SUCCESS;
    MMAL_PORT_T *camera_still_port = NULL;
    MMAL_PORT_T *encoder_input_port = NULL;
    MMAL_PORT_T *encoder_output_port = NULL;
    
    bcm_host_init();
    
    // Register our application with the logging system
    vcos_log_register("RaspiStill", VCOS_LOG_CATEGORY);
    
    default_status(&state);
    
    std::string jpg = "test.jpg";
    state.filename = new char[jpg.length() + 1];
    strcpy(state.filename, jpg.c_str());        
    
    // Setup for sensor specific parameters
    set_sensor_defaults(&state);
    
    
    // Disable USR1 and USR2 for the moment - may be reenabled if go in to signal capture mode
    signal(SIGINT, signal_handler);
    signal(SIGUSR1, SIG_IGN);
    signal(SIGUSR2, SIG_IGN);
    
    if (state.verbose)
    {
        fprintf(stderr, "\nCamera App %s\n\n", VERSION_STRING);
        dump_status(&state);
    }
    
    // OK, we have a nice set of parameters. Now set up our components
    // We have three components. Camera, Preview and encoder.
    // Camera and encoder are different in stills/video, but preview
    // is the same so handed off to a separate module
    
    if ((status = create_camera_component(&state)) != MMAL_SUCCESS)
    {
        vcos_log_error("%s: Failed to create camera component", __func__);
        exit_code = EX_SOFTWARE;
    }
    else if ((status = create_encoder_component(&state)) != MMAL_SUCCESS)
    {
        vcos_log_error("%s: Failed to create encode component", __func__);
        destroy_camera_component(&state);
        exit_code = EX_SOFTWARE;
    }
    else
    {
        PORT_USERDATA callback_data;
        
        if (state.verbose)
            fprintf(stderr, "Starting component connection stage\n");
        
        camera_still_port   = state.camera_component->output[MMAL_CAMERA_CAPTURE_PORT];
        encoder_input_port  = state.encoder_component->input[0];
        encoder_output_port = state.encoder_component->output[0];
        
        if (status == MMAL_SUCCESS)
        {
            VCOS_STATUS_T vcos_status;
            
            if (state.verbose)
                fprintf(stderr, "Connecting camera stills port to encoder input port\n");
            
            // Now connect the camera to the encoder
            status = connect_ports(camera_still_port, encoder_input_port, &state.encoder_connection);
            
            if (status != MMAL_SUCCESS)
            {
                vcos_log_error("%s: Failed to connect camera video port to encoder input", __func__);
                goto error;
            }
            
            // Set up our userdata - this is passed though to the callback where we need the information.
            // Null until we open our filename
            callback_data.file_handle = NULL;
            callback_data.pstate = &state;
            vcos_status = vcos_semaphore_create(&callback_data.complete_semaphore, "RaspiStill-sem", 0);
            
            vcos_assert(vcos_status == VCOS_SUCCESS);
            
            if (status != MMAL_SUCCESS)
            {
                vcos_log_error("Failed to setup encoder output");
                goto error;
            }
                        
            int frame = (int)time(NULL);
            
            if (state.verbose)
                fprintf(stderr, "Opening output file %s\n", state.filename);
            
            FILE *output_file = fopen( state.filename , "wb");;
            callback_data.file_handle = output_file;
            
            if (!output_file)
            {
                // Notify user, carry on but discarding encoded output buffers
                vcos_log_error("%s: Error opening output file: %s\nNo output file will be generated\n", __func__, state.filename);
            } else {
                
                int num, q;
                
                mmal_port_parameter_set_boolean(state.encoder_component->output[0], MMAL_PARAMETER_EXIF_DISABLE, 1);                        
                
                // There is a possibility that shutter needs to be set each loop.
                /*
                 if (mmal_status_to_int(mmal_port_parameter_set_uint32(state.camera_component->control, MMAL_PARAMETER_SHUTTER_SPEED, state.camera_parameters.speed)) != MMAL_SUCCESS)
                 vcos_log_error("Unable to set shutter speed");
                 */
                
                
                // Enable the encoder output port
                encoder_output_port->userdata = (struct MMAL_PORT_USERDATA_T *)&callback_data;
                
                if (state.verbose)
                    fprintf(stderr, "Enabling encoder output port\n");
                
                // Enable the encoder output port and tell it its callback function
                status = mmal_port_enable(encoder_output_port, encoder_buffer_callback);
                
                // Send all the buffers to the encoder output port
                num = mmal_queue_length(state.encoder_pool->queue);
                
                for (q=0;q<num;q++)
                {
                    MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.encoder_pool->queue);
                    
                    if (!buffer)
                        vcos_log_error("Unable to get a required buffer %d from pool queue", q);
                    
                    if (mmal_port_send_buffer(encoder_output_port, buffer)!= MMAL_SUCCESS)
                        vcos_log_error("Unable to send a buffer to encoder output port (%d)", q);
                }
                /*
                 if (state.burstCaptureMode && frame==1)
                 {
                 mmal_port_parameter_set_boolean(state.camera_component->control,  MMAL_PARAMETER_CAMERA_BURST_CAPTURE, 1);
                 }
                 */
                
                if (state.verbose)
                    fprintf(stderr, "Starting capture %d\n", frame);
                
                if (mmal_port_parameter_set_boolean(camera_still_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS)
                {
                    vcos_log_error("%s: Failed to start capture", __func__);
                }
                else
                {
                    // Wait for capture to complete
                    // For some reason using vcos_semaphore_wait_timeout sometimes returns immediately with bad parameter error
                    // even though it appears to be all correct, so reverting to untimed one until figure out why its erratic
                    vcos_semaphore_wait(&callback_data.complete_semaphore);
                    if (state.verbose)
                        fprintf(stderr, "Finished capture %d\n", frame);
                }
                
                // Ensure we don't die if get callback with no open file
                callback_data.file_handle = NULL;
                
                if (output_file == stdout)
                {
                    fflush(output_file);
                }
                // Disable encoder output port
                status = mmal_port_disable(encoder_output_port);
            }
            
            vcos_semaphore_delete(&callback_data.complete_semaphore);
        }
        
        else
        {
            mmal_status_to_int(status);
            vcos_log_error("%s: Failed to connect camera to preview", __func__);
        }
        
    error:
        
        mmal_status_to_int(status);
        
        if (state.verbose)
            fprintf(stderr, "Closing down\n");
        
        // Disable all our ports that are not handled by connections
        check_disable_port(encoder_output_port);
        
        if (state.encoder_connection)
            mmal_connection_destroy(state.encoder_connection);
        
        
        /* Disable components */
        if (state.encoder_component)
            mmal_component_disable(state.encoder_component);
        
        if (state.camera_component)
            mmal_component_disable(state.camera_component);
        
        destroy_encoder_component(&state);
        destroy_camera_component(&state);
        
        if (state.verbose)
            fprintf(stderr, "Close down completed, all components disconnected, disabled and destroyed\n\n");
    }
    
    
    fprintf(stderr, "GPU: %d\n", get_mem_gpu());

    
    
    return exit_code;
}


int main(int argc, const char **argv) {
    return old_main();
}


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

/*
 * \file RaspiStillYUV.c
 * Command line program to capture a still frame and dump uncompressed it to file.
 * Also optionally display a preview/viewfinder of current camera input.
 *
 * \date 4th March 2013
 * \Author: James Hughes
 *
 * Description
 *
 * 2 components are created; camera and preview.
 * Camera component has three ports, preview, video and stills.
 * Preview is connected using standard mmal connections, the stills output
 * is written straight to the file in YUV 420 format via the requisite buffer
 * callback. video port is not used
 *
 * We use the RaspiCamControl code to handle the specific camera settings.
 * We use the RaspiPreview code to handle the generic preview
 */

// We use some GNU extensions (basename)
//#define _GNU_SOURCE

extern "C" {
#include "types.h"
}

#include "FlashCam.h"
#include <string>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include <sysexits.h>
#include <unistd.h>
#include <errno.h>

#define VERSION_STRING "v1.3.6"

#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"


//#include "RaspiCamControl.h"
//#include "RaspiPreview.h"
//#include "RaspiCLI.h"

#include <semaphore.h>

// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT 0
//#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2


// Stills format information
// 0 implies variable
#define STILLS_FRAME_RATE_NUM 0
#define STILLS_FRAME_RATE_DEN 1
#define PREVIEW_FRAME_RATE_NUM 0
#define PREVIEW_FRAME_RATE_DEN 1

/// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM 3

static void signal_handler(int signal_number);

typedef struct
{
    int opacity;                           /// Opacity of window - 0 = transparent, 255 = opaque
    MMAL_RECT_T previewWindow;             /// Destination rectangle for the preview window.
    MMAL_COMPONENT_T *preview_component;   /// Pointer to the created preview display component
} FLASHCAM_PREVIEW_T;


/** Structure containing all state information for the current run
 */
typedef struct
{
    unsigned int width;                     // Requested width of image
    unsigned int height;                    // requested height of image
    
    char *filename;                         // filename of output file
    int verbose;                            // !0 if want detailed run information
    int useRGB;                             // Output RGB data rather than YUV
    int settings;                           // Request settings from the camera
    int cameraNum;                          // Camera number
    int onlyLuma;                           // Only output the luma / Y plane of the YUV data
    
    FLASHCAM_PREVIEW_T preview_parameters;  // Preview setup parameters
    FLASHCAM_PARAMS_T  camera_parameters;   // Camera setup parameters
    
    MMAL_COMPONENT_T *camera_component;     // Pointer to the camera component
    
    MMAL_CONNECTION_T *preview_connection;  // Pointer to the connection from camera to preview
    MMAL_POOL_T *camera_pool;               // Pointer to the pool of buffers used by camera stills port
} RASPISTILLYUV_STATE;


/** Struct used to pass information in camera still port userdata to callback
 */
typedef struct
{
    FILE *file_handle;                   /// File handle to write buffer data to.
    VCOS_SEMAPHORE_T complete_semaphore; /// semaphore which is posted when we reach end of frame (indicates end of capture or fault)
    RASPISTILLYUV_STATE *pstate;            /// pointer to our state in case required in callback
} PORT_USERDATA;

static void display_valid_parameters(char *app_name);

/**
 * Assign a default set of parameters to the state passed in
 *
 * @param state Pointer to state structure to assign defaults to
 */
static void default_status(RASPISTILLYUV_STATE *state)
{
    if (!state)
    {
        vcos_assert(0);
        return;
    }
    
    // Default everything to zero
    memset(state, 0, sizeof(RASPISTILLYUV_STATE));
    
    // Now set anything non-zero
    state->width        = 100;
    state->height       = 100;
    state->filename     = NULL;
    state->verbose      = 1;
    state->settings     = 0;
    state->cameraNum    = 0;
    state->onlyLuma     = 0;
    
    // Setup preview window defaults
    state->preview_parameters.opacity               = 255;
    state->preview_parameters.previewWindow.x       = 0;
    state->preview_parameters.previewWindow.y       = 0;
    state->preview_parameters.previewWindow.width   = 1024;
    state->preview_parameters.previewWindow.height  = 768;
    state->preview_parameters.preview_component     = NULL;
    
    // Set up the camera_parameters to default
    FlashCam::getDefaultParams(&state->camera_parameters);
}

/**
 * Dump image state parameters to stderr. Used for debugging
 *
 * @param state Pointer to state structure to assign defaults to
 */
static void dump_status(RASPISTILLYUV_STATE *state)
{
    int i;
    if (!state)
    {
        vcos_assert(0);
        return;
    }
    
    fprintf(stderr, "Width %d, Height %d, filename %s\n", state->width,
            state->height, state->filename);    
}


/**
 *  buffer header callback function for camera control
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void camera_control_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
    fprintf(stderr, "Camera control callback  cmd=0x%08x", buffer->cmd);
    
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
    {
        vcos_log_error("Received unexpected camera control callback event, 0x%08x", buffer->cmd);
    }
    
    mmal_buffer_header_release(buffer);
}

/**
 *  buffer header callback function for camera output port
 *
 *  Callback will dump buffer data to the specific file
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void camera_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
    int complete = 0;
    // We pass our file handle and other stuff in via the userdata field.
    
    
    PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;
    
    if (pData)
    {
        int bytes_written = 0;
        int bytes_to_write = buffer->length;
        
        if (pData->pstate->onlyLuma)
            bytes_to_write = vcos_min(buffer->length, port->format->es->video.width * port->format->es->video.height);
        
        if (bytes_to_write && pData->file_handle)
        {
            mmal_buffer_header_mem_lock(buffer);
            
            bytes_written = fwrite(buffer->data, 1, bytes_to_write, pData->file_handle);
            
            mmal_buffer_header_mem_unlock(buffer);
        }
        
        // We need to check we wrote what we wanted - it's possible we have run out of storage.
        if (buffer->length && bytes_written != bytes_to_write)
        {
            vcos_log_error("Unable to write buffer to file - aborting %d vs %d", bytes_written, bytes_to_write);
            complete = 1;
        }
        
        // Check end of frame or error
        if (buffer->flags & (MMAL_BUFFER_HEADER_FLAG_FRAME_END | MMAL_BUFFER_HEADER_FLAG_TRANSMISSION_FAILED))
            complete = 1;
    }
    else
    {
        vcos_log_error("Received a camera still buffer callback with no state");
    }
    
    // release buffer back to the pool
    mmal_buffer_header_release(buffer);
    
    // and send one back to the port (if still open)
    if (port->is_enabled)
    {
        MMAL_STATUS_T status;
        MMAL_BUFFER_HEADER_T *new_buffer = mmal_queue_get(pData->pstate->camera_pool->queue);
        
        // and back to the port from there.
        if (new_buffer)
        {
            status = mmal_port_send_buffer(port, new_buffer);
        }
        
        if (!new_buffer || status != MMAL_SUCCESS)
            vcos_log_error("Unable to return the buffer to the camera still port");
    }
    
    if (complete)
    {
        vcos_semaphore_post(&(pData->complete_semaphore));
    }
}


/**
 * Create the camera component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return 0 if failed, pointer to component if successful
 *
 */
static MMAL_STATUS_T create_camera_component(RASPISTILLYUV_STATE *state)
{
    MMAL_COMPONENT_T *camera = 0;
    MMAL_ES_FORMAT_T *format;
    MMAL_PORT_T *preview_port = NULL;
    MMAL_PORT_T *still_port = NULL;
    MMAL_STATUS_T status;
    MMAL_POOL_T *pool;
    MMAL_PARAMETER_INT32_T camera_num;
    
    //FlashCam functions
    FlashCam flashcam = FlashCam();
    FLASHCAM_PARAMS_T camera_parameters_copy;
    
    /* Create the component */
    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);
    
    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("Failed to create camera component");
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
    
    preview_port = camera->output[MMAL_CAMERA_PREVIEW_PORT];
    still_port   = camera->output[MMAL_CAMERA_CAPTURE_PORT];
    
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
    
    if (status != MMAL_SUCCESS )
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
    
    //raspicamcontrol_set_all_parameters(camera, &state->camera_parameters);
    flashcam.setCamera(camera);

    flashcam.setFlashMode(MMAL_PARAM_FLASH_ON);
    
    //get params
    if ( flashcam.getAllParams(&state->camera_parameters) != 0) {
        vcos_log_error("Error getting camera params");
    } else {
        flashcam.printParams(&state->camera_parameters);
    }
    
    
    // Now set up the port formats
    
    format = preview_port->format;
    
    format->encoding         = MMAL_ENCODING_OPAQUE;
    format->encoding_variant = MMAL_ENCODING_I420;
    
    if(state->camera_parameters.speed > 6000000) {
        MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
            { 50, 1000 }, {166, 1000}};
        mmal_port_parameter_set(preview_port, &fps_range.hdr);
    } else if(state->camera_parameters.speed > 1000000) {
        MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
            { 166, 1000 }, {999, 1000}};
        mmal_port_parameter_set(preview_port, &fps_range.hdr);
    }
    
    // In this mode we are forcing the preview to be generated from the full capture resolution.
    // This runs at a max of 15fps with the OV5647 sensor.
    format->es->video.width = VCOS_ALIGN_UP(state->width, 32);
    format->es->video.height = VCOS_ALIGN_UP(state->height, 16);
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = state->width;
    format->es->video.crop.height = state->height;
    format->es->video.frame_rate.num = PREVIEW_FRAME_RATE_NUM;
    format->es->video.frame_rate.den = PREVIEW_FRAME_RATE_DEN;
    
    
    status = mmal_port_format_commit(preview_port);
    
    if (status != MMAL_SUCCESS ) {
        vcos_log_error("camera viewfinder format couldn't be set");
        goto error;
    }
    
    format = still_port->format;
    
    if(state->camera_parameters.speed > 6000000) {
        MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
            { 50, 1000 }, {166, 1000}};
        mmal_port_parameter_set(still_port, &fps_range.hdr);
    } else if(state->camera_parameters.speed > 1000000) {
        MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
            { 167, 1000 }, {999, 1000}};
        mmal_port_parameter_set(still_port, &fps_range.hdr);
    }
    
    // Set our stills format on the stills  port
    if (state->useRGB) {
        format->encoding = mmal_util_rgb_order_fixed(still_port) ? MMAL_ENCODING_RGB24 : MMAL_ENCODING_BGR24;
        format->encoding_variant = 0;  //Irrelevant when not in opaque mode
    } else {
        format->encoding = MMAL_ENCODING_I420;
        format->encoding_variant = MMAL_ENCODING_I420;
    }
    
    format->es->video.width = VCOS_ALIGN_UP(state->width, 32);
    format->es->video.height = VCOS_ALIGN_UP(state->height, 16);
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = state->width;
    format->es->video.crop.height = state->height;
    format->es->video.frame_rate.num = STILLS_FRAME_RATE_NUM;
    format->es->video.frame_rate.den = STILLS_FRAME_RATE_DEN;
    
    if (still_port->buffer_size < still_port->buffer_size_min)
        still_port->buffer_size = still_port->buffer_size_min;
    
    still_port->buffer_num = still_port->buffer_num_recommended;
    
    status = mmal_port_format_commit(still_port);
    
    if (status != MMAL_SUCCESS )
    {
        vcos_log_error("camera still format couldn't be set");
        goto error;
    }
    
    /* Enable component */
    status = mmal_component_enable(camera);
    
    if (status != MMAL_SUCCESS )
    {
        vcos_log_error("camera component couldn't be enabled");
        goto error;
    }
    
    /* Create pool of buffer headers for the output port to consume */
    pool = mmal_port_pool_create(still_port, still_port->buffer_num, still_port->buffer_size);
    
    if (!pool)
    {
        vcos_log_error("Failed to create buffer header pool for camera still port %s", still_port->name);
    }
    
    state->camera_pool      = pool;
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
static void destroy_camera_component(RASPISTILLYUV_STATE *state)
{
    if (state->camera_component)
    {
        mmal_component_destroy(state->camera_component);
        state->camera_component = NULL;
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
 * Handler for sigint signals
 *
 * @param signal_number ID of incoming signal.
 *
 */
static void signal_handler(int signal_number)
{
    if (signal_number == SIGUSR1)
    {
        // Handle but ignore - prevents us dropping out if started in none-signal mode
        // and someone sends us the USR1 signal anyway
    }
    else
    {
        // Going to abort on all other signals
        vcos_log_error("Aborting program\n");
        exit(130);
    }
}


static MMAL_STATUS_T preview_create( FLASHCAM_PREVIEW_T *state ) {
    MMAL_COMPONENT_T *preview = 0;
    MMAL_PORT_T *preview_port = NULL;
    MMAL_STATUS_T status;
    
    //set nullsink
    status = mmal_component_create("vc.null_sink", &preview);
    if (status != MMAL_SUCCESS) {
        vcos_log_error("Unable to create null sink component");
        goto error;
    }
    
    // Enable component
    status = mmal_component_enable(preview);
    if (status != MMAL_SUCCESS) {
        vcos_log_error("Unable to enable preview/null sink component (%u)", status);
        goto error;
    }
    
    //copy preview
    state->preview_component = preview;
    
    return status;
    
error:
    
    if (preview)
        mmal_component_destroy(preview);
    
    return status;    
}

/**
 * Destroy the preview component
 *
 * @param state Pointer to state control struct
 *
 */
void preview_destroy(FLASHCAM_PREVIEW_T *state)
{
    if (state->preview_component)
    {
        mmal_component_destroy(state->preview_component);
        state->preview_component = NULL;
    }
}

/**
 * main
 */
int main(int argc, const char **argv)
{
    // Our main data storage vessel..
    RASPISTILLYUV_STATE state;
    int exit_code = EX_OK;
    
    MMAL_STATUS_T status = MMAL_SUCCESS;
    MMAL_PORT_T *camera_preview_port = NULL;
    //MMAL_PORT_T *camera_video_port = NULL;
    MMAL_PORT_T *camera_still_port = NULL;
    MMAL_PORT_T *preview_input_port = NULL;
    FILE *output_file = NULL;
    
    bcm_host_init();
    
    // Register our application with the logging system
    vcos_log_register("RaspiStillYUV", VCOS_LOG_CATEGORY);
    
    // Disable USR1 for the moment - may be reenabled if go in to signal capture mode
    signal(SIGINT, signal_handler);
    signal(SIGUSR1, SIG_IGN);
    
    // Setup for sensor specific parameters
    default_status(&state);
    
    //set filename for output
    std::string jpg = "test.jpg";
    state.filename = new char[jpg.length() + 1];
    strcpy(state.filename, jpg.c_str());  
    
    if (state.verbose)
    {
        fprintf(stderr, "\nCamera App %s\n\n", VERSION_STRING);
        dump_status(&state);
    }
    
    // OK, we have a nice set of parameters. Now set up our components
    // We have two components. Camera and Preview
    // Camera is different in stills/video, but preview
    // is the same so handed off to a separate module
    
    if ((status = create_camera_component(&state)) != MMAL_SUCCESS)
    {
        vcos_log_error("%s: Failed to create camera component", __func__);
        exit_code = EX_SOFTWARE;
    }
    else if ((status = preview_create(&state.preview_parameters)) != MMAL_SUCCESS)
    {
        vcos_log_error("%s: Failed to create preview component", __func__);
        destroy_camera_component(&state);
        exit_code = EX_SOFTWARE;
    }
    else
    {
        PORT_USERDATA callback_data;
        
        if (state.verbose)
            fprintf(stderr, "Starting component connection stage\n");
        
        camera_preview_port = state.camera_component->output[MMAL_CAMERA_PREVIEW_PORT];
        camera_still_port   = state.camera_component->output[MMAL_CAMERA_CAPTURE_PORT];
        
        // Note we are lucky that the preview and null sink components use the same input port
        // so we can simple do this without conditionals
        preview_input_port  = state.preview_parameters.preview_component->input[0];
        
        // Connect camera to preview (which might be a null_sink if no preview required)
        status = connect_ports(camera_preview_port, preview_input_port, &state.preview_connection);
        
        if (status == MMAL_SUCCESS)
        {
            VCOS_STATUS_T vcos_status;
            
            // Set up our userdata - this is passed though to the callback where we need the information.
            // Null until we open our filename
            callback_data.file_handle = NULL;
            callback_data.pstate = &state;
            
            vcos_status = vcos_semaphore_create(&callback_data.complete_semaphore, "RaspiStill-sem", 0);
            vcos_assert(vcos_status == VCOS_SUCCESS);
            
            camera_still_port->userdata = (struct MMAL_PORT_USERDATA_T *)&callback_data;
            
            if (state.verbose)
                fprintf(stderr, "Enabling camera still output port\n");
            
            // Enable the camera still output port and tell it its callback function
            status = mmal_port_enable(camera_still_port, camera_buffer_callback);
            
            if (status != MMAL_SUCCESS)
            {
                vcos_log_error("Failed to setup camera output");
                goto error;
            }
            
            if (state.verbose)
                fprintf(stderr, "Starting video preview\n");
            
            
            int frame = (int)time(NULL);
            
            
            if (state.verbose)
                fprintf(stderr, "Opening output file %s\n", state.filename);
            // Technically it is opening the temp~ filename which will be ranamed to the final filename
            
            FILE *output_file = fopen( state.filename , "wb");;
            
            if (!output_file)
            {
                // Notify user, carry on but discarding encoded output buffers
                vcos_log_error("%s: Error opening output file: %s\nNo output file will be generated\n", __func__, state.filename);
            }
            
            callback_data.file_handle = output_file;
            

            if (output_file)
            {
                int num, q;
                
                // There is a possibility that shutter needs to be set each loop.
                if (mmal_port_parameter_set_uint32(state.camera_component->control, MMAL_PARAMETER_SHUTTER_SPEED, state.camera_parameters.speed) != MMAL_SUCCESS)
                    vcos_log_error("Unable to set shutter speed");
                
                
                // Send all the buffers to the camera output port
                num = mmal_queue_length(state.camera_pool->queue);
                
                for (q=0;q<num;q++)
                {
                    MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.camera_pool->queue);
                    
                    if (!buffer)
                        vcos_log_error("Unable to get a required buffer %d from pool queue", q);
                    
                    if (mmal_port_send_buffer(camera_still_port, buffer)!= MMAL_SUCCESS)
                        vcos_log_error("Unable to send a buffer to camera output port (%d)", q);
                }
                
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
        
        if (output_file)
            fclose(output_file);
        
        // Disable all our ports that are not handled by connections
        
        if (state.preview_connection)
            mmal_connection_destroy(state.preview_connection);
        
        /* Disable components */
        if (state.preview_parameters.preview_component)
            mmal_component_disable(state.preview_parameters.preview_component);
        
        if (state.camera_component)
            mmal_component_disable(state.camera_component);
        
        preview_destroy(&state.preview_parameters);
        destroy_camera_component(&state);
        
        if (state.verbose)
            fprintf(stderr, "Close down completed, all components disconnected, disabled and destroyed\n\n");
    }
    
    //if (status != MMAL_SUCCESS)
    //    raspicamcontrol_check_configuration(128);
    
    return exit_code;
}





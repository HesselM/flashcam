/**********************************************************
 Software developed by Hessel van der Molen
 Main author Hessel van der Molen (hmolen.science at gmail dot com)
 This software is released under BSD license as expressed below
 -------------------------------------------------------------------
 Copyright (c) 2017, Hessel van der Molen
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:
 1. Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 3. All advertising materials mentioning features or use of this software
 must display the following acknowledgement:
 
 This product includes software developed by Hessel van der Molen
 
 4. None of the names of the author or irs contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY Hessel van der Molen ''AS IS'' AND ANY
 EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL AVA BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************/

#include "FlashCam.h"
#include "FlashCam_util_mmal.h"

#include "bcm_host.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"

#include <stdio.h>
#include <sysexits.h>

/*
 * Constructor
 */
FlashCam::FlashCam(){    
    clear();
}

/*
 * Destructor
 */
FlashCam::~FlashCam(){
    //allow callback/buffer data to be emptied..
    sleep(1);
    //cleanup.
    destroyComponents();
    FlashCamPLL::destroy();
    vcos_semaphore_delete(&_userdata.sem_capture);
}

void FlashCam::clear() {    
    if (_active) {
        fprintf(stderr, "%s: Cannot clear FlashCam while it is capturing.\n", __func__);
        return;
    }
    
    if (_initialised) {
        //allow callback/buffer data to be emptied..
        sleep(1);
        //clear old values
        destroyComponents();
        FlashCamPLL::destroy();
        vcos_semaphore_delete(&_userdata.sem_capture);
    }
    
    _initialised = false;
    _active      = false;

    
    FlashCamPLL::init(&_state);
    
    //init with default parameter set;
    getDefaultSettings(&_settings);
    
    //disable verbose as it might confuse user.
    unsigned int v = _settings.verbose;
    _settings.verbose = 0;
    
    //init with default settings
    setSettings(&_settings); //implicitly resets/configures camera
    
    //reset verbose
    _settings.verbose = v;
}


int FlashCam::resetCamera() {
    int status;
    
    if (_active) {
        fprintf(stderr, "%s: Cannot reset camera while it is capturing.\n", __func__);
        return FlashCamMMAL::mmal_to_int(MMAL_EINVAL);
    }
    
    if (_settings.verbose)
        fprintf(stdout, "%s: (re)setting/initializing components.\n", __func__);
    
    // setup / reset camera (return upon error)
    if (status = setupComponents())
        return status; 
    
    if (_settings.verbose)
        fprintf(stdout, "%s: (re)setting/initializing mode.\n", __func__);
    
    // enable / reset selected mode
    if (status = setSettingCaptureMode( _settings.mode ))
        return status;
    
    // update parameters
    if (status = setParams(&_params))
        return status;
    
    if (_settings.verbose)
        fprintf(stdout, "%s: Succes.\n", __func__);
    
    //succes
    return FlashCamMMAL::mmal_to_int(MMAL_SUCCESS);
    
}


// setup the camera with specified settings
int FlashCam::setupComponents() {
    
    // Is camera active?
    if (_active) {
        fprintf(stderr, "%s: Cannot reset camera while it is capturing.\n", __func__);
        return FlashCamMMAL::mmal_to_int(MMAL_EINVAL);
    }
    
    // Only update these settings when initialising for the first time
    if (!_initialised) {        
        //print that we are starting
        if (_settings.verbose)
            fprintf(stdout, "\n FlashCam Version: %s\n\n", FLASHCAM_VERSION_STRING);

        bcm_host_init();
        
        //register to VCOS for logging
        vcos_log_register("FlashCam", VCOS_LOG_CATEGORY);
        
        // create & init semaphore
        if (vcos_semaphore_create(&_userdata.sem_capture, "FlashCam_sem_captured", 0) != VCOS_SUCCESS) {
            vcos_log_error("%s: Failed to create semaphore", __func__);
            return FlashCamMMAL::mmal_to_int(MMAL_EINVAL);
        }    
                
        //setup default camera params 
        getDefaultParams(&_params);
    }

    //clear current setup
    destroyComponents();

    // init/reset private variables
    //_initialised          : set at the end of this function.
    //_active = false       : initialised by constructor. Set when camera is recording.
    //_params = default     : initialised by constructor. Updated by User.
    //_settings             : should be set before calling setupComponents().
    //_camera_component     : (re)set by destroyComponents()
    //_preview_component    : (re)set by destroyComponents()
    //_preview_connection   : (re)set by destroyComponents()
    //_camera_pool          : (re)set by destroyComponents()
    //_framebuffer          : (re)set by destroyComponents()
    //_opengl_queue         : (re)set by destroyComponents()

    //set userdata
    _userdata.params            = &_params;
    _userdata.settings          = &_settings;
    _userdata.camera_pool       = NULL;
    _userdata.framebuffer       = NULL;
    _userdata.framebuffer_size  = 0;
    _userdata.framebuffer_idx   = 0;
    _userdata.callback          = NULL;
    
#ifdef BUILD_FLASHCAM_WITH_OPENGL
    _userdata.callback_egl      = NULL;
    _userdata.opengl_queue      = NULL;
    FlashCamOpenGL::init(&_state);
#endif    
    
    //Status flags
    MMAL_STATUS_T status        = MMAL_SUCCESS;
        
    //setup camera
    // - internally sets:
    //      _camera_component
    //      _camera_pool
    //      _userdata.camera_pool
    //      _userdata.framebuffer
    //      _userdata.framebuffer_size
    if ((status = setupComponentCamera()) != MMAL_SUCCESS)  {
        vcos_log_error("%s: Failed to create camera component", __func__);
        destroyComponents();
        return FlashCamMMAL::mmal_to_int(status);
    }
    
    //setup preview (nullsink!)
    // -> But we need it as it is used for measurements such as awb
    // internally sets:
    //      _preview_component
    if ((status = setupComponentPreview()) != MMAL_SUCCESS) {
        vcos_log_error("%s: Failed to create preview component", __func__);
        destroyComponents();
        return FlashCamMMAL::mmal_to_int(status);
    }
    
    if (_settings.verbose)
        fprintf(stdout, "%s: Starting component connection stage\n", __func__);
    
    //get ports
    MMAL_PORT_T *preview_port       = _camera_component->output[MMAL_CAMERA_PREVIEW_PORT];   
    MMAL_PORT_T *preview_port_input = _preview_component->input[0];
    
    // Connect preview
    if ((status = connectPorts(preview_port, preview_port_input, &_preview_connection)) != MMAL_SUCCESS) {
        vcos_log_error("%s: Failed to connect preview port", __func__);
        destroyComponents();
        return FlashCamMMAL::mmal_to_int(status);
    }
    
    if (_settings.verbose)
        fprintf(stdout, "%s: Finished setup\n", __func__);
    
    _initialised = true;

    return EX_OK;
}


//Camera setup function
MMAL_STATUS_T FlashCam::setupComponentCamera() {
    MMAL_STATUS_T status;
    
    // Create Component
    if ((status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &_camera_component)) != MMAL_SUCCESS) {
        vcos_log_error("%s: Failed to create camera component", __func__);
        destroyComponents();        
        return status;
    }
    
    // Select Camera
    if ( setCameraNum(_params.cameranum) ) {
        vcos_log_error("%s: Could not select camera %d", __func__, _params.cameranum);
        destroyComponents();        
        return MMAL_EINVAL;
    }
    
    // Validate outputs are available
    if (!_camera_component->output_num){
        vcos_log_error("%s: Camera doesn't have output ports", __func__);
        destroyComponents();        
        return MMAL_EINVAL;
    }
    
    // Do we want setting-update messages?
    if (setChangeEventRequest(MMAL_PARAMETER_CAMERA_SETTINGS, _settings.update)) {
        vcos_log_error("%s: No camera settings events", __func__);
    }
    
    // Enable the camera, and tell it its control callback function
    if ((status = mmal_port_enable(_camera_component->control, FlashCam::control_callback)) != MMAL_SUCCESS ) {
        vcos_log_error("%s: Unable to enable control port (%u)", __func__, status);
        destroyComponents();        
        return status;
    }
    
    // Set sensormode
    if ( setSensorMode(_settings.sensormode) ) {
        vcos_log_error("%s: Could not set sensormode %d", __func__, _settings.sensormode);
        destroyComponents();        
        return MMAL_EINVAL;
    }
    
    // Setup ports
    MMAL_PORT_T *preview_port = _camera_component->output[MMAL_CAMERA_PREVIEW_PORT];
    MMAL_PORT_T   *video_port = _camera_component->output[MMAL_CAMERA_VIDEO_PORT  ];
    MMAL_PORT_T *capture_port = _camera_component->output[MMAL_CAMERA_CAPTURE_PORT];
    
    // Align height/width
    _settings.width  = VCOS_ALIGN_UP(_settings.width , 32);
    _settings.height = VCOS_ALIGN_UP(_settings.height, 16);
    
    if (_settings.verbose)
        fprintf(stdout, "%s: Aligned image size: %d x %d (w x h) \n" , __func__, _settings.width , _settings.height);
    
    // setup the camera configuration
    MMAL_PARAMETER_CAMERA_CONFIG_T cam_config =
    {
        { MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) },
        .max_stills_w                           = _settings.width,
        .max_stills_h                           = _settings.height,
        .stills_yuv422                          = 0,
        .one_shot_stills                        = 1,
        .max_preview_video_w                    = _settings.width,
        .max_preview_video_h                    = _settings.height,
        .num_preview_video_frames               = 3,
        .stills_capture_circular_buffer_height  = 0,
        .fast_preview_resume                    = 0,
        .use_stc_timestamp                      = MMAL_PARAM_TIMESTAMP_MODE_RAW_STC
    };
    
    if ( setCameraConfig( &cam_config ) ) {
        vcos_log_error("%s: Could not setup camera", __func__);
        destroyComponents();        
        return MMAL_EINVAL;
    }
    
    // setup port-formats
    MMAL_ES_FORMAT_T *format = preview_port->format;
    
    //Preview format
    format->encoding                    = MMAL_ENCODING_OPAQUE;
    format->encoding_variant            = MMAL_ENCODING_I420;        
    format->es->video.width             = _settings.width;
    format->es->video.height            = _settings.height;
    format->es->video.crop.x            = 0;
    format->es->video.crop.y            = 0;
    format->es->video.crop.width        = _settings.width;
    format->es->video.crop.height       = _settings.height;
    format->es->video.frame_rate.num    = PREVIEW_FRAME_RATE_NUM;
    format->es->video.frame_rate.den    = PREVIEW_FRAME_RATE_DEN;
        
    //Update preview-port with set format
    if ((status = mmal_port_format_commit(preview_port)) != MMAL_SUCCESS ) {
        vcos_log_error("%s: Preview format couldn't be set (%u)", __func__, status);
        destroyComponents();        
        return status;
    }
    
    //Video format ==> same as Preview, except for encoding (YUV!)
    format = video_port->format;
    if (_settings.useOpenGL) {
        //For openGL we require the OPAQUE (= special GPU format) encoding.
        format->encoding                = MMAL_ENCODING_OPAQUE;
    } else {
        format->encoding                = MMAL_ENCODING_I420;
    }
    format->encoding_variant            = MMAL_ENCODING_I420;     
    format->es->video.width             = _settings.width;
    format->es->video.height            = _settings.height;
    format->es->video.crop.x            = 0;
    format->es->video.crop.y            = 0;
    format->es->video.crop.width        = _settings.width;
    format->es->video.crop.height       = _settings.height;
    format->es->video.frame_rate.num    = VIDEO_FRAME_RATE_NUM;
    format->es->video.frame_rate.den    = VIDEO_FRAME_RATE_DEN;
    
#ifdef BUILD_FLASHCAM_WITH_OPENGL
    /* Enable ZERO_COPY mode on the preview port which instructs MMAL to only
     * pass the 4-byte opaque buffer handle instead of the contents of the opaque
     * buffer.
     * The opaque handle is resolved on VideoCore by the GL driver when the EGL
     * image is created.
     */
    if (_settings.useOpenGL) {
        if ((status = mmal_port_parameter_set_boolean(video_port, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE)) != MMAL_SUCCESS ) {
            vcos_log_error("%s: Failed to enable zero copy on video port (%u)", __func__, status);
            destroyComponents();        
            return status;
        }
    }
#endif
    
    //Update capture-port with set format
    if ((status = mmal_port_format_commit(video_port)) != MMAL_SUCCESS ) {
        vcos_log_error("%s: Video format couldn't be set (%u)", __func__, status);
        destroyComponents();        
        return status;
    }
    
    //Capture format ==> same as Preview, except for encoding (YUV!)    
    format = capture_port->format;
    format->encoding                    = MMAL_ENCODING_I420;
    format->encoding_variant            = MMAL_ENCODING_I420;     
    format->es->video.width             = _settings.width;
    format->es->video.height            = _settings.height;
    format->es->video.crop.x            = 0;
    format->es->video.crop.y            = 0;
    format->es->video.crop.width        = _settings.width;
    format->es->video.crop.height       = _settings.height;
    format->es->video.frame_rate.num    = CAPTURE_FRAME_RATE_NUM;
    format->es->video.frame_rate.den    = CAPTURE_FRAME_RATE_DEN;
    
    //Update capture-port with set format
    if ((status = mmal_port_format_commit(capture_port)) != MMAL_SUCCESS ) {
        vcos_log_error("%s: Capture format couldn't be set (%u)", __func__, status);
        destroyComponents();        
        return status;
    }
    
    //Enable camera
    if ((status = mmal_component_enable(_camera_component)) != MMAL_SUCCESS ) {
        vcos_log_error("%s: Camera component couldn't be enabled (%u)", __func__, status);
        destroyComponents();        
        return status;
    }
    
    // YUV-data framesize 
    _userdata.framebuffer_size = VCOS_ALIGN_UP(_settings.width * _settings.height * 1.5, 32);    
    
    //create buffer for image
    _framebuffer = new unsigned char[_userdata.framebuffer_size];
    if (!_framebuffer) {
        vcos_log_error("%s: Failed to allocate image buffer", __func__);
        destroyComponents();        
        return MMAL_ENOMEM;
    } 
    
    //update userdata with framebuffer
    _userdata.framebuffer = _framebuffer;
    
    if (_settings.verbose)
        fprintf(stdout, "%s: Success.\n", __func__);
    return MMAL_SUCCESS;
}

//Preview setup functions
MMAL_STATUS_T FlashCam::setupComponentPreview() {
    MMAL_STATUS_T status;
    
    //set nullsink
    if ((status = mmal_component_create("vc.null_sink", &_preview_component)) != MMAL_SUCCESS) {
        vcos_log_error("%s: Unable to create null sink component (%u)", __func__, status);
        destroyComponents();        
        return status;
    }
    
    // Enable component
    if ((status = mmal_component_enable( _preview_component )) != MMAL_SUCCESS) {
        vcos_log_error("%s: Unable to enable preview/null sink component (%u)", __func__, status);
        destroyComponents();        
        return status;
    }
    
    return MMAL_SUCCESS;
}


void FlashCam::destroyComponents() {
    if (_settings.verbose)
        fprintf(stdout, "%s: Clearing components\n", __func__);
    
    //reset init flag    
    _initialised = false;
    
    // Disable connections
    if (_preview_connection) {
        mmal_connection_destroy(_preview_connection);
    }
    
    // Disable ports
    if (_preview_component) {
        mmal_component_disable( _preview_component );
        mmal_component_destroy( _preview_component );
    }
    
    if (_camera_component) {
        mmal_component_disable( _camera_component );
        mmal_component_destroy( _camera_component );
    }
    
    // Disable pool
    if (_camera_pool) {
        mmal_pool_destroy( _camera_pool );
    }
    
    // Clear buffer
    if (_framebuffer) 
        delete[] _framebuffer;
    
#ifdef BUILD_FLASHCAM_WITH_OPENGL
    if (_opengl_queue) {
        mmal_queue_destroy( _opengl_queue );
        _opengl_queue = NULL;
    }
    FlashCamOpenGL::destroy();
#endif

    // Reset pointers
    _preview_connection = NULL;
    _preview_component  = NULL;
    _camera_component   = NULL;
    _camera_pool        = NULL;
    _framebuffer        = NULL;
    
    if (_settings.verbose)
        fprintf(stdout, "%s: Components cleared\n", __func__);
}

/*
 * void FlashCam::control_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
 *  Callback function for control-events
 */
void FlashCam::control_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    //fprintf(stderr, "Camera control callback  cmd=0x%08x", buffer->cmd);
    
    //check received event
    if (buffer->cmd == MMAL_EVENT_PARAMETER_CHANGED) {        
        MMAL_EVENT_PARAMETER_CHANGED_T *param = (MMAL_EVENT_PARAMETER_CHANGED_T *)buffer->data;
        switch (param->hdr.id) {
            case MMAL_PARAMETER_CAMERA_SETTINGS:
            {
                MMAL_PARAMETER_CAMERA_SETTINGS_T *settings = (MMAL_PARAMETER_CAMERA_SETTINGS_T*)param;
                vcos_log_error("%s: Exposure now %u, analog gain %u/%u, digital gain %u/%u",
                               __func__, settings->exposure,
                               settings->analog_gain.num, settings->analog_gain.den,
                               settings->digital_gain.num, settings->digital_gain.den);
                vcos_log_error("%s: AWB R=%u/%u, B=%u/%u", __func__, 
                               settings->awb_red_gain.num, settings->awb_red_gain.den,
                               settings->awb_blue_gain.num, settings->awb_blue_gain.den
                               );
            }
                break;
                
            default:
            {
                vcos_log_error("%s: Received unexpected updated: id=%d", __func__, param->hdr.id);
            }
                break;
        }
    } else if (buffer->cmd == MMAL_EVENT_ERROR) {
        vcos_log_error("%s: No data received from sensor. Check all connections, including the Sunny one on the camera board", __func__);
    } else {
        vcos_log_error("%s: Received unexpected camera control callback event, 0x%08x", __func__, buffer->cmd);
    }
    
    //release header
    mmal_buffer_header_release(buffer);
}

/*
 * void FlashCam::control_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
 *  Callback function for buffer-events (that is, camera returned an image)
 */
void FlashCam::buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    int abort           = 0; //flag for detecting if we need to abort due to error
    int complete        = 0; //flag for detecting if a full frame is recieved
    int discard         = 0; //flag for detecting if we need to discard buffer
    int max_idx         = 0; //flag for detecting if _framebuffer is out of memory
    uint64_t presentationtime = 0;
    
    //retrieve userdata
    FLASHCAM_PORT_USERDATA_T *userdata = (FLASHCAM_PORT_USERDATA_T *)port->userdata;
          
    //is userdata properly set?
    if (userdata) {
        
        // Are there bytes to write?
        if (buffer->length) {

            bool pll_state = false;
#ifdef BUILD_FLASHCAM_WITH_PLL
            FlashCamPLL::update(buffer->pts, &pll_state);
#endif
            
            //OpenGL processing?
            if (userdata->settings->useOpenGL) {
#ifdef BUILD_FLASHCAM_WITH_OPENGL      
                unsigned int length = mmal_queue_length(userdata->opengl_queue);
                //fprintf(stdout, "%s: QueueSize - %d (%d)  \n", __func__, length, port->buffer_num);
                
                //push to queue, unlock buffer, update PLL and return.
                if ( length + 1 < port->buffer_num) {
                    
                    // set FlashCam data in Buffer.
                    FLASHCAM_OPENGL_BUF_T* glb = FlashCamOpenGL::getOpenGLBuffer();
                    
                    if (glb != NULL) {
                        glb->pll_state = pll_state;
                        glb->buffer    = buffer;
                    } else {
                        vcos_log_error("%s: No OpenGL buffer available in pool." , __func__);
                   }
                    
                    //push buffer to OpenGL queue for processing
                    mmal_queue_put(userdata->opengl_queue, &glb->glb_mmal_buffer);
                    
                    fprintf(stdout, "%s: Posting...\n", __func__);
                    vcos_semaphore_post(&(userdata->sem_capture));

                    //buffer released by OpenGL worker.
                    return;
                } 
                fprintf(stdout, "%s: DISCARD! \n", __func__);
#else 
                vcos_log_error("%s: OpenGL Support not build." , __func__);
#endif
                discard = 1;
                // `normal` processing
            } else {
                    
                //lock buffer --> callback is async!
                mmal_buffer_header_mem_lock(buffer);
                
                // We are decoding YUV packages
                // - 4/6 = Y
                // - 1/6 = U
                // - 1/6 = V
                unsigned int length_Y = (buffer->length << 2) / 6;
                unsigned int length_U = (buffer->length - length_Y) >> 1;
                unsigned int length_V = length_U;
                
                unsigned int offset_Y = userdata->framebuffer_idx;
                unsigned int offset_U = userdata->settings->height * userdata->settings->width * 1.00 + (userdata->framebuffer_idx >> 2);
                unsigned int offset_V = userdata->settings->height * userdata->settings->width * 1.25 + (userdata->framebuffer_idx >> 2);
                                                    
                //max index to be written
                max_idx = offset_V + length_V;
                
                //does it fit in buffer?
                if ( max_idx > userdata->framebuffer_size ) {
                    vcos_log_error("%s: Framebuffer full (%d > %d) - aborting.." , __func__, max_idx , userdata->framebuffer_size );
                    abort = 1;
                } else {
                    //copy Y
                    memcpy ( &userdata->framebuffer[offset_Y] , &buffer->data[0]                   , length_Y );
                    //copy U
                    memcpy ( &userdata->framebuffer[offset_U] , &buffer->data[length_Y]            , length_U );
                    //copy V
                    memcpy ( &userdata->framebuffer[offset_V] , &buffer->data[length_Y + length_U] , length_V );
                    //update index
                    userdata->framebuffer_idx += length_Y;
                }

                //done with data..
                mmal_buffer_header_mem_unlock(buffer);
            }
        }
        
        if (discard == 0) {
            // Check end of frame or error
            if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_TRANSMISSION_FAILED)    
                abort = 1;
            
            if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END)              
                complete = 1;
            
            presentationtime = buffer->pts;
        }
    } else {
        vcos_log_error("%s: Received a camera still buffer callback with no state", __func__);
    }

    // release buffer back to the pool
    mmal_buffer_header_release(buffer);
    
    // and send one back to the port (if still open)
    if (port->is_enabled) {
        MMAL_STATUS_T status;
        MMAL_BUFFER_HEADER_T *new_buffer = mmal_queue_get(userdata->camera_pool->queue);
        
        // and back to the port from there.
        if (new_buffer)
            status = mmal_port_send_buffer(port, new_buffer);
        
        if (!new_buffer || status != MMAL_SUCCESS)
            vcos_log_error("%s: Unable to return the buffer to the camera still port", __func__);
    }
    
    if (discard == 0) {
        //post that we are done
        if (abort) {
            vcos_semaphore_post(&(userdata->sem_capture));
        } else if (complete) {        
            if (userdata->callback)
                userdata->callback( userdata->framebuffer , userdata->settings->width , userdata->settings->height);
            
            //release semaphore
            userdata->framebuffer_idx = 0;
            vcos_semaphore_post(&(userdata->sem_capture));
        }
    }
}



// Setup connection between Input/Output ports
MMAL_STATUS_T FlashCam::connectPorts(MMAL_PORT_T *output_port, MMAL_PORT_T *input_port, MMAL_CONNECTION_T **connection) {    
    MMAL_STATUS_T status = mmal_connection_create(connection, output_port, input_port, MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
    if (status == MMAL_SUCCESS) {
        if ((status =  mmal_connection_enable(*connection)) != MMAL_SUCCESS)
            mmal_connection_destroy(*connection);
    }
    return status;
}


/*
 * int FlashCam::capture( void )
 * 
 * Capture an image
 *
 */
int FlashCam::startCapture() {
    int status;
    
    if (!_initialised) {
        fprintf(stderr, "%s: Camera not initialised.\n", __func__);
        return FlashCamMMAL::mmal_to_int(MMAL_EINVAL);
    }
    
    // Is camera active?
    if (_active) {
        fprintf(stderr, "%s: Camera already capturing.\n", __func__);
        return FlashCamMMAL::mmal_to_int(MMAL_EINVAL);
    }

    if (_settings.verbose)
        fprintf(stdout, "%s: Starting capture\n", __func__);

    //update state
    _state.settings = &_settings;
    _state.params   = &_params;
    _state.userdata = &_userdata;

    if (_settings.mode == FLASHCAM_MODE_VIDEO) {
        _state.port = _camera_component->output[MMAL_CAMERA_VIDEO_PORT];
    } else if ( _settings.mode = FLASHCAM_MODE_CAPTURE ) { 
        _state.port = _camera_component->output[MMAL_CAMERA_CAPTURE_PORT];
    } else {
        fprintf(stderr, "%s: Cannot start camera. Unknown mode (%u)\n", __func__, _settings.mode);
        return FlashCamMMAL::mmal_to_int(MMAL_EINVAL);
    }
    
#ifdef BUILD_FLASHCAM_WITH_OPENGL
    //start EGL thread for processing
    if (_settings.useOpenGL) {
        FlashCamOpenGL::start();
    }
#endif 
        
#ifdef BUILD_FLASHCAM_WITH_PLL
    if (_settings.mode == FLASHCAM_MODE_VIDEO) {        
        if (FlashCamPLL::start()) {
            fprintf(stderr, "%s: PLL cannot be started.\n", __func__);
            return FlashCamMMAL::mmal_to_int(MMAL_EINVAL);
        }
    }
#endif
     
    //start camera
    if (status = setCapture(_state.port, 1)) {
        vcos_log_error("%s: Failed to start video stream", __func__);
        return status;
    }    
    _active = true;
        
    // When in capturemode: Wait for capture to complete
    if ( _settings.mode == FLASHCAM_MODE_CAPTURE ) { 
        vcos_semaphore_wait(&_userdata.sem_capture);
        _active = false;
    } 
    
    if (_settings.verbose)
        fprintf(stdout, "%s: Success\n", __func__);

    return FlashCamMMAL::mmal_to_int(MMAL_SUCCESS);
}


int FlashCam::stopCapture() {
    int status;
    
    if (!_initialised) {
        fprintf(stderr, "%s: Camera not initialised.\n", __func__);
        return FlashCamMMAL::mmal_to_int(MMAL_EINVAL);
    }
        
    // Is camera active?
    if (!_active) {
        fprintf(stderr, "%s: Camera not capturing.\n", __func__);
        return FlashCamMMAL::mmal_to_int(MMAL_EINVAL);
    }
    
    if (_settings.verbose)
        fprintf(stdout, "%s: Stopping stream.\n", __func__);
    
    if (_settings.mode == FLASHCAM_MODE_VIDEO) {
        
        //shutdown PLL before camera, as PLL has some internal Camera dependencies.
        // Shutting down camera will crash PLL.
#ifdef BUILD_FLASHCAM_WITH_PLL
        if (FlashCamPLL::stop()) {
            fprintf(stderr, "%s: PLL cannot be stopped.\n", __func__);
            return FlashCamMMAL::mmal_to_int(MMAL_EINVAL);
        }
#endif
        
        //stop video
        if (status = setCapture(_camera_component->output[MMAL_CAMERA_VIDEO_PORT], 0)) {
            vcos_log_error("%s: Failed to stop camera", __func__);
            return status;
        }        
        
    } else if ( _settings.mode = FLASHCAM_MODE_CAPTURE ) { 
        //stop capture
        if (status = setCapture(_camera_component->output[MMAL_CAMERA_CAPTURE_PORT], 0)) {
            vcos_log_error("%s: Failed to stop camera", __func__);
            return status;
        }        
        
    } else {
        //unknown mode..
        fprintf(stderr, "%s: Cannot stop camera. Unknown mode (%u)\n", __func__, _settings.mode);
        return FlashCamMMAL::mmal_to_int(MMAL_EINVAL);
    }
    
#ifdef BUILD_FLASHCAM_WITH_OPENGL
    //Stop EGL thread
    // This needs to be done after shutting down the camera, 
    //  ensuring that pending frames/buffers are processed
    //  and pushed to the appropiate inter-thread queues. 
    // Sleeping is required to provide all async mmal-related systems to terminate.
    usleep(1000000);
    FlashCamOpenGL::stop();
#endif 
    
    //camera inactive
    _active = false;
    
    if (_settings.verbose)
        fprintf(stdout, "%s: Success\n", __func__);
    
    return FlashCamMMAL::mmal_to_int(MMAL_SUCCESS);
}



void FlashCam::setFrameCallback(FLASHCAM_CALLBACK_T callback) {
    if (_active) return; //no changer/reset while in capturemode
    _userdata.callback = callback;
}

#ifdef BUILD_FLASHCAM_WITH_OPENGL
void FlashCam::setFrameCallback(FLASHCAM_CALLBACK_EGL_T callback) {
    if (_active) return; //no changer/reset while in capturemode
    _userdata.callback_egl = callback;
}
#endif 

void FlashCam::resetFrameCallback() {
    if (_active) return; //no changer/reset while in capturemode
    _userdata.callback = NULL;
#ifdef BUILD_FLASHCAM_WITH_OPENGL
    _userdata.callback_egl = NULL;
#endif 
}



/* SETTING MANAGEMENT */

void FlashCam::getDefaultSettings(FLASHCAM_SETTINGS_T *settings) {
    settings->width         = 640;
    settings->height        = 480;
    settings->verbose       = 1;
    settings->update        = 0;
    settings->mode          = FLASHCAM_MODE_CAPTURE;
    settings->useOpenGL     = 0;
#ifdef BUILD_FLASHCAM_WITH_PLL
    FlashCamPLL::getDefaultSettings(settings);
#endif    
    
}

void FlashCam::printSettings(FLASHCAM_SETTINGS_T *settings) {
    fprintf(stdout, "Width        : %d\n", settings->width);
    fprintf(stdout, "Height       : %d\n", settings->height);
    fprintf(stdout, "Verbose      : %d\n", settings->verbose);
    fprintf(stdout, "Update       : %d\n", settings->update);
    fprintf(stdout, "Camera-Mode  : %d\n", settings->mode);    
    fprintf(stdout, "OpenGL       : %d\n", settings->useOpenGL);    
#ifdef BUILD_FLASHCAM_WITH_PLL
    FlashCamPLL::printSettings(settings);
#endif    
}

#ifdef PLLTUNE
void FlashCam::getInternalState( FLASHCAM_INTERNAL_STATE_T** state ) {
    *state = &_state;
}
#endif

int FlashCam::setSettings(FLASHCAM_SETTINGS_T *settings) {

    //update settings
    memcpy(&_settings, settings, sizeof(FLASHCAM_SETTINGS_T));

    if (_settings.verbose)
        fprintf(stdout, "%s: (re)setting camera.\n", __func__);

    //reset camera
    return resetCamera();
}

int FlashCam::getSettings(FLASHCAM_SETTINGS_T *settings) {
    memcpy(settings, &_settings, sizeof(FLASHCAM_SETTINGS_T));
}


//To update size we need to reset & re-initialise all components
int FlashCam::setSettingSize( unsigned int  width, unsigned int  height ) {
    
    //update settings
    _settings.width  = width;
    _settings.height = height;
    
    if (_settings.verbose)
        fprintf(stdout, "%s: Updating size to: %u x %u (w x h)\n", __func__, width, height);
    
    //reset camera
    return resetCamera();
}

int FlashCam::getSettingSize( unsigned int *width, unsigned int *height ) {
    *width  = _settings.width;
    *height = _settings.height;
    return FlashCamMMAL::mmal_to_int(MMAL_SUCCESS);    
}

int FlashCam::setSettingVerbose( int  verbose ) {
    _settings.verbose = verbose;
    return FlashCamMMAL::mmal_to_int(MMAL_SUCCESS);    
}

int FlashCam::getSettingVerbose( int *verbose ) {
    *verbose = _settings.verbose;
    return FlashCamMMAL::mmal_to_int(MMAL_SUCCESS);    
}

int FlashCam::setSettingUpdate( int  update ) {
    _settings.update = update;
    return setChangeEventRequest(MMAL_PARAMETER_CAMERA_SETTINGS, _settings.update); 
}

int FlashCam::getSettingUpdate( int *update ) {
    *update = _settings.update;
    return FlashCamMMAL::mmal_to_int(MMAL_SUCCESS);

}

int FlashCam::setSettingCaptureMode( FLASHCAM_MODE_T  mode ) {
    MMAL_STATUS_T status;
    
    // Is camera active?
    if (_active) {
        fprintf(stderr, "%s: Cannot change camera mode while being in use\n", __func__);
        return FlashCamMMAL::mmal_to_int(MMAL_EINVAL);
    }
    
    // Is camera initialised?
    if (!_initialised) {
        fprintf(stderr, "%s: Components not initialised\n", __func__);
        return FlashCamMMAL::mmal_to_int(MMAL_EINVAL);
    }
    
    _settings.mode = mode;
    
    //Setup userdata / camera
    MMAL_PORT_T *old_port = NULL;     
    MMAL_PORT_T *new_port = NULL;     
    
    if (_settings.mode == FLASHCAM_MODE_VIDEO) {
        //video mode..
        if (_settings.verbose)
            fprintf(stdout, "%s: Enabling video-mode.\n", __func__);
        
        new_port = _camera_component->output[MMAL_CAMERA_VIDEO_PORT];
        old_port = _camera_component->output[MMAL_CAMERA_CAPTURE_PORT];
        
        //set buffer sizes
        if (new_port->buffer_size < new_port->buffer_size_min)
            new_port->buffer_size = new_port->buffer_size_min;
        
        if (new_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
            new_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;
        
    } else if (_settings.mode == FLASHCAM_MODE_CAPTURE) {
        //capture mode..
        if (_settings.verbose)
            fprintf(stdout, "%s: Enabling capture-mode.\n", __func__);
        
        old_port = _camera_component->output[MMAL_CAMERA_VIDEO_PORT];
        new_port = _camera_component->output[MMAL_CAMERA_CAPTURE_PORT];
        
        //set buffer sizes
        if (new_port->buffer_size < new_port->buffer_size_min)
            new_port->buffer_size = new_port->buffer_size_min;
        
        if (new_port->buffer_num < new_port->buffer_num_recommended)
            new_port->buffer_num = new_port->buffer_num_recommended;
        
    } else {
        //unknown mode..
        fprintf(stderr, "%s: Cannot enable camera. Unknown mode (%u)\n", __func__, _settings.mode);
        return FlashCamMMAL::mmal_to_int(MMAL_EINVAL);
    }
    
    // Disable old port
    if ( old_port && old_port->is_enabled )
        mmal_port_disable( old_port );
    
    // Disable old pool
    if ( _camera_pool )
        mmal_pool_destroy( _camera_pool );
    
    // Pool/Buffer sizes 
    if ( _settings.verbose ) {
        fprintf(stdout, "%s: - Pool size  : %d\n", __func__, new_port->buffer_num);
        fprintf(stdout, "%s: - Buffer size: %d\n", __func__, new_port->buffer_size);
        fprintf(stdout, "%s: - Total size : %d\n", __func__, new_port->buffer_num * new_port->buffer_size);
    }
    
    // Create pool of buffer headers for the output port to consume
    _camera_pool = mmal_port_pool_create(new_port, new_port->buffer_num, new_port->buffer_size);
    if ( !_camera_pool ) {
        vcos_log_error("%s: Failed to create buffer header pool", __func__);
    } else {
        _userdata.camera_pool = _camera_pool;
    }
    
#ifdef BUILD_FLASHCAM_WITH_OPENGL
    if (_settings.useOpenGL) {
        _opengl_queue = mmal_queue_create();
        if (! _opengl_queue ) {
            vcos_log_error("Error allocating OpenGL queue");
            return FlashCamMMAL::mmal_to_int(MMAL_ENOMEM);
        } else {
            _userdata.opengl_queue = _opengl_queue;
        }
    }
#endif
    
    //set userdata
    new_port->userdata = (struct MMAL_PORT_USERDATA_T *)&_userdata;
    
    // Enable the camera output port with callback
    if ((status = mmal_port_enable(new_port, FlashCam::buffer_callback)) != MMAL_SUCCESS) {
        vcos_log_error("%s: Failed to setup camera output (%u)", __func__, status);
        return FlashCamMMAL::mmal_to_int(status);
    }
    
    // buffer pointer
    MMAL_BUFFER_HEADER_T *buffer;
    // number of buffers
    int qsize = mmal_queue_length(_camera_pool->queue);

    // Send all the buffers to the camera output port
    for (int i=0; i<qsize; i++) {
        //get buffer
        buffer = mmal_queue_get(_camera_pool->queue);
        
        if (!buffer)
            vcos_log_error("%s: Unable to get a required buffer %d from pool queue", __func__, i);
        
        if (mmal_port_send_buffer(new_port, buffer)!= MMAL_SUCCESS)
            vcos_log_error("%s: Unable to send a buffer to camera output port (%d)", __func__, i);
    }
    
    if (_settings.verbose)
        fprintf(stdout, "%s: Success.\n", __func__);
    
    return FlashCamMMAL::mmal_to_int(MMAL_SUCCESS);
}

int FlashCam::getSettingCaptureMode( FLASHCAM_MODE_T *mode ) {
    *mode = _settings.mode;
    return FlashCamMMAL::mmal_to_int(MMAL_SUCCESS);
}

int FlashCam::setSettingSensorMode( unsigned int  sensormode ) {    
    //update settings
    _settings.sensormode = sensormode;

    if (_settings.verbose)
        fprintf(stdout, "%s: Updating sensormode to: %d\n", __func__, sensormode);
    
    //reset camera
    return resetCamera();
}

int FlashCam::getSettingSensorMode( unsigned int *sensormode ) {
    *sensormode = _settings.sensormode;
    return FlashCamMMAL::mmal_to_int(MMAL_SUCCESS);
}

/*** PLL FUNCTIONS ***/

#ifndef BUILD_FLASHCAM_WITH_PLL
// PLL function-implementations are extern: FlashCam_pll.cpp
// These functions are prototypes in case PLL is not build.

int FlashCam::setPLLEnabled( unsigned int  enabled ) {    
    fprintf(stderr, "%s: Cannot set PLL-mode. PLL not build.\n", __func__);
    return FlashCamMMAL::mmal_to_int(MMAL_ENOSYS);
}

int FlashCam::getPLLEnabled( unsigned int *enabled ) {
    fprintf(stderr, "%s: Cannot get PLL-mode. PLL not build.\n", __func__);
    return FlashCamMMAL::mmal_to_int(MMAL_ENOSYS);
}

int FlashCam::setPLLPulseWidth( float  pulsewidth ){    
    fprintf(stderr, "%s: Cannot set PLL-pulsewidth. PLL not build.\n", __func__);
    return FlashCamMMAL::mmal_to_int(MMAL_ENOSYS);
}

int FlashCam::getPLLPulseWidth( float *pulsewidth ) {
    fprintf(stderr, "%s: Cannot get PLL-pulsewidth. PLL not build.\n", __func__);
    return FlashCamMMAL::mmal_to_int(MMAL_ENOSYS);
}

int FlashCam::setPLLDivider( unsigned int  divider ){    
    fprintf(stderr, "%s: Cannot set PLL-divider. PLL not build.\n", __func__);
    return FlashCamMMAL::mmal_to_int(MMAL_ENOSYS);
}

int FlashCam::getPLLDivider( unsigned int *divider ) {
    fprintf(stderr, "%s: Cannot get PLL-divider. PLL not build.\n", __func__);
    return FlashCamMMAL::mmal_to_int(MMAL_ENOSYS);
}

int FlashCam::setPLLOffset( int  offset ){    
    fprintf(stderr, "%s: Cannot set PLL-offset. PLL not build.\n", __func__);
    return FlashCamMMAL::mmal_to_int(MMAL_ENOSYS);
}

int FlashCam::getPLLOffset( int *offset ) {
    fprintf(stderr, "%s: Cannot get PLL-offset. PLL not build.\n", __func__);
    return FlashCamMMAL::mmal_to_int(MMAL_ENOSYS);
}

int FlashCam::setPLLFPSReducerEnabled( unsigned int  enabled ) {    
    fprintf(stderr, "%s: Cannot set PLL-fpsreducer-mode. PLL not build.\n", __func__);
    return FlashCamMMAL::mmal_to_int(MMAL_ENOSYS);
}

int FlashCam::getPLLFPSReducerEnabled( unsigned int *enabled ) {
    fprintf(stderr, "%s: Cannot get PLL-fpsreducer-mode. PLL not build.\n", __func__);
    return FlashCamMMAL::mmal_to_int(MMAL_ENOSYS);
}

#endif //BUILD_FLASHCAM_WITH_PLL     



/* PARAMETER MANAGEMENT */

void FlashCam::getDefaultParams(FLASHCAM_PARAMS_T *params) {
    params->rotation        = 0;
    params->awbmode         = MMAL_PARAM_AWBMODE_AUTO;
    params->flashmode       = MMAL_PARAM_FLASH_OFF;
    params->mirror          = MMAL_PARAM_MIRROR_NONE;
    params->cameranum       = 0;
    params->exposuremode    = MMAL_PARAM_EXPOSUREMODE_AUTO;
    params->metering        = MMAL_PARAM_EXPOSUREMETERINGMODE_AVERAGE;
    params->framerate       = VIDEO_FRAME_RATE_NUM;
    params->stabilisation   = 0;
    params->drc             = MMAL_PARAMETER_DRC_STRENGTH_OFF;
    params->sharpness       = 0;
    params->contrast        = 0;
    params->brightness      = 50;
    params->saturation      = 0;
    params->iso             = 0;
    params->sensormode      = 0;
    params->shutterspeed    = 0;
    params->awbgain_red     = 0;
    params->awbgain_blue    = 0;
    params->denoise         = 1;
}

void FlashCam::printParams(FLASHCAM_PARAMS_T *params) {
    fprintf(stdout, "Rotation     : %d\n", params->rotation);
    fprintf(stdout, "AWB          : %d\n", params->awbmode);
    fprintf(stdout, "Flash        : %d\n", params->flashmode);
    fprintf(stdout, "Mirror       : %d\n", params->mirror);
    fprintf(stdout, "Camera Num   : %d\n", params->cameranum);
    fprintf(stdout, "Exposure     : %d\n", params->exposuremode);
    fprintf(stdout, "Metering     : %d\n", params->metering);
    fprintf(stdout, "Framerate    : %f\n", params->framerate);
    fprintf(stdout, "Stabilisation: %d\n", params->stabilisation);
    fprintf(stdout, "DRC          : %d\n", params->drc);
    fprintf(stdout, "Sharpness    : %d\n", params->sharpness);
    fprintf(stdout, "Contrast     : %d\n", params->contrast);
    fprintf(stdout, "Brightness   : %d\n", params->brightness);
    fprintf(stdout, "Saturation   : %d\n", params->saturation);
    fprintf(stdout, "ISO          : %d\n", params->iso);
    fprintf(stdout, "Sensormode   : %d\n", params->sensormode);
    fprintf(stdout, "Shutterspeed : %d\n", params->shutterspeed);
    fprintf(stdout, "AWB-red      : %d\n", params->awbgain_red);
    fprintf(stdout, "AWB-blue     : %d\n", params->awbgain_blue);
    fprintf(stdout, "Denoise      : %d\n", params->denoise);
    
}

int FlashCam::setParams(FLASHCAM_PARAMS_T *params) {
    int status = 0;
    
    //set values
    if (_settings.verbose) fprintf(stdout, "%s: Rotation      :%d\n", __func__, params->rotation);        
    status += setRotation(params->rotation);        
    if (_settings.verbose) fprintf(stdout, "%s: AWB-Mode      :%d\n", __func__, params->awbmode);
    status += setAWBMode(params->awbmode);
    if (_settings.verbose) fprintf(stdout, "%s: Flash-Mode    :%d\n", __func__, params->flashmode);
    status += setFlashMode(params->flashmode);
    if (_settings.verbose) fprintf(stdout, "%s: Mirror        :%d\n", __func__, params->mirror);
    status += setMirror(params->mirror);
    if (_settings.verbose) fprintf(stdout, "%s: CameraNum     :%d - ignored. Set when initialising camera\n", __func__, params->cameranum);
    //status += setCameraNum(params->cameranum);
    if (_settings.verbose) fprintf(stdout, "%s: Exposure-Mode :%d\n", __func__, params->exposuremode);
    status += setExposureMode(params->exposuremode);
    if (_settings.verbose) fprintf(stdout, "%s: Metering      :%d\n", __func__, params->metering);
    status += setMeteringMode(params->metering);
    if (_settings.verbose) fprintf(stdout, "%s: Framerate     :%f\n", __func__, params->framerate);
    status += setFrameRate(params->framerate);
    if (_settings.verbose) fprintf(stdout, "%s: Stabilisation :%d\n", __func__, params->stabilisation);
    status += setStabilisation(params->stabilisation);
    if (_settings.verbose) fprintf(stdout, "%s: DRC           :%d\n", __func__, params->drc);
    status += setDRC(params->drc);
    if (_settings.verbose) fprintf(stdout, "%s: Sharpness     :%d\n", __func__, params->sharpness);
    status += setSharpness(params->sharpness);
    if (_settings.verbose) fprintf(stdout, "%s: Contrast      :%d\n", __func__, params->contrast);
    status += setContrast(params->contrast);
    if (_settings.verbose) fprintf(stdout, "%s: Brightness    :%d\n", __func__, params->brightness);
    status += setBrightness(params->brightness);
    if (_settings.verbose) fprintf(stdout, "%s: Saturation    :%d\n", __func__, params->saturation);
    status += setSaturation(params->saturation);
    if (_settings.verbose) fprintf(stdout, "%s: ISO           :%d\n", __func__, params->iso);
    status += setISO(params->iso);
    if (_settings.verbose) fprintf(stdout, "%s: Sensormode    :%d - ignored. Set when initialising camera\n", __func__, params->sensormode);
    //status += setSensorMode(params->sensormode);
    if (_settings.verbose) fprintf(stdout, "%s: Shutterspeed  :%d\n", __func__, params->shutterspeed);
    status += setShutterSpeed(params->shutterspeed);
    if (_settings.verbose) fprintf(stdout, "%s: AWB-Gains     :%f/%f\n", __func__, params->awbgain_red, params->awbgain_blue);
    status += setAWBGains(params->awbgain_red, params->awbgain_blue);
    if (_settings.verbose) fprintf(stdout, "%s: Denoise       :%d\n", __func__, params->denoise);
    status += setDenoise(params->denoise);
        
    return status;
}

int FlashCam::getParams(FLASHCAM_PARAMS_T *params, bool mem) {
    int status = 0;
    
    if (mem) {
        if (_settings.verbose) fprintf(stdout, "%s: Copying params from memory\n", __func__ );
        memcpy(params, &_params, sizeof(FLASHCAM_PARAMS_T));
        
    } else {
        if (_settings.verbose) fprintf(stdout, "%s: Rotation\n", __func__ );
        status += getRotation( &(params->rotation) );
        if (_settings.verbose) fprintf(stdout, "%s: AWB-mode\n", __func__ );
        status += getAWBMode( &(params->awbmode) );
        if (_settings.verbose) fprintf(stdout, "%s: Flash-mode\n", __func__ );
        status += getFlashMode( &(params->flashmode) );
        if (_settings.verbose) fprintf(stdout, "%s: Mirror\n", __func__ );
        status += getMirror( &(params->mirror) );
        if (_settings.verbose) fprintf(stdout, "%s: CameraNum\n", __func__ );
        status += getCameraNum( &(params->cameranum) );
        if (_settings.verbose) fprintf(stdout, "%s: Exposure-mode\n", __func__ );
        status += getExposureMode( &(params->exposuremode) );
        if (_settings.verbose) fprintf(stdout, "%s: Metering\n", __func__ );
        status += getMeteringMode( &(params->metering) );
        if (_settings.verbose) fprintf(stdout, "%s: Framerate\n", __func__ );
        status += getFrameRate( &(params->framerate) );
        if (_settings.verbose) fprintf(stdout, "%s: Stabilisation\n", __func__ );
        status += getStabilisation( &(params->stabilisation) );
        if (_settings.verbose) fprintf(stdout, "%s: DRC\n", __func__ );
        status += getDRC( &(params->drc) );
        if (_settings.verbose) fprintf(stdout, "%s: Sharpness\n", __func__ );
        status += getSharpness( &(params->sharpness) );
        if (_settings.verbose) fprintf(stdout, "%s: Contrast\n", __func__ );
        status += getContrast( &(params->contrast) );
        if (_settings.verbose) fprintf(stdout, "%s: Brightness\n", __func__ );
        status += getBrightness( &(params->brightness) );
        if (_settings.verbose) fprintf(stdout, "%s: Saturation\n", __func__ );
        status += getSaturation( &(params->saturation) );
        if (_settings.verbose) fprintf(stdout, "%s: ISO\n", __func__ );
        status += getISO( &(params->iso) );
        if (_settings.verbose) fprintf(stdout, "%s: SensorMode\n", __func__ );
        status += getSensorMode( &(params->sensormode) );
        if (_settings.verbose) fprintf(stdout, "%s: Shutterspeed\n", __func__ );
        status += getShutterSpeed( &(params->shutterspeed) );
        if (_settings.verbose) fprintf(stdout, "%s: getAWBGains\n", __func__ );
        status += getAWBGains( &(params->awbgain_red),  &(params->awbgain_blue) );
        if (_settings.verbose) fprintf(stdout, "%s: Denoise\n", __func__ );
        status += getDenoise( &(params->denoise) );
    }
    return status;
}

/* GENERAL (PRIVATE) SETTER/GETTER */

MMAL_STATUS_T FlashCam::setParameterRational( int id, int val ) {
    MMAL_RATIONAL_T rational = {val, 100};
    return mmal_port_parameter_set_rational(_camera_component->control, id, rational );
}

MMAL_STATUS_T FlashCam::getParameterRational( int id, int *val ) {
    MMAL_RATIONAL_T rational;        
    MMAL_STATUS_T   status = mmal_port_parameter_get_rational(_camera_component->control, id, &rational);
    *val                   = rational.num;
    return status;        
}

/* PUBLIC SETTER/GETTER */

int FlashCam::setRotation ( int rotation ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    rotation = ((rotation % 360 ) / 90) * 90;
    //set param
    MMAL_STATUS_T status;    
    if ((status = mmal_port_parameter_set_int32(_camera_component->output[MMAL_CAMERA_PREVIEW_PORT], MMAL_PARAMETER_ROTATION, rotation)) != MMAL_SUCCESS)
        return FlashCamMMAL::mmal_to_int(status);
    if ((status = mmal_port_parameter_set_int32(_camera_component->output[MMAL_CAMERA_VIDEO_PORT  ], MMAL_PARAMETER_ROTATION, rotation)) != MMAL_SUCCESS)
        return FlashCamMMAL::mmal_to_int(status);
    if ((status = mmal_port_parameter_set_int32(_camera_component->output[MMAL_CAMERA_CAPTURE_PORT], MMAL_PARAMETER_ROTATION, rotation)) != MMAL_SUCCESS)
        return FlashCamMMAL::mmal_to_int(status);
    //success
    _params.rotation = rotation;
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::getRotation ( int *rotation ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    //just pick a port: all ports should have equal settings.
    MMAL_STATUS_T status = mmal_port_parameter_get_int32(_camera_component->output[MMAL_CAMERA_CAPTURE_PORT], MMAL_PARAMETER_ROTATION, rotation);        
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::setAWBMode ( MMAL_PARAM_AWBMODE_T awb ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    MMAL_PARAMETER_AWBMODE_T param  = {{MMAL_PARAMETER_AWB_MODE, sizeof(param)}, awb};
    MMAL_STATUS_T            status = mmal_port_parameter_set(_camera_component->control, &param.hdr);   
    if ( status == MMAL_SUCCESS ) _params.awbmode = awb;
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::getAWBMode ( MMAL_PARAM_AWBMODE_T *awb ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    // MMAL_PARAM_AWBMODE_MAX: just usa a value to allocate `param`
    MMAL_PARAMETER_AWBMODE_T param  = {{MMAL_PARAMETER_AWB_MODE, sizeof(param)}, MMAL_PARAM_AWBMODE_MAX};
    MMAL_STATUS_T            status = mmal_port_parameter_get(_camera_component->control, &param.hdr);
    //update value
    *awb = param.value;
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::setFlashMode ( MMAL_PARAM_FLASH_T flash ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    MMAL_PARAMETER_FLASH_T param  = {{MMAL_PARAMETER_FLASH, sizeof(param)}, flash};
    MMAL_STATUS_T          status = mmal_port_parameter_set(_camera_component->control, &param.hdr);
    if ( status == MMAL_SUCCESS ) _params.flashmode = flash;
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::getFlashMode ( MMAL_PARAM_FLASH_T *flash ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    // MMAL_PARAM_FLASH_MAX: just usa a value to allocate `param`
    MMAL_PARAMETER_FLASH_T param  = {{MMAL_PARAMETER_FLASH, sizeof(param)}, MMAL_PARAM_FLASH_MAX};
    MMAL_STATUS_T          status = mmal_port_parameter_get(_camera_component->control, &param.hdr);
    //update value
    *flash = param.value;
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::setMirror ( MMAL_PARAM_MIRROR_T mirror ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    MMAL_PARAMETER_MIRROR_T param  = {{MMAL_PARAMETER_MIRROR, sizeof(param)}, mirror};
    //set param
    MMAL_STATUS_T status;
    if ((status = mmal_port_parameter_set(_camera_component->output[MMAL_CAMERA_PREVIEW_PORT], &param.hdr)) != MMAL_SUCCESS)
        return FlashCamMMAL::mmal_to_int(status);
    if ((status = mmal_port_parameter_set(_camera_component->output[MMAL_CAMERA_VIDEO_PORT  ], &param.hdr)) != MMAL_SUCCESS)
        return FlashCamMMAL::mmal_to_int(status);
    if ((status = mmal_port_parameter_set(_camera_component->output[MMAL_CAMERA_CAPTURE_PORT], &param.hdr)) != MMAL_SUCCESS)
        return FlashCamMMAL::mmal_to_int(status);
    //success
    _params.mirror = mirror;
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::getMirror ( MMAL_PARAM_MIRROR_T *mirror ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    // MMAL_PARAM_MIRROR_NONE: just usa a value to allocate `param`
    MMAL_PARAMETER_MIRROR_T param  = {{MMAL_PARAMETER_MIRROR, sizeof(param)}, MMAL_PARAM_MIRROR_NONE};
    //just pick a port: all ports should have equal settings.
    MMAL_STATUS_T           status = mmal_port_parameter_get(_camera_component->output[MMAL_CAMERA_CAPTURE_PORT], &param.hdr);       
    //update value
    *mirror = param.value;
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::setCameraNum ( unsigned int num ) {
    //cameranum should be set when camera is not (yet) initialised
    if (( !_camera_component ) || ( _initialised )) return 1;
    MMAL_PARAMETER_UINT32_T param  = {{MMAL_PARAMETER_CAMERA_NUM, sizeof(param)}, num};
    MMAL_STATUS_T           status = mmal_port_parameter_set(_camera_component->control, &param.hdr);
    if ( status == MMAL_SUCCESS ) _params.cameranum = num;    
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::getCameraNum ( unsigned int *num ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    // MMAL_PARAM_MIRROR_NONE: just usa a value to allocate `param`
    MMAL_PARAMETER_UINT32_T param  = {{MMAL_PARAMETER_CAMERA_NUM, sizeof(param)}, 0};
    MMAL_STATUS_T           status = mmal_port_parameter_get(_camera_component->control, &param.hdr);       
    //update value
    *num = param.value;
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::setCapture ( MMAL_PORT_T *port, int capture ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_set_boolean(port, MMAL_PARAMETER_CAPTURE, capture);
    //value is only needed when taking a snapshot, so it is not tracked in _params
    //if ( status == MMAL_SUCCESS ) _params.capture = capture;
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::getCapture ( MMAL_PORT_T *port, int *capture ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_get_boolean(port, MMAL_PARAMETER_CAPTURE, capture);   
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::setExposureMode ( MMAL_PARAM_EXPOSUREMODE_T exposure ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    MMAL_PARAMETER_EXPOSUREMODE_T param  = {{MMAL_PARAMETER_EXPOSURE_MODE, sizeof(param)}, exposure};
    MMAL_STATUS_T                 status = mmal_port_parameter_set(_camera_component->control, &param.hdr);
    if ( status == MMAL_SUCCESS ) _params.exposuremode = exposure;
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::getExposureMode ( MMAL_PARAM_EXPOSUREMODE_T *exposure ) {
    // MMAL_PARAM_EXPOSUREMODE_MAX: just usa a value to allocate `param`
    if (( !_camera_component ) || ( !_initialised )) return 1;
    MMAL_PARAMETER_EXPOSUREMODE_T param  = {{MMAL_PARAMETER_EXPOSURE_MODE, sizeof(param)}, MMAL_PARAM_EXPOSUREMODE_MAX};
    MMAL_STATUS_T                 status = mmal_port_parameter_get(_camera_component->control, &param.hdr);       
    //update value
    *exposure = param.value;
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::setMeteringMode ( MMAL_PARAM_EXPOSUREMETERINGMODE_T  metering ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    MMAL_PARAMETER_EXPOSUREMETERINGMODE_T param  = {{MMAL_PARAMETER_EXP_METERING_MODE, sizeof(param)}, metering};        
    MMAL_STATUS_T                         status = mmal_port_parameter_set(_camera_component->control, &param.hdr);
    if ( status == MMAL_SUCCESS ) _params.metering = metering;
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::getMeteringMode ( MMAL_PARAM_EXPOSUREMETERINGMODE_T *metering ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    // MMAL_PARAM_EXPOSUREMETERINGMODE_MAX: just usa a value to allocate `param`
    MMAL_PARAMETER_EXPOSUREMETERINGMODE_T param  = {{MMAL_PARAMETER_EXP_METERING_MODE, sizeof(param)}, MMAL_PARAM_EXPOSUREMETERINGMODE_MAX};
    MMAL_STATUS_T                         status = mmal_port_parameter_get(_camera_component->control, &param.hdr);       
    //update value
    *metering = param.value;
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::setCameraConfig ( MMAL_PARAMETER_CAMERA_CONFIG_T *config ) {
    //camera configuration should only be updated when camera is not (yet) initialised
    if (( !_camera_component ) || ( _initialised )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_set(_camera_component->control, &config->hdr);
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::getCameraConfig ( MMAL_PARAMETER_CAMERA_CONFIG_T *config ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_get(_camera_component->control, &config->hdr);       
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::setFrameRate ( float  fps ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    //limit fps
    if ( fps <   0 ) fps =   0;
    if ( fps > 120 ) fps = 120;
    //create rationale
    MMAL_RATIONAL_T f;        
    f.den = 256;
    f.num = (unsigned int) (fps * f.den);
    //setup param
    MMAL_STATUS_T status;
    MMAL_PARAMETER_FRAME_RATE_T param = {{MMAL_PARAMETER_VIDEO_FRAME_RATE, sizeof(param)}, f};
    if ((status = mmal_port_parameter_set(_camera_component->output[MMAL_CAMERA_PREVIEW_PORT], &param.hdr)) != MMAL_SUCCESS)
        return FlashCamMMAL::mmal_to_int(status);
    if ((status = mmal_port_parameter_set(_camera_component->output[MMAL_CAMERA_VIDEO_PORT  ], &param.hdr)) != MMAL_SUCCESS)
        return FlashCamMMAL::mmal_to_int(status);
    // Do not set capture port: does not have a fps-option
    //if ((status = mmal_port_parameter_set(_camera_component->output[MMAL_CAMERA_CAPTURE_PORT], &param.hdr)) != MMAL_SUCCESS)
    //    return FlashCamMMAL::mmal_to_int(status);
    //success
    _params.framerate = fps;
    return FlashCamMMAL::mmal_to_int(status);
    
}

int FlashCam::getFrameRate ( float *fps ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    // {0,0}: just usa a value to allocate `param`
    MMAL_PARAMETER_FRAME_RATE_T param  = {{MMAL_PARAMETER_VIDEO_FRAME_RATE, sizeof(param)}, {0,0}};
    //just pick a port: all ports should have equal settings.
    MMAL_STATUS_T               status = mmal_port_parameter_get(_camera_component->output[MMAL_CAMERA_VIDEO_PORT], &param.hdr);       
    //update value
    *fps  = ((float)param.frame_rate.num) / ((float)param.frame_rate.den) ;
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::setStabilisation ( int stabilisation ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_set_boolean(_camera_component->control, MMAL_PARAMETER_VIDEO_STABILISATION, stabilisation);   
    if ( status == MMAL_SUCCESS ) _params.stabilisation = stabilisation;
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::getStabilisation ( int *stabilisation ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_get_boolean(_camera_component->control, MMAL_PARAMETER_VIDEO_STABILISATION, stabilisation);        
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::setDRC ( MMAL_PARAMETER_DRC_STRENGTH_T  strength ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    MMAL_PARAMETER_DRC_T param  = {{MMAL_PARAMETER_DYNAMIC_RANGE_COMPRESSION, sizeof(param)}, strength};
    MMAL_STATUS_T        status = mmal_port_parameter_set(_camera_component->control, &param.hdr);
    if ( status == MMAL_SUCCESS ) _params.drc = strength;
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::getDRC ( MMAL_PARAMETER_DRC_STRENGTH_T *strength ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    // MMAL_PARAMETER_DRC_STRENGTH_MAX: just usa a value to allocate `param`
    MMAL_PARAMETER_DRC_T param  = {{MMAL_PARAMETER_DYNAMIC_RANGE_COMPRESSION, sizeof(param)}, MMAL_PARAMETER_DRC_STRENGTH_MAX};
    MMAL_STATUS_T        status = mmal_port_parameter_get(_camera_component->control, &param.hdr);  
    //update value
    *strength = param.strength;
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::setSharpness ( int  sharpness ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    if ( sharpness < -100 ) sharpness = -100;
    if ( sharpness >  100 ) sharpness =  100;
    MMAL_STATUS_T status = setParameterRational(MMAL_PARAMETER_SHARPNESS, sharpness);
    if ( status == MMAL_SUCCESS ) _params.sharpness = sharpness;
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::getSharpness ( int *sharpness ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    return getParameterRational( MMAL_PARAMETER_SHARPNESS, sharpness);        
}

int FlashCam::setContrast ( int  contrast ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    if ( contrast < -100 ) contrast = -100;
    if ( contrast >  100 ) contrast =  100;
    MMAL_STATUS_T status = setParameterRational(MMAL_PARAMETER_CONTRAST, contrast);
    if ( status == MMAL_SUCCESS ) _params.contrast = contrast;
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::getContrast ( int *contrast ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    return getParameterRational( MMAL_PARAMETER_CONTRAST, contrast);        
}

int FlashCam::setBrightness ( int  brightness ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    if ( brightness <    0 ) brightness =    0;
    if ( brightness >  100 ) brightness =  100;
    MMAL_STATUS_T status = setParameterRational(MMAL_PARAMETER_BRIGHTNESS, brightness);
    if ( status == MMAL_SUCCESS ) _params.brightness = brightness;
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::getBrightness ( int *brightness ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    return getParameterRational( MMAL_PARAMETER_BRIGHTNESS, brightness);        
}

int FlashCam::setSaturation ( int  saturation ) {        
    if (( !_camera_component ) || ( !_initialised )) return 1;
    if ( saturation < -100 ) saturation = -100;
    if ( saturation >  100 ) saturation =  100;
    MMAL_STATUS_T status = setParameterRational(MMAL_PARAMETER_SATURATION, saturation);
    if ( status == MMAL_SUCCESS ) _params.saturation = saturation;
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::getSaturation ( int *saturation ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    return getParameterRational( MMAL_PARAMETER_SATURATION, saturation);        
}

int FlashCam::setISO ( unsigned int  iso ) {        
    if (( !_camera_component ) || ( !_initialised )) return 1;
    if ( iso > 1600 ) iso = 1600;
    MMAL_STATUS_T status = mmal_port_parameter_set_uint32(_camera_component->control, MMAL_PARAMETER_ISO, iso); 
    if ( status == MMAL_SUCCESS ) _params.iso = iso;
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::getISO ( unsigned int *iso ) {        
    if (( !_camera_component ) || ( !_initialised )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_get_uint32(_camera_component->control, MMAL_PARAMETER_ISO, iso);        
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::setSensorMode ( unsigned int  mode ) {
    //sensormode should be set while not (yet) initialised
    if (( !_camera_component ) || ( _initialised )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_set_uint32(_camera_component->control, MMAL_PARAMETER_CAMERA_CUSTOM_SENSOR_CONFIG, mode); 
    if ( status == MMAL_SUCCESS ) _params.sensormode = mode;
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::getSensorMode ( unsigned int *mode ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_get_uint32(_camera_component->control, MMAL_PARAMETER_CAMERA_CUSTOM_SENSOR_CONFIG, mode);        
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::setShutterSpeed ( unsigned int  speed ) {        
    if (( !_camera_component ) || ( !_initialised )) return 1;
    if ( speed > 330000 ) speed = 330000;
    MMAL_STATUS_T status = mmal_port_parameter_set_uint32(_camera_component->control, MMAL_PARAMETER_SHUTTER_SPEED, speed);  
    if ( status == MMAL_SUCCESS ) _params.shutterspeed = speed;
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::getShutterSpeed ( unsigned int *speed ) {        
    if (( !_camera_component ) || ( !_initialised )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_get_uint32(_camera_component->control, MMAL_PARAMETER_SHUTTER_SPEED, speed);        
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::setAWBGains ( float red , float blue ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    if (red  < 0.0) red  = 0.0;
    if (red  > 8.0) red  = 8.0;
    if (blue < 0.0) blue = 0.0;
    if (blue > 8.0) blue = 8.0;
    //recompute gains
    MMAL_RATIONAL_T r, b;        
    r.num = (unsigned int) (red  * 65536);
    b.num = (unsigned int) (blue * 65536);
    r.den = b.den = 65536;
    //setup param
    MMAL_PARAMETER_AWB_GAINS_T param  = {{MMAL_PARAMETER_CUSTOM_AWB_GAINS,sizeof(param)}, r, b};
    MMAL_STATUS_T              status = mmal_port_parameter_set(_camera_component->control, &param.hdr);  
    if ( status == MMAL_SUCCESS ) {
        _params.awbgain_red  = red;   
        _params.awbgain_blue = blue;   
    }
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::getAWBGains ( float *red , float *blue ) { 
    if (( !_camera_component ) || ( !_initialised )) return 1;
    //recompute gains
    MMAL_RATIONAL_T r, b;        
    r.num = b.num = 0;
    r.den = b.den = 65536;
    // {0,0}, {0,0}: just usa a value to allocate `param`
    MMAL_PARAMETER_AWB_GAINS_T param  = {{MMAL_PARAMETER_CUSTOM_AWB_GAINS,sizeof(param)}, r, b};
    MMAL_STATUS_T              status = mmal_port_parameter_set(_camera_component->control, &param.hdr);  
    //update value
    *red  = ((float)param.r_gain.num) / ((float)param.r_gain.den) ;
    *blue = ((float)param.b_gain.num) / ((float)param.b_gain.den) ;
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::setDenoise ( int  denoise ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_set_boolean(_camera_component->control, MMAL_PARAMETER_STILLS_DENOISE, denoise);     
    if ( status == MMAL_SUCCESS ) _params.denoise = denoise;
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::getDenoise ( int *denoise ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_get_boolean(_camera_component->control, MMAL_PARAMETER_STILLS_DENOISE, denoise);        
    return FlashCamMMAL::mmal_to_int(status);
}


int FlashCam::setChangeEventRequest ( unsigned int id , int  request ) {
    // allow request even not (yet) initialised
    if ( !_camera_component ) return 1;
    MMAL_PARAMETER_CHANGE_EVENT_REQUEST_T param  = {{MMAL_PARAMETER_CHANGE_EVENT_REQUEST, sizeof(param)}, id, request};
    MMAL_STATUS_T                         status = mmal_port_parameter_set(_camera_component->control, &param.hdr);
    return FlashCamMMAL::mmal_to_int(status);
}

int FlashCam::getChangeEventRequest ( unsigned int id , int *request ) {
    if (( !_camera_component ) || ( !_initialised )) return 1;
    // 0: just usa a value to allocate `param`
    MMAL_PARAMETER_CHANGE_EVENT_REQUEST_T param  = {{MMAL_PARAMETER_CHANGE_EVENT_REQUEST, sizeof(param)}, id, 0};
    MMAL_STATUS_T                         status = mmal_port_parameter_get(_camera_component->control, &param.hdr);  
    //update value
    *request = param.enable;
    return FlashCamMMAL::mmal_to_int(status);
}









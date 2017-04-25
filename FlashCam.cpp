#include "FlashCam.h"

extern "C" {
#include "types.h"
}

#include "bcm_host.h"


#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include <sysexits.h>


#define FLASHCAM_VERSION_STRING "v0.1"
#define DEBUG 1

// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT   1
#define MMAL_CAMERA_CAPTURE_PORT 2

// Stills format information
// 0 implies variable
#define PREVIEW_FRAME_RATE_NUM 0
#define PREVIEW_FRAME_RATE_DEN 1
#define   VIDEO_FRAME_RATE_NUM 30
#define   VIDEO_FRAME_RATE_DEN 1
#define CAPTURE_FRAME_RATE_NUM 0
#define CAPTURE_FRAME_RATE_DEN 1

/*
 * Constructor
 */
FlashCam::FlashCam(){    
    //basic init. 
    _initialised = false;
    _active      = false;
    
    //init with default parameter set;
    getDefaultSettings(&_settings);
    setSettings(&_settings); //implicitly resets/configures camera
}

FlashCam::FlashCam(FLASHCAM_SETTINGS_T *settings){
    //basic init. 
    _initialised = false;
    _active      = false;
    
    //init with given parameter set;
    setSettings(settings); //implicitly resets/configures camera
}

/*
 * Destructor
 */
FlashCam::~FlashCam(){
    destroyComponents();
    // delete semaphore
    vcos_semaphore_delete(&_userdata.sem_capture);
}



int FlashCam::resetCamera() {
    if (_active) {
        fprintf(stderr, "%s: Cannot reset camera while it is capturing.\n", __func__);
        return mmal_status_to_int(MMAL_EINVAL);
    }
    
    if (_settings.verbose)
        fprintf(stdout, "%s: Resetting/initializing components.\n", __func__);
    
    // setup / reset camera (return upon error)
    if (status = setupComponents( &_settings ))
        return status; 
    
    if (_settings.verbose)
        fprintf(stdout, "%s: Resetting/initializing mode.\n", __func__);
    
    // enable / reset selected mode
    if (status = setSettingMode( _settings.mode ))
        return status;
    
    if (_settings.verbose)
        fprintf(stdout, "%s: Succes.\n", __func__);
    
    //succes
    return mmal_status_to_int(MMAL_SUCCESS);
    
}


// setup the camera with specified settings
int FlashCam::setupComponents() {
    
    // Is camera active?
    if (_active) {
        fprintf(stderr, "%s: Cannot reset camera while it is capturing.\n", __func__);
        return mmal_status_to_int(MMAL_EINVAL);
    }
    
    // Only update these settings when initialising for the first time
    if (!_initialised) {
        bcm_host_init();
        
        //register to VCOS for logging
        vcos_log_register("FlashCam", VCOS_LOG_CATEGORY);
        
        // create & init semaphore
        if (vcos_semaphore_create(&_userdata.sem_capture, "FlashCam-sem", 0) != VCOS_SUCCESS) {
            vcos_log_error("%s: Failed to create semaphore", __func__);
            return mmal_status_to_int(MMAL_EINVAL);
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
    
    //set userdata
    _userdata.params            = &_params;
    _userdata.settings          = &_settings;
    _userdata.camera_pool       = NULL;
    _userdata.framebuffer       = NULL;
    _userdata.framebuffer_size  = 0;
    _userdata.framebuffer_idx   = 0;
    _userdata.callback          = NULL;
    
    //Status flags
    MMAL_STATUS_T status        = MMAL_SUCCESS;
        
    //print that we are starting
    if (_settings.verbose)
        fprintf(stdout, "\n FlashCam Version: %s\n\n", FLASHCAM_VERSION_STRING);
        
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
        return mmal_status_to_int(status);
    }
    
    //setup preview (nullsink!)
    // -> But we need it as it is used for measurements such as awb
    // internally sets:
    //      _preview_component
    if ((status = setupComponentPreview()) != MMAL_SUCCESS) {
        vcos_log_error("%s: Failed to create preview component", __func__);
        destroyComponents();
        return mmal_status_to_int(status);
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
        return mmal_status_to_int(status);
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
        vcos_log_error("%s: Could not select camera", __func__);
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
        .use_stc_timestamp                      = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
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
    format->encoding                    = MMAL_ENCODING_I420;
    format->encoding_variant            = MMAL_ENCODING_I420;     
    format->es->video.width             = _settings.width;
    format->es->video.height            = _settings.height;
    format->es->video.crop.x            = 0;
    format->es->video.crop.y            = 0;
    format->es->video.crop.width        = _settings.width;
    format->es->video.crop.height       = _settings.height;
    format->es->video.frame_rate.num    = VIDEO_FRAME_RATE_NUM;
    format->es->video.frame_rate.den    = VIDEO_FRAME_RATE_DEN;
    
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
                vcos_log_error("%s: AWB R=%u/%u, B=%u/%u", __func__
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
    int abort    = 0; //flag for detecting if we need to abort due to error
    int complete = 0; //flag for detecting if a full frame is recieved
    int max_idx  = 0; //flag for detecting if _framebuffer is out of memory
    
    //lock buffer --> callback is async!
    mmal_buffer_header_mem_lock(buffer);
    
    //retrieve userdata
    FLASHCAM_PORT_USERDATA_T *userdata = (FLASHCAM_PORT_USERDATA_T *)port->userdata;
    
    //is userdata properly set?
    if (userdata) {
        
        // Are there bytes to write?
        if (buffer->length) {
            
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
                                                
            if (userdata->settings->verbose) {
                fprintf(stdout, "%s, Copying %d bytes @ %d (%d x %d)\n", 
                        __func__, buffer->length, userdata->framebuffer_idx, 
                        userdata->settings->height, userdata->settings->width);
                
                /*
                fprintf(stderr, "Y     : %d @ %d\n", length_Y, offset_Y);   
                fprintf(stderr, "U     : %d @ %d\n", length_U, offset_U);   
                fprintf(stderr, "V     : %d @ %d\n", length_V, offset_V);   
                fprintf(stderr, "Total : %d (%d)\n", length_Y + length_U + length_V, buffer->length);   
                 */
                
                /*
                fprintf(stderr, "Buffervalues: \n");
                fprintf(stderr, "- next      : %p\n", buffer->next);
                fprintf(stderr, "- cmd       : %d\n", buffer->cmd);
                fprintf(stderr, "- alloc_size: %d\n", buffer->alloc_size);
                fprintf(stderr, "- length    : %d\n", buffer->length);
                fprintf(stderr, "- offset    : %d\n", buffer->offset);
                fprintf(stderr, "- flags     : %d\n", buffer->flags);
                fprintf(stderr, "- pts       : %d\n", buffer->pts);
                fprintf(stderr, "- dts       : %d\n", buffer->dts);
                fprintf(stderr, "- type      : %d\n", buffer->type);   
                 */
            }
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
        }
        
        // Check end of frame or error
        if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_TRANSMISSION_FAILED)    
            abort = 1;
        
        if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END)              
            complete = 1;
        
    } else {
        vcos_log_error("%s: Received a camera still buffer callback with no state", __func__);
    }
    
    // release buffer back to the pool
    mmal_buffer_header_mem_unlock(buffer);
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



// Setup connection between Input/Output ports
MMAL_STATUS_T FlashCam::connect_ports(MMAL_PORT_T *output_port, MMAL_PORT_T *input_port, MMAL_CONNECTION_T **connection) {    
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
        return mmal_status_to_int(MMAL_EINVAL);
    }
    
    // Is camera active?
    if (_active) {
        fprintf(stderr, "%s: Camera already capturing.\n", __func__);
        return mmal_status_to_int(MMAL_EINVAL);
    }

    if (_settings.verbose)
        fprintf(stdout, "%s: Starting capture\n", __func__);

    if (_settings.mode == FLASHCAM_MODE_VIDEO) {
        
        if (status = setCapture(_camera_component->output[MMAL_CAMERA_VIDEO_PORT], 1)) {
            vcos_log_error("%s: Failed to start video stream", __func__);
            return status;
        }
        
        //starting stream
        _active = true;
        
    } else if ( _settings.mode = FLASHCAM_MODE_CAPTURE ) { 
        
        if (status = setCapture(_camera_component->output[MMAL_CAMERA_CAPTURE_PORT], 1)) {
            vcos_log_error("%s: Failed to start capture", __func__);
            return status;
        }
        
        // Wait for capture to complete
        // For some reason using vcos_semaphore_wait_timeout sometimes returns immediately with bad parameter error
        // even though it appears to be all correct, so reverting to untimed one until figure out why its erratic
        _active = true;
        vcos_semaphore_wait(&_userdata.sem_capture);
        _active = false;
    
    } else {
        //unknown mode..
        fprintf(stderr, "%s: Cannot start camera. Unknown mode (%u)\n", __func__, _settings.mode);
        return mmal_status_to_int(MMAL_EINVAL);
    }
    
    if (_settings.verbose)
        fprintf(stdout, "%s: Success\n", __func__);

    return mmal_status_to_int(MMAL_SUCCESS);
}


int stopCapture() {
    int status;
    
    if (!_initialised) {
        fprintf(stderr, "%s: Camera not initialised.\n", __func__);
        return mmal_status_to_int(MMAL_EINVAL);
    }
        
    // Is camera active?
    if (!_active) {
        fprintf(stderr, "%s: Camera not capturing.\n", __func__);
        return mmal_status_to_int(MMAL_EINVAL);
    }
    
    if (_settings.verbose)
        fprintf(stdout, "%s: Stopping stream.\n", __func__);
    
    if (_settings.mode == FLASHCAM_MODE_VIDEO) {
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
        return mmal_status_to_int(MMAL_EINVAL);
    }

    //camera inactive
    _active = false;
    
    if (_settings.verbose)
        fprintf(stdout, "%s: Success\n", __func__);
    
    return mmal_status_to_int(MMAL_SUCCESS);
}



void FlashCam::setFrameCallback(FLASHCAM_CALLBACK_T callback) {
    if (_active) return; //no changer/reset while in capturemode
    _userdata.callback = callback;
}

void FlashCam::resetFrameCallback() {
    if (_active) return; //no changer/reset while in capturemode
    _userdata.callback = NULL;
}



/* SETTING MANAGEMENT */

void FlashCam::getDefaultSettings(FLASHCAM_SETTINGS_T *settings) {
    settings->width     = 640;
    settings->height    = 480;
    settings->verbose   = 1;
    settings->update    = 0;
    settings->mode      = FLASHCAM_MODE_CAPTURE;
}

void FlashCam::printSettings(FLASHCAM_SETTINGS_T *settings) {
    fprintf(stderr, "Width        : %d\n", settings->width);
    fprintf(stderr, "Height       : %d\n", settings->height);
    fprintf(stderr, "Verbose      : %d\n", settings->verbose);
    fprintf(stderr, "Update       : %d\n", settings->update);
    fprintf(stderr, "Camera-Mode  : %d\n", settings->mode);
}

int FlashCam::setSettings(FLASHCAM_SETTINGS_T *settings) {

    //update settings
    memcpy(&_settings, settings, sizeof(FLASHCAM_SETTINGS_T));

    if (_settings.verbose)
        fprintf(stdout, "%s: Resetting camera.\n", __func__);

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
    return mmal_status_to_int(MMAL_SUCCESS);    
}

int FlashCam::setSettingVerbose( int  verbose ) {
    _settings.verbose = verbose;
    return mmal_status_to_int(MMAL_SUCCESS);    
}

int FlashCam::getSettingVerbose( int *verbose ) {
    *verbose = _settings.verbose;
    return mmal_status_to_int(MMAL_SUCCESS);    
}

int FlashCam::setSettingUpdate( int  update ) {
    _settings.update = update;
    return setChangeEventRequest(MMAL_PARAMETER_CAMERA_SETTINGS, _settings.update); 
}

int FlashCam::getSettingUpdate( int *update ) {
    *update = _settings.update;
    return mmal_status_to_int(MMAL_SUCCESS);

}

int FlashCam::setSettingMode() {
    // Is camera active?
    if (_active) {
        fprintf(stderr, "%s: Cannot change camera mode while being in use\n", __func__);
        return mmal_status_to_int(MMAL_EINVAL);
    }
    
    // Is camera initialised?
    if (!_initialised) {
        fprintf(stderr, "%s: Components not initialised\n", __func__);
        return mmal_status_to_int(MMAL_EINVAL);
    }
    
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
        return mmal_status_to_int(MMAL_EINVAL);
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
    
    //set userdata
    new_port->userdata = (struct MMAL_PORT_USERDATA_T *)&_userdata;
    
    // Enable the camera output port with callback
    if ((status = mmal_port_enable(new_port, FlashCam::buffer_callback)) != MMAL_SUCCESS) {
        vcos_log_error("%s: Failed to setup camera output (%u)", __func__, status);
        return mmal_status_to_int(status);
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
    
    return mmal_status_to_int(MMAL_SUCCESS);
}

int FlashCam::getSettingMode( FLASHCAM_MODE_T *mode ) {
    *mode = _settings.mode;
    return mmal_status_to_int(MMAL_SUCCESS);
}


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
    fprintf(stdout, "Framerate    : %d\n", params->framerate);
    fprintf(stdout, "Stabilisation: %d\n", params->stabilisation);
    fprintf(stdout, "DRC          : %d\n", params->drc);
    fprintf(stdout, "Sharpness    : %d\n", params->sharpness);
    fprintf(stdout, "Contrast     : %d\n", params->contrast);
    fprintf(stdout, "Brightness   : %d\n", params->brightness);
    fprintf(stdout, "Saturation   : %d\n", params->saturation);
    fprintf(stdout, "ISO          : %d\n", params->iso);
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
    if (_settings.verbose) fprintf(stdout, "%s: Framerate     :%d\n", __func__, params->framerate);
    status += setFramerate(params->framerate);
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
    if (_settings.verbose) fprintf(stdout, "%s: Shutterspeed  :%d\n", __func__, params->shutterspeed);
    status += setShutterSpeed(params->shutterspeed);
    if (_settings.verbose) fprintf(stdout, "%s: AWB-Gains     :%d/%d\n", __func__, params->awbgain_red, params->awbgain_blue);
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
        status += getFramerate( &(params->framerate) );
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
    if (( !_camera_component ) || ( _active )) return 1;
    rotation = ((rotation % 360 ) / 90) * 90;
    MMAL_STATUS_T status = mmal_port_parameter_set_int32(_camera_component->output[MMAL_CAMERA_CAPTURE_PORT], MMAL_PARAMETER_ROTATION, rotation);        
    if ( status == MMAL_SUCCESS ) _params.rotation = rotation;
    return mmal_status_to_int(status);
}

int FlashCam::getRotation ( int *rotation ) {
    if (( !_camera_component ) || ( _active )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_get_int32(_camera_component->output[MMAL_CAMERA_CAPTURE_PORT], MMAL_PARAMETER_ROTATION, rotation);        
    return mmal_status_to_int(status);
}

int FlashCam::setAWBMode ( MMAL_PARAM_AWBMODE_T awb ) {
    if (( !_camera_component ) || ( _active )) return 1;
    MMAL_PARAMETER_AWBMODE_T param  = {{MMAL_PARAMETER_AWB_MODE, sizeof(param)}, awb};
    MMAL_STATUS_T            status = mmal_port_parameter_set(_camera_component->control, &param.hdr);   
    if ( status == MMAL_SUCCESS ) _params.awbmode = awb;
    return mmal_status_to_int(status);
}

int FlashCam::getAWBMode ( MMAL_PARAM_AWBMODE_T *awb ) {
    if (( !_camera_component ) || ( _active )) return 1;
    // MMAL_PARAM_AWBMODE_MAX: just usa a value to allocate `param`
    MMAL_PARAMETER_AWBMODE_T param  = {{MMAL_PARAMETER_AWB_MODE, sizeof(param)}, MMAL_PARAM_AWBMODE_MAX};
    MMAL_STATUS_T            status = mmal_port_parameter_get(_camera_component->control, &param.hdr);
    //update value
    *awb = param.value;
    return mmal_status_to_int(status);
}

int FlashCam::setFlashMode ( MMAL_PARAM_FLASH_T flash ) {
    if (( !_camera_component ) || ( _active )) return 1;
    MMAL_PARAMETER_FLASH_T param  = {{MMAL_PARAMETER_FLASH, sizeof(param)}, flash};
    MMAL_STATUS_T          status = mmal_port_parameter_set(_camera_component->control, &param.hdr);
    if ( status == MMAL_SUCCESS ) _params.flashmode = flash;
    return mmal_status_to_int(status);
}

int FlashCam::getFlashMode ( MMAL_PARAM_FLASH_T *flash ) {
    if (( !_camera_component ) || ( _active )) return 1;
    // MMAL_PARAM_FLASH_MAX: just usa a value to allocate `param`
    MMAL_PARAMETER_FLASH_T param  = {{MMAL_PARAMETER_FLASH, sizeof(param)}, MMAL_PARAM_FLASH_MAX};
    MMAL_STATUS_T          status = mmal_port_parameter_get(_camera_component->control, &param.hdr);
    //update value
    *flash = param.value;
    return mmal_status_to_int(status);
}

int FlashCam::setMirror ( MMAL_PARAM_MIRROR_T mirror ) {
    if (( !_camera_component ) || ( _active )) return 1;
    MMAL_PARAMETER_MIRROR_T param  = {{MMAL_PARAMETER_MIRROR, sizeof(param)}, mirror};
    MMAL_STATUS_T           status = mmal_port_parameter_set(_camera_component->output[MMAL_CAMERA_CAPTURE_PORT], &param.hdr);
    if ( status == MMAL_SUCCESS ) _params.mirror = mirror;
    return mmal_status_to_int(status);
}

int FlashCam::getMirror ( MMAL_PARAM_MIRROR_T *mirror ) {
    if (( !_camera_component ) || ( _active )) return 1;
    // MMAL_PARAM_MIRROR_NONE: just usa a value to allocate `param`
    MMAL_PARAMETER_MIRROR_T param  = {{MMAL_PARAMETER_MIRROR, sizeof(param)}, MMAL_PARAM_MIRROR_NONE};
    MMAL_STATUS_T           status = mmal_port_parameter_get(_camera_component->output[MMAL_CAMERA_CAPTURE_PORT], &param.hdr);       
    //update value
    *mirror = param.value;
    return mmal_status_to_int(status);
}

int FlashCam::setCameraNum ( unsigned int num ) {
    if (( !_camera_component ) || ( _active )) return 1;
    MMAL_PARAMETER_UINT32_T param  = {{MMAL_PARAMETER_CAMERA_NUM, sizeof(param)}, num};
    MMAL_STATUS_T           status = mmal_port_parameter_set(_camera_component->control, &param.hdr);
    if ( status == MMAL_SUCCESS ) _params.cameranum = num;
    return mmal_status_to_int(status);
}

int FlashCam::getCameraNum ( unsigned int *num ) {
    if (( !_camera_component ) || ( _active )) return 1;
    // MMAL_PARAM_MIRROR_NONE: just usa a value to allocate `param`
    MMAL_PARAMETER_UINT32_T param  = {{MMAL_PARAMETER_CAMERA_NUM, sizeof(param)}, 0};
    MMAL_STATUS_T           status = mmal_port_parameter_get(_camera_component->control, &param.hdr);       
    //update value
    *num = param.value;
    return mmal_status_to_int(status);
}

int FlashCam::setCapture ( int capture ) {
    if (( !_camera_component ) || ( _active )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_set_boolean(_camera_component->output[MMAL_CAMERA_CAPTURE_PORT], MMAL_PARAMETER_CAPTURE, capture);
    //value is only needed when taking a snapshot, so it is not tracked in _params
    //if ( status == MMAL_SUCCESS ) _params.capture = capture;
    return mmal_status_to_int(status);
}

int FlashCam::getCapture ( int *capture ) {
    if (( !_camera_component ) || ( _active )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_get_boolean(_camera_component->output[MMAL_CAMERA_CAPTURE_PORT], MMAL_PARAMETER_CAPTURE, capture);   
    return mmal_status_to_int(status);
}

int FlashCam::setExposureMode ( MMAL_PARAM_EXPOSUREMODE_T exposure ) {
    if (( !_camera_component ) || ( _active )) return 1;
    MMAL_PARAMETER_EXPOSUREMODE_T param  = {{MMAL_PARAMETER_EXPOSURE_MODE, sizeof(param)}, exposure};
    MMAL_STATUS_T                 status = mmal_port_parameter_set(_camera_component->control, &param.hdr);
    if ( status == MMAL_SUCCESS ) _params.exposuremode = exposure;
    return mmal_status_to_int(status);
}

int FlashCam::getExposureMode ( MMAL_PARAM_EXPOSUREMODE_T *exposure ) {
    // MMAL_PARAM_EXPOSUREMODE_MAX: just usa a value to allocate `param`
    if (( !_camera_component ) || ( _active )) return 1;
    MMAL_PARAMETER_EXPOSUREMODE_T param  = {{MMAL_PARAMETER_EXPOSURE_MODE, sizeof(param)}, MMAL_PARAM_EXPOSUREMODE_MAX};
    MMAL_STATUS_T                 status = mmal_port_parameter_get(_camera_component->control, &param.hdr);       
    //update value
    *exposure = param.value;
    return mmal_status_to_int(status);
}

int FlashCam::setMeteringMode ( MMAL_PARAM_EXPOSUREMETERINGMODE_T  metering ) {
    if (( !_camera_component ) || ( _active )) return 1;
    MMAL_PARAMETER_EXPOSUREMETERINGMODE_T param  = {{MMAL_PARAMETER_EXP_METERING_MODE, sizeof(param)}, metering};        
    MMAL_STATUS_T                         status = mmal_port_parameter_set(_camera_component->control, &param.hdr);
    if ( status == MMAL_SUCCESS ) _params.metering = metering;
    return mmal_status_to_int(status);
}

int FlashCam::getMeteringMode ( MMAL_PARAM_EXPOSUREMETERINGMODE_T *metering ) {
    if (( !_camera_component ) || ( _active )) return 1;
    // MMAL_PARAM_EXPOSUREMETERINGMODE_MAX: just usa a value to allocate `param`
    MMAL_PARAMETER_EXPOSUREMETERINGMODE_T param  = {{MMAL_PARAMETER_EXP_METERING_MODE, sizeof(param)}, MMAL_PARAM_EXPOSUREMETERINGMODE_MAX};
    MMAL_STATUS_T                         status = mmal_port_parameter_get(_camera_component->control, &param.hdr);       
    //update value
    *metering = param.value;
    return mmal_status_to_int(status);
}

int FlashCam::setCameraConfig ( MMAL_PARAMETER_CAMERA_CONFIG_T *config ) {
    if (( !_camera_component ) || ( _active )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_set(_camera_component->control, &config->hdr);
    return mmal_status_to_int(status);
}

int FlashCam::getCameraConfig ( MMAL_PARAMETER_CAMERA_CONFIG_T *config ) {
    if (( !_camera_component ) || ( _active )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_get(_camera_component->control, &config->hdr);       
    return mmal_status_to_int(status);
}

int FlashCam::setFrameRate ( int  fps ) {
    
}

int FlashCam::getFrameRate ( int *fps ) {
    
}

int FlashCam::setStabilisation ( int stabilisation ) {
    if (( !_camera_component ) || ( _active )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_set_boolean(_camera_component->control, MMAL_PARAMETER_VIDEO_STABILISATION, stabilisation);   
    if ( status == MMAL_SUCCESS ) _params.stabilisation = stabilisation;
    return mmal_status_to_int(status);
}

int FlashCam::getStabilisation ( int *stabilisation ) {
    if (( !_camera_component ) || ( _active )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_get_boolean(_camera_component->control, MMAL_PARAMETER_VIDEO_STABILISATION, stabilisation);        
    return mmal_status_to_int(status);
}

int FlashCam::setDRC ( MMAL_PARAMETER_DRC_STRENGTH_T  strength ) {
    if (( !_camera_component ) || ( _active )) return 1;
    MMAL_PARAMETER_DRC_T param  = {{MMAL_PARAMETER_DYNAMIC_RANGE_COMPRESSION, sizeof(param)}, strength};
    MMAL_STATUS_T        status = mmal_port_parameter_set(_camera_component->control, &param.hdr);
    if ( status == MMAL_SUCCESS ) _params.drc = strength;
    return mmal_status_to_int(status);
}

int FlashCam::getDRC ( MMAL_PARAMETER_DRC_STRENGTH_T *strength ) {
    if (( !_camera_component ) || ( _active )) return 1;
    // MMAL_PARAMETER_DRC_STRENGTH_MAX: just usa a value to allocate `param`
    MMAL_PARAMETER_DRC_T param  = {{MMAL_PARAMETER_DYNAMIC_RANGE_COMPRESSION, sizeof(param)}, MMAL_PARAMETER_DRC_STRENGTH_MAX};
    MMAL_STATUS_T        status = mmal_port_parameter_get(_camera_component->control, &param.hdr);  
    //update value
    *strength = param.strength;
    return mmal_status_to_int(status);
}

int FlashCam::setSharpness ( int  sharpness ) {
    if (( !_camera_component ) || ( _active )) return 1;
    if ( sharpness < -100 ) sharpness = -100;
    if ( sharpness >  100 ) sharpness =  100;
    MMAL_STATUS_T status = setParameterRational(MMAL_PARAMETER_SHARPNESS, sharpness);
    if ( status == MMAL_SUCCESS ) _params.sharpness = sharpness;
    return mmal_status_to_int(status);
}

int FlashCam::getSharpness ( int *sharpness ) {
    if (( !_camera_component ) || ( _active )) return 1;
    return getParameterRational( MMAL_PARAMETER_SHARPNESS, sharpness);        
}

int FlashCam::setContrast ( int  contrast ) {
    if (( !_camera_component ) || ( _active )) return 1;
    if ( contrast < -100 ) contrast = -100;
    if ( contrast >  100 ) contrast =  100;
    MMAL_STATUS_T status = setParameterRational(MMAL_PARAMETER_CONTRAST, contrast);
    if ( status == MMAL_SUCCESS ) _params.contrast = contrast;
    return mmal_status_to_int(status);
}

int FlashCam::getContrast ( int *contrast ) {
    if (( !_camera_component ) || ( _active )) return 1;
    return getParameterRational( MMAL_PARAMETER_CONTRAST, contrast);        
}

int FlashCam::setBrightness ( int  brightness ) {
    if (( !_camera_component ) || ( _active )) return 1;
    if ( brightness <    0 ) brightness =    0;
    if ( brightness >  100 ) brightness =  100;
    MMAL_STATUS_T status = setParameterRational(MMAL_PARAMETER_BRIGHTNESS, brightness);
    if ( status == MMAL_SUCCESS ) _params.brightness = brightness;
    return mmal_status_to_int(status);
}

int FlashCam::getBrightness ( int *brightness ) {
    if (( !_camera_component ) || ( _active )) return 1;
    return getParameterRational( MMAL_PARAMETER_BRIGHTNESS, brightness);        
}

int FlashCam::setSaturation ( int  saturation ) {        
    if (( !_camera_component ) || ( _active )) return 1;
    if ( saturation < -100 ) saturation = -100;
    if ( saturation >  100 ) saturation =  100;
    MMAL_STATUS_T status = setParameterRational(MMAL_PARAMETER_SATURATION, saturation);
    if ( status == MMAL_SUCCESS ) _params.saturation = saturation;
    return mmal_status_to_int(status);
}

int FlashCam::getSaturation ( int *saturation ) {
    if (( !_camera_component ) || ( _active )) return 1;
    return getParameterRational( MMAL_PARAMETER_SATURATION, saturation);        
}

int FlashCam::setISO ( unsigned int  iso ) {        
    if (( !_camera_component ) || ( _active )) return 1;
    if ( iso > 1600 ) iso = 1600;
    MMAL_STATUS_T status = mmal_port_parameter_set_uint32(_camera_component->control, MMAL_PARAMETER_ISO, iso); 
    if ( status == MMAL_SUCCESS ) _params.iso = iso;
    return mmal_status_to_int(status);
}

int FlashCam::getISO ( unsigned int *iso ) {        
    if (( !_camera_component ) || ( _active )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_get_uint32(_camera_component->control, MMAL_PARAMETER_ISO, iso);        
    return mmal_status_to_int(status);
}

int FlashCam::setShutterSpeed ( unsigned int  speed ) {        
    if (( !_camera_component ) || ( _active )) return 1;
    if ( speed > 330000 ) speed = 330000;
    MMAL_STATUS_T status = mmal_port_parameter_set_uint32(_camera_component->control, MMAL_PARAMETER_SHUTTER_SPEED, speed);  
    if ( status == MMAL_SUCCESS ) _params.shutterspeed = speed;
    return mmal_status_to_int(status);
}

int FlashCam::getShutterSpeed ( unsigned int *speed ) {        
    if (( !_camera_component ) || ( _active )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_get_uint32(_camera_component->control, MMAL_PARAMETER_SHUTTER_SPEED, speed);        
    return mmal_status_to_int(status);
}

int FlashCam::setAWBGains ( float red , float blue ) {
    if (( !_camera_component ) || ( _active )) return 1;
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
    return mmal_status_to_int(status);
}

int FlashCam::getAWBGains ( float *red , float *blue ) { 
    if (( !_camera_component ) || ( _active )) return 1;
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
    return mmal_status_to_int(status);
}

int FlashCam::setDenoise ( int  denoise ) {
    if (( !_camera_component ) || ( _active )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_set_boolean(_camera_component->control, MMAL_PARAMETER_STILLS_DENOISE, denoise);     
    if ( status == MMAL_SUCCESS ) _params.denoise = denoise;
    return mmal_status_to_int(status);
}

int FlashCam::getDenoise ( int *denoise ) {
    if (( !_camera_component ) || ( _active )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_get_boolean(_camera_component->control, MMAL_PARAMETER_STILLS_DENOISE, denoise);        
    return mmal_status_to_int(status);
}


int FlashCam::setChangeEventRequest ( unsigned int id , int  request ) {
    if (( !_camera_component ) || ( _active )) return 1;
    MMAL_PARAMETER_CHANGE_EVENT_REQUEST_T param  = {{MMAL_PARAMETER_CHANGE_EVENT_REQUEST, sizeof(param)}, id, request};
    MMAL_STATUS_T                         status = mmal_port_parameter_set(_camera_component->control, &param.hdr);
    return mmal_status_to_int(status);
}

int FlashCam::getChangeEventRequest ( unsigned int id , int *request ) {
    if (( !_camera_component ) || ( _active )) return 1;
    // 0: just usa a value to allocate `param`
    MMAL_PARAMETER_CHANGE_EVENT_REQUEST_T param  = {{MMAL_PARAMETER_CHANGE_EVENT_REQUEST, sizeof(param)}, id, 0};
    MMAL_STATUS_T                         status = mmal_port_parameter_get(_camera_component->control, &param.hdr);  
    //update value
    *request = param.enable;
    return mmal_status_to_int(status);
}









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
//#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

// Stills format information
// 0 implies variable
#define CAPTURE_FRAME_RATE_NUM 0
#define CAPTURE_FRAME_RATE_DEN 1
#define PREVIEW_FRAME_RATE_NUM 0
#define PREVIEW_FRAME_RATE_DEN 1

/*
 * Constructor
 */
FlashCam::FlashCam(){
    //we are not yet initialised
    _init = false;
    
    //init with current (=default!) parameter set;
    getDefaultParams(&_params);    
    initCamera(&_params);
}

FlashCam::FlashCam(FLASHCAM_PARAMS_T *params){
    //we are not yet initialised
    _init = false;
    
    //init with current (=default!) parameter set;
    initCamera(params);
}

/*
 * Destructor
 */
FlashCam::~FlashCam(){
    release();
    // delete semaphore
    vcos_semaphore_delete(&_userdata.sem_capture);
}

// initialize the camera with specified parameters
int FlashCam::initCamera(FLASHCAM_PARAMS_T *params) {
    
    // Are we already initialised?
    if (_init) {
        if (_params.verbose)
            fprintf(stderr, "Already initialised\n");
        return EX_OK;
    }
    
    // initialise bcm_host
    bcm_host_init();
    
    //register to VCOS for logging
    vcos_log_register("FlashCam", VCOS_LOG_CATEGORY);
    
    // init private variables
    _capturing                  = false;
    _camera_component           = NULL;
    _preview_component          = NULL;
    _preview_connection         = NULL;
    _camera_pool                = NULL;
    _framebuffer                = NULL;
    
    //set userdata
    _userdata.params            = &_params;
    _userdata.camera_pool       = NULL;
    _userdata.framebuffer       = NULL;
    _userdata.framebuffer_size  = 0;
    _userdata.framebuffer_idx   = 0;
    _userdata.callback          = NULL;
    
    // create & init semaphore
    VCOS_STATUS_T vcos_status = vcos_semaphore_create(&_userdata.sem_capture, "FlashCam-sem", 0);
    vcos_assert(vcos_status == VCOS_SUCCESS);
    
    //Status flags
    int exit_code               = EX_OK;
    MMAL_STATUS_T status        = MMAL_SUCCESS;
        
    //print that we are starting
    if (params->verbose)
        fprintf(stderr, "\n FlashCam Version: %s\n\n", FLASHCAM_VERSION_STRING);
    
    //Set params --> indirectly sets *params -> _params
    setAllParams(params);
    
    //copy meta data
    _params.verbose             = params->verbose;
    _params.width               = params->width;
    _params.height              = params->height;
    _params.getsettings         = params->getsettings;
    
    //setup camera
    // - internally sets:
    //      _camera_component
    //      _camera_pool
    //      _userdata.camera_pool
    //      _userdata.framebuffer
    //      _userdata.framebuffer_size
    if ((status = create_camera_component()) != MMAL_SUCCESS)  {
        vcos_log_error("%s: Failed to create camera component", __func__);
        release();
        return EX_SOFTWARE;
    }
    
    //setup preview (nullsink!)
    // -> But we need it as it is used for measurements such as awb
    // internally sets:
    //      _preview_component
    if ((status = create_preview_component()) != MMAL_SUCCESS) {
        vcos_log_error("%s: Failed to create preview component", __func__);
        release();
        return EX_SOFTWARE;
    }
    
    if (_params.verbose)
        fprintf(stderr, "Starting component connection stage\n");
    
    //get ports
    MMAL_PORT_T *capture_port = _camera_component->output[MMAL_CAMERA_CAPTURE_PORT];
    MMAL_PORT_T *preview_port = _camera_component->output[MMAL_CAMERA_PREVIEW_PORT];    
    // Note we are lucky that the preview and null sink components use the same input port
    // so we can simple do this without conditionals
    MMAL_PORT_T *preview_input_port = _preview_component->input[0];
    
    // Connect preview
    if ((status = connect_ports(preview_port, preview_input_port, &_preview_connection)) != MMAL_SUCCESS) {
        vcos_log_error("%s: Failed to connect camera to preview", __func__);
        release();
        return EX_SOFTWARE;
    }
    
    //Setup userdata / camera
    capture_port->userdata = (struct MMAL_PORT_USERDATA_T *)&_userdata;
    
    if (_params.verbose)
        fprintf(stderr, "Enabling camera still output port\n");
    
    // Enable the camera still output port and tell it its callback function
    if ((status = mmal_port_enable(capture_port, FlashCam::buffer_callback)) != MMAL_SUCCESS) {
        vcos_log_error("Failed to setup camera output");
        release();
        return EX_SOFTWARE;
    }
    
    if (_params.verbose)
        fprintf(stderr, "Finished setup\n");
    
    _init = true;
    return EX_OK;
}

/*
 * void FlashCam::control_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
 *  Callback function for control-events
 */
void FlashCam::control_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    fprintf(stderr, "Camera control callback  cmd=0x%08x", buffer->cmd);
    
    //check received event
    if (buffer->cmd == MMAL_EVENT_PARAMETER_CHANGED) {        
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
                
            default:
            {
                vcos_log_error("Received unexpected updated: id=%d", param->hdr.id);
            }
                break;
        }
    } else if (buffer->cmd == MMAL_EVENT_ERROR) {
        vcos_log_error("No data received from sensor. Check all connections, including the Sunny one on the camera board");
    } else {
        vcos_log_error("Received unexpected camera control callback event, 0x%08x", buffer->cmd);
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
            
            if (userdata->params->verbose) {
                fprintf(stderr, "Copying %d bytes @ %d (%d x %d)\n",  buffer->length, userdata->framebuffer_idx, userdata->params->height, userdata->params->width);
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
            max_idx = userdata->framebuffer_idx + buffer->length;
            
            //does it fit in buffer?
            if ( max_idx > userdata->framebuffer_size ) {
                vcos_log_error("Framebuffer full (%d > %d) - aborting.." , max_idx , userdata->framebuffer_size );
                abort = 1;
            } else {
                memcpy ( &userdata->framebuffer[userdata->framebuffer_idx] , buffer->data , buffer->length );
                userdata->framebuffer_idx += buffer->length;
                //if (userdata->params->verbose) fprintf(stderr, "Done...\n");
            }
        }
        
        // Check end of frame or error
        if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_TRANSMISSION_FAILED)    
            abort = 1;
        
        if (buffer->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END)              
            complete = 1;
        
    } else {
        vcos_log_error("Received a camera still buffer callback with no state");
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
            vcos_log_error("Unable to return the buffer to the camera still port");
    }
    
    //post that we are done
    if (abort) {
        fprintf(stderr, "Aborting.. \n");
        vcos_semaphore_post(&(userdata->sem_capture));
    } else if (complete) {
        fprintf(stderr, "Complete! \n");
        
        //TODO: locking & ensuring no call to camera can be maded
        if (userdata->callback) {
            fprintf(stderr, "Callback assigned \n");
            userdata->callback( userdata->framebuffer , userdata->params->width ,  userdata->params->height);
        } else {
            fprintf(stderr, "No Callback \n");
        }
        
        //release semaphore
        fprintf(stderr, "Release.. \n");
        userdata->framebuffer_idx = 0;
        vcos_semaphore_post(&(userdata->sem_capture));
    }
}



//Camera setup function
MMAL_STATUS_T FlashCam::create_camera_component() {
    MMAL_STATUS_T status;
    
    // Create Component
    if ((status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &_camera_component)) != MMAL_SUCCESS) {
        vcos_log_error("Failed to create camera component");
        destroy_component( _camera_component );        
        return status;
    }
    
    // Select Camera
    if ( setCameraNum(_params.cameranum) ) {
        vcos_log_error("Could not select camera");
        destroy_component( _camera_component );        
        return MMAL_EINVAL;
    }
    
    // Validate outputs are available
    if (!_camera_component->output_num){
        vcos_log_error("Camera doesn't have output ports");
        destroy_component( _camera_component );        
        return MMAL_EINVAL;
    }
    
    MMAL_PORT_T *preview_port = _camera_component->output[MMAL_CAMERA_PREVIEW_PORT];
    MMAL_PORT_T *capture_port = _camera_component->output[MMAL_CAMERA_CAPTURE_PORT];
    
    // Do we want setting-update messages?
    if (_params.getsettings) {
        if (setChangeEventRequest(MMAL_PARAMETER_CAMERA_SETTINGS, 1)) {
            vcos_log_error("No camera settings events");
        }
    }
    
    // Enable the camera, and tell it its control callback function
    if ((status = mmal_port_enable(_camera_component->control, FlashCam::control_callback)) != MMAL_SUCCESS ) {
        vcos_log_error("Unable to enable control port : error %d", status);
        destroy_component( _camera_component );        
        return status;
    }
    
    // Align height/width
    _params.width  = VCOS_ALIGN_UP(_params.width, 32);
    _params.height = VCOS_ALIGN_UP(_params.height, 16);
    
    // setup the camera configuration
    MMAL_PARAMETER_CAMERA_CONFIG_T cam_config =
    {
        { MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) },
        .max_stills_w                           = _params.width,
        .max_stills_h                           = _params.height,
        .stills_yuv422                          = 0,
        .one_shot_stills                        = 1,
        .max_preview_video_w                    = _params.width,
        .max_preview_video_h                    = _params.height,
        .num_preview_video_frames               = 3,
        .stills_capture_circular_buffer_height  = 0,
        .fast_preview_resume                    = 0,
        .use_stc_timestamp                      = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
    };
    
    if ( setCameraConfig( &cam_config ) ) {
        vcos_log_error("Could not setup camera");
        destroy_component( _camera_component );        
        return MMAL_EINVAL;
    }
    
    // setup port-formats
    MMAL_ES_FORMAT_T *format = preview_port->format;
    
    //Preview format
    format->encoding                    = MMAL_ENCODING_OPAQUE;
    format->encoding_variant            = MMAL_ENCODING_I420;        
    format->es->video.width             = _params.width;
    format->es->video.height            = _params.height;
    format->es->video.crop.x            = 0;
    format->es->video.crop.y            = 0;
    format->es->video.crop.width        = _params.width;
    format->es->video.crop.height       = _params.height;
    format->es->video.frame_rate.num    = PREVIEW_FRAME_RATE_NUM;
    format->es->video.frame_rate.den    = PREVIEW_FRAME_RATE_DEN;
    
    //Update preview-port with set format
    if ((status = mmal_port_format_commit(preview_port)) != MMAL_SUCCESS ) {
        vcos_log_error("Preview format couldn't be set");
        destroy_component( _camera_component );        
        return status;
    }
    
    //Capture format ==> same as Preview, except for encoding (YUV!)
    format->encoding                    = MMAL_ENCODING_I420;
    format->encoding_variant            = MMAL_ENCODING_I420;     
    // YUV-data framesize 
    unsigned int framebuffer_size       = VCOS_ALIGN_UP(_params.width * _params.height * 1.5, 32);
    
    // RGB encoding..
    //format->encoding = mmal_util_rgb_order_fixed(still_port) ? MMAL_ENCODING_RGB24 : MMAL_ENCODING_BGR24;
    //format->encoding_variant = 0;
    format->es->video.frame_rate.num    = CAPTURE_FRAME_RATE_NUM;
    format->es->video.frame_rate.den    = CAPTURE_FRAME_RATE_DEN;
    
    //Set correct buffer sizes 
    if (capture_port->buffer_size < capture_port->buffer_size_min)
        capture_port->buffer_size = capture_port->buffer_size_min;
    
    //Allow atleast 2 buffer for when the usercallback stalls
    capture_port->buffer_num = capture_port->buffer_num_recommended + 1;
    
    if (_params.verbose) {
        fprintf(stderr, "Camera Pool size  : %d\n", capture_port->buffer_num);
        fprintf(stderr, "Camera Buffer size: %d\n", capture_port->buffer_size);
        fprintf(stderr, "Total Pool size   : %d\n", capture_port->buffer_num * capture_port->buffer_size);
    }
    
    
    //Update capture-port with set format
    if ((status = mmal_port_format_commit(capture_port)) != MMAL_SUCCESS ) {
        vcos_log_error("Capture format couldn't be set");
        destroy_component( _camera_component );        
        return status;
    }
    
    //Enable camera
    if ((status = mmal_component_enable(_camera_component)) != MMAL_SUCCESS ) {
        vcos_log_error("Camera component couldn't be enabled");
        destroy_component( _camera_component );        
        return status;
    }
    
    // Create pool of buffer headers for the output port to consume
    _camera_pool = mmal_port_pool_create(capture_port, capture_port->buffer_num, capture_port->buffer_size);
    if (!_camera_pool) {
        vcos_log_error("Failed to create buffer header pool");
    } else {
        _userdata.camera_pool = _camera_pool;
    }
    
    //create buffer for image
    _framebuffer = new unsigned char[framebuffer_size];
    if (!_framebuffer) {
        vcos_log_error("Failed to allocate image buffer");
    } else {
        _userdata.framebuffer       = _framebuffer;
        _userdata.framebuffer_size  = framebuffer_size; 
    }
    
    
    if (_params.verbose)
        fprintf(stderr, "Camera component done\n");
    
    return MMAL_SUCCESS;
}

//Preview setup functions
MMAL_STATUS_T FlashCam::create_preview_component() {
    MMAL_STATUS_T status;
    
    //set nullsink
    if ((status = mmal_component_create("vc.null_sink", &_preview_component)) != MMAL_SUCCESS) {
        vcos_log_error("Unable to create null sink component (%u)", status);
        destroy_component( _preview_component );        
        return status;
    }
    
    // Enable component
    if ((status = mmal_component_enable( _preview_component )) != MMAL_SUCCESS) {
        vcos_log_error("Unable to enable preview/null sink component (%u)", status);
        destroy_component( _preview_component );        
        return status;
    }
    
    return MMAL_SUCCESS;
}


// Delete component
void FlashCam::destroy_component(MMAL_COMPONENT_T *component) {
    if (component) {
        mmal_component_destroy(component);
        component = NULL;
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


void FlashCam::release() {
    if (_params.verbose)
        fprintf(stderr, "Closing down\n");
    
    //reset init flag    
    _init = false;
    
    // Disable connections
    if (_preview_connection)
        mmal_connection_destroy(_preview_connection);
    
    // Disable ports
    if (_preview_component)
        mmal_component_disable(_preview_component);
    
    if (_camera_component)
        mmal_component_disable(_camera_component);
    
    // Dealloc components
    destroy_component( _preview_component );
    destroy_component( _camera_component  );
    
    //clear buffer
    if (_framebuffer) 
        delete[] _framebuffer;
    
    if (_params.verbose)
        fprintf(stderr, "Close down completed, all components disconnected, disabled and destroyed\n\n");
}

/*
 * int FlashCam::capture( void )
 * 
 * Capture an image
 *
 */
int FlashCam::capture() {
    
    if (!_init) {
        vcos_log_error("Camera not initialised\n");
        return EX_SOFTWARE;
    }
    
    if (_capturing) {
        vcos_log_error("Camera already waiting capturing..\n");
        return EX_OK;
    } 
    
    // buffer pointer
    MMAL_BUFFER_HEADER_T *buffer;
    // number of buffers
    int qsize = mmal_queue_length(_camera_pool->queue) - 1;
    
    // Send all the buffers to the camera output port
    for (int i=0; i<qsize; i++) {
        //get buffer
        buffer = mmal_queue_get(_camera_pool->queue);
        
        if (!buffer)
            vcos_log_error("Unable to get a required buffer %d from pool queue", i);
        
        if (mmal_port_send_buffer(_camera_component->output[MMAL_CAMERA_CAPTURE_PORT], buffer)!= MMAL_SUCCESS)
            vcos_log_error("Unable to send a buffer to camera output port (%d)", i);
    }
    
    if (_params.verbose)
        fprintf(stderr, "Starting capture\n");
    
    //create capture window
    //cv::namedWindow( "cvwindow", cv::WINDOW_AUTOSIZE );
    
    if (setCapture(1)) {
        vcos_log_error("%s: Failed to start capture", __func__);
        return EX_SOFTWARE;
    }
    
    // Wait for capture to complete
    // For some reason using vcos_semaphore_wait_timeout sometimes returns immediately with bad parameter error
    // even though it appears to be all correct, so reverting to untimed one until figure out why its erratic
    _capturing = true;
    vcos_semaphore_wait(&_userdata.sem_capture);
    _capturing = false;
    
    if (_params.verbose)
        fprintf(stderr, "Finished capture\n");
    
    // cv::imshow ("cvwindow", state.cvimage);
    // cv::waitKey(0);
    
    return EX_OK;
}

void FlashCam::setFrameCallback(FLASHCAM_CALLBACK_T callback) {
    if (_capturing) return; //no changer/reset while in capturemode
    _userdata.callback = callback;
}

void FlashCam::resetFrameCallback() {
    if (_capturing) return; //no changer/reset while in capturemode
    _userdata.callback = NULL;
}

















/* PARAMETER MANAGEMENT */

int FlashCam::setAllParams(FLASHCAM_PARAMS_T *params) {
    int status = 0;
    
    //set values
    if (_params.verbose) fprintf(stderr, "Setting:Rotation\n");        
    status += setRotation(_params.rotation);        
    if (_params.verbose) fprintf(stderr, "Setting:AWB\n");
    status += setAWBMode(_params.awb);
    if (_params.verbose) fprintf(stderr, "Setting:Flash\n");
    status += setFlashMode(_params.flash);
    if (_params.verbose) fprintf(stderr, "Setting:Mirror\n");
    status += setMirror(_params.mirror);
    if (_params.verbose) fprintf(stderr, "Setting:CameraNum\n");
    status += setCameraNum(_params.cameranum);
    if (_params.verbose) fprintf(stderr, "Setting:Exposure\n");
    status += setExposureMode(_params.exposure);
    if (_params.verbose) fprintf(stderr, "Setting:Metering\n");
    status += setMeteringMode(_params.metering);
    if (_params.verbose) fprintf(stderr, "Setting:Stabilisation\n");
    status += setStabilisation(_params.stabilisation);
    if (_params.verbose) fprintf(stderr, "Setting:DRC\n");
    status += setDRC(_params.strength);
    if (_params.verbose) fprintf(stderr, "Setting:Sharpness\n");
    status += setSharpness(_params.sharpness);
    if (_params.verbose) fprintf(stderr, "Setting:Contrast\n");
    status += setContrast(_params.contrast);
    if (_params.verbose) fprintf(stderr, "Setting:Brightness\n");
    status += setBrightness(_params.brightness);
    if (_params.verbose) fprintf(stderr, "Setting:Saturation\n");
    status += setSaturation(_params.saturation);
    if (_params.verbose) fprintf(stderr, "Setting:ISO\n");
    status += setISO(_params.iso);
    if (_params.verbose) fprintf(stderr, "Setting:Shutterspeed\n");
    status += setShutterSpeed(_params.speed);
    if (_params.verbose) fprintf(stderr, "Setting:getAWBGains\n");
    status += setAWBGains(_params.awbgain_red, _params.awbgain_blue);
    if (_params.verbose) fprintf(stderr, "Setting:Denoise\n");
    status += setDenoise(_params.denoise);
    
    //if (_params.verbose) fprintf(stderr, "Setting:Width   - Ignored: metadata\n");    
    //if (_params.verbose) fprintf(stderr, "Setting:Height  - Ignored: metadata\n");    
    //if (_params.verbose) fprintf(stderr, "Setting:Verbose - Ignored: metadata\n");    
    
    return status;
}

int FlashCam::getAllParams(FLASHCAM_PARAMS_T *params, bool mem) {
    int status = 0;
    
    if (mem) {
        if (_params.verbose) fprintf(stderr, "Copying params from memory\n");
        memcpy(params, &_params, sizeof(FLASHCAM_PARAMS_T));
        
    } else {
        if (_params.verbose) fprintf(stderr, "Getting:Rotation\n");
        status += getRotation( &(params->rotation) );
        if (_params.verbose) fprintf(stderr, "Getting:AWB\n");
        status += getAWBMode( &(params->awb) );
        if (_params.verbose) fprintf(stderr, "Getting:Flash\n");
        status += getFlashMode( &(params->flash) );
        if (_params.verbose) fprintf(stderr, "Getting:Mirror\n");
        status += getMirror( &(params->mirror) );
        if (_params.verbose) fprintf(stderr, "Getting:CameraNum\n");
        status += getCameraNum( &(params->cameranum) );
        if (_params.verbose) fprintf(stderr, "Getting:Exposure\n");
        status += getExposureMode( &(params->exposure) );
        if (_params.verbose) fprintf(stderr, "Getting:Metering\n");
        status += getMeteringMode( &(params->metering) );
        if (_params.verbose) fprintf(stderr, "Getting:Stabilisation\n");
        status += getStabilisation( &(params->stabilisation) );
        if (_params.verbose) fprintf(stderr, "Getting:DRC\n");
        status += getDRC( &(params->strength) );
        if (_params.verbose) fprintf(stderr, "Getting:Sharpness\n");
        status += getSharpness( &(params->sharpness) );
        if (_params.verbose) fprintf(stderr, "Getting:Contrast\n");
        status += getContrast( &(params->contrast) );
        if (_params.verbose) fprintf(stderr, "Getting:Brightness\n");
        status += getBrightness( &(params->brightness) );
        if (_params.verbose) fprintf(stderr, "Getting:Saturation\n");
        status += getSaturation( &(params->saturation) );
        if (_params.verbose) fprintf(stderr, "Getting:ISO\n");
        status += getISO( &(params->iso) );
        if (_params.verbose) fprintf(stderr, "Getting:Shutterspeed\n");
        status += getShutterSpeed( &(params->speed) );
        if (_params.verbose) fprintf(stderr, "Getting:getAWBGains\n");
        status += getAWBGains( &(params->awbgain_red),  &(params->awbgain_blue) );
        if (_params.verbose) fprintf(stderr, "Getting:Denoise\n");
        status += getDenoise( &(params->denoise) );
    }
    return status;
}

void FlashCam::printParams(FLASHCAM_PARAMS_T *params) {
    fprintf(stderr, "Rotation     : %d\n", params->rotation);
    fprintf(stderr, "AWB          : %d\n", params->awb);
    fprintf(stderr, "Flash        : %d\n", params->flash);
    fprintf(stderr, "Mirror       : %d\n", params->mirror);
    fprintf(stderr, "Camera Num   : %d\n", params->cameranum);
    fprintf(stderr, "Exposure     : %d\n", params->exposure);
    fprintf(stderr, "Metering     : %d\n", params->metering);
    fprintf(stderr, "Stabilisation: %d\n", params->stabilisation);
    fprintf(stderr, "DRC          : %d\n", params->strength);
    fprintf(stderr, "Sharpness    : %d\n", params->sharpness);
    fprintf(stderr, "Contrast     : %d\n", params->contrast);
    fprintf(stderr, "Brightness   : %d\n", params->brightness);
    fprintf(stderr, "Saturation   : %d\n", params->saturation);
    fprintf(stderr, "ISO          : %d\n", params->iso);
    fprintf(stderr, "Shutterspeed : %d\n", params->speed);
    fprintf(stderr, "AWB-red      : %d\n", params->awbgain_red);
    fprintf(stderr, "AWB-blue     : %d\n", params->awbgain_blue);
    fprintf(stderr, "Denoise      : %d\n", params->denoise);
    fprintf(stderr, "Width        : %d\n", params->width);
    fprintf(stderr, "Height       : %d\n", params->height);
    fprintf(stderr, "Verbose      : %d\n", params->verbose);
    
}

void FlashCam::getDefaultParams(FLASHCAM_PARAMS_T *params) {
    params->rotation        = 0;
    params->awb             = MMAL_PARAM_AWBMODE_AUTO;
    params->flash           = MMAL_PARAM_FLASH_OFF;
    params->mirror          = MMAL_PARAM_MIRROR_NONE;
    params->cameranum       = 0;
    params->exposure        = MMAL_PARAM_EXPOSUREMODE_AUTO;
    params->metering        = MMAL_PARAM_EXPOSUREMETERINGMODE_AVERAGE;
    params->stabilisation   = 0;
    params->strength        = MMAL_PARAMETER_DRC_STRENGTH_OFF;
    params->sharpness       = 0;
    params->contrast        = 0;
    params->brightness      = 50;
    params->saturation      = 0;
    params->iso             = 0;
    params->speed           = 0;
    params->awbgain_red     = 0;
    params->awbgain_blue    = 0;
    params->denoise         = 0;
    
    //meta params
    params->width           = 640;
    params->height          = 480;
    params->verbose         = 1;
    params->getsettings     = 1;
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
    if (( !_camera_component ) || ( _capturing )) return 1;
    rotation = ((rotation % 360 ) / 90) * 90;
    MMAL_STATUS_T status = mmal_port_parameter_set_int32(_camera_component->output[MMAL_CAMERA_CAPTURE_PORT], MMAL_PARAMETER_ROTATION, rotation);        
    if ( status == MMAL_SUCCESS ) _params.rotation = rotation;
    return mmal_status_to_int(status);
}

int FlashCam::getRotation ( int *rotation ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_get_int32(_camera_component->output[MMAL_CAMERA_CAPTURE_PORT], MMAL_PARAMETER_ROTATION, rotation);        
    return mmal_status_to_int(status);
}

int FlashCam::setAWBMode ( MMAL_PARAM_AWBMODE_T awb ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    MMAL_PARAMETER_AWBMODE_T param  = {{MMAL_PARAMETER_AWB_MODE, sizeof(param)}, awb};
    MMAL_STATUS_T            status = mmal_port_parameter_set(_camera_component->control, &param.hdr);   
    if ( status == MMAL_SUCCESS ) _params.awb = awb;
    return mmal_status_to_int(status);
}

int FlashCam::getAWBMode ( MMAL_PARAM_AWBMODE_T *awb ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    // MMAL_PARAM_AWBMODE_MAX: just usa a value to allocate `param`
    MMAL_PARAMETER_AWBMODE_T param  = {{MMAL_PARAMETER_AWB_MODE, sizeof(param)}, MMAL_PARAM_AWBMODE_MAX};
    MMAL_STATUS_T            status = mmal_port_parameter_get(_camera_component->control, &param.hdr);
    //update value
    *awb = param.value;
    return mmal_status_to_int(status);
}

int FlashCam::setFlashMode ( MMAL_PARAM_FLASH_T flash ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    MMAL_PARAMETER_FLASH_T param  = {{MMAL_PARAMETER_FLASH, sizeof(param)}, flash};
    MMAL_STATUS_T          status = mmal_port_parameter_set(_camera_component->control, &param.hdr);
    if ( status == MMAL_SUCCESS ) _params.flash = flash;
    return mmal_status_to_int(status);
}

int FlashCam::getFlashMode ( MMAL_PARAM_FLASH_T *flash ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    // MMAL_PARAM_FLASH_MAX: just usa a value to allocate `param`
    MMAL_PARAMETER_FLASH_T param  = {{MMAL_PARAMETER_FLASH, sizeof(param)}, MMAL_PARAM_FLASH_MAX};
    MMAL_STATUS_T          status = mmal_port_parameter_get(_camera_component->control, &param.hdr);
    //update value
    *flash = param.value;
    return mmal_status_to_int(status);
}

int FlashCam::setMirror ( MMAL_PARAM_MIRROR_T mirror ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    MMAL_PARAMETER_MIRROR_T param  = {{MMAL_PARAMETER_MIRROR, sizeof(param)}, mirror};
    MMAL_STATUS_T           status = mmal_port_parameter_set(_camera_component->output[MMAL_CAMERA_CAPTURE_PORT], &param.hdr);
    if ( status == MMAL_SUCCESS ) _params.mirror = mirror;
    return mmal_status_to_int(status);
}

int FlashCam::getMirror ( MMAL_PARAM_MIRROR_T *mirror ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    // MMAL_PARAM_MIRROR_NONE: just usa a value to allocate `param`
    MMAL_PARAMETER_MIRROR_T param  = {{MMAL_PARAMETER_MIRROR, sizeof(param)}, MMAL_PARAM_MIRROR_NONE};
    MMAL_STATUS_T           status = mmal_port_parameter_get(_camera_component->output[MMAL_CAMERA_CAPTURE_PORT], &param.hdr);       
    //update value
    *mirror = param.value;
    return mmal_status_to_int(status);
}

int FlashCam::setCameraNum ( unsigned int num ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    MMAL_PARAMETER_UINT32_T param  = {{MMAL_PARAMETER_CAMERA_NUM, sizeof(param)}, num};
    MMAL_STATUS_T           status = mmal_port_parameter_set(_camera_component->control, &param.hdr);
    if ( status == MMAL_SUCCESS ) _params.cameranum = num;
    return mmal_status_to_int(status);
}

int FlashCam::getCameraNum ( unsigned int *num ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    // MMAL_PARAM_MIRROR_NONE: just usa a value to allocate `param`
    MMAL_PARAMETER_UINT32_T param  = {{MMAL_PARAMETER_CAMERA_NUM, sizeof(param)}, 0};
    MMAL_STATUS_T           status = mmal_port_parameter_get(_camera_component->control, &param.hdr);       
    //update value
    *num = param.value;
    return mmal_status_to_int(status);
}

int FlashCam::setCapture ( int capture ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_set_boolean(_camera_component->output[MMAL_CAMERA_CAPTURE_PORT], MMAL_PARAMETER_CAPTURE, capture);
    //value is only needed when taking a snapshot, so it is not tracked in _params
    //if ( status == MMAL_SUCCESS ) _params.capture = capture;
    return mmal_status_to_int(status);
}

int FlashCam::getCapture ( int *capture ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_get_boolean(_camera_component->output[MMAL_CAMERA_CAPTURE_PORT], MMAL_PARAMETER_CAPTURE, capture);   
    return mmal_status_to_int(status);
}

int FlashCam::setExposureMode ( MMAL_PARAM_EXPOSUREMODE_T exposure ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    MMAL_PARAMETER_EXPOSUREMODE_T param  = {{MMAL_PARAMETER_EXPOSURE_MODE, sizeof(param)}, exposure};
    MMAL_STATUS_T                 status = mmal_port_parameter_set(_camera_component->control, &param.hdr);
    if ( status == MMAL_SUCCESS ) _params.exposure = exposure;
    return mmal_status_to_int(status);
}

int FlashCam::getExposureMode ( MMAL_PARAM_EXPOSUREMODE_T *exposure ) {
    // MMAL_PARAM_EXPOSUREMODE_MAX: just usa a value to allocate `param`
    if (( !_camera_component ) || ( _capturing )) return 1;
    MMAL_PARAMETER_EXPOSUREMODE_T param  = {{MMAL_PARAMETER_EXPOSURE_MODE, sizeof(param)}, MMAL_PARAM_EXPOSUREMODE_MAX};
    MMAL_STATUS_T                 status = mmal_port_parameter_get(_camera_component->control, &param.hdr);       
    //update value
    *exposure = param.value;
    return mmal_status_to_int(status);
}

int FlashCam::setMeteringMode ( MMAL_PARAM_EXPOSUREMETERINGMODE_T  metering ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    MMAL_PARAMETER_EXPOSUREMETERINGMODE_T param  = {{MMAL_PARAMETER_EXP_METERING_MODE, sizeof(param)}, metering};        
    MMAL_STATUS_T                         status = mmal_port_parameter_set(_camera_component->control, &param.hdr);
    if ( status == MMAL_SUCCESS ) _params.metering = metering;
    return mmal_status_to_int(status);
}

int FlashCam::getMeteringMode ( MMAL_PARAM_EXPOSUREMETERINGMODE_T *metering ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    // MMAL_PARAM_EXPOSUREMETERINGMODE_MAX: just usa a value to allocate `param`
    MMAL_PARAMETER_EXPOSUREMETERINGMODE_T param  = {{MMAL_PARAMETER_EXP_METERING_MODE, sizeof(param)}, MMAL_PARAM_EXPOSUREMETERINGMODE_MAX};
    MMAL_STATUS_T                         status = mmal_port_parameter_get(_camera_component->control, &param.hdr);       
    //update value
    *metering = param.value;
    return mmal_status_to_int(status);
}

int FlashCam::setCameraConfig ( MMAL_PARAMETER_CAMERA_CONFIG_T *config ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_set(_camera_component->control, &config->hdr);
    return mmal_status_to_int(status);
}

int FlashCam::getCameraConfig ( MMAL_PARAMETER_CAMERA_CONFIG_T *config ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_get(_camera_component->control, &config->hdr);       
    return mmal_status_to_int(status);
}

int FlashCam::setStabilisation ( int stabilisation ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_set_boolean(_camera_component->control, MMAL_PARAMETER_VIDEO_STABILISATION, stabilisation);   
    if ( status == MMAL_SUCCESS ) _params.stabilisation = stabilisation;
    return mmal_status_to_int(status);
}

int FlashCam::getStabilisation ( int *stabilisation ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_get_boolean(_camera_component->control, MMAL_PARAMETER_VIDEO_STABILISATION, stabilisation);        
    return mmal_status_to_int(status);
}

int FlashCam::setDRC ( MMAL_PARAMETER_DRC_STRENGTH_T  strength ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    MMAL_PARAMETER_DRC_T param  = {{MMAL_PARAMETER_DYNAMIC_RANGE_COMPRESSION, sizeof(param)}, strength};
    MMAL_STATUS_T        status = mmal_port_parameter_set(_camera_component->control, &param.hdr);
    if ( status == MMAL_SUCCESS ) _params.strength = strength;
    return mmal_status_to_int(status);
}

int FlashCam::getDRC ( MMAL_PARAMETER_DRC_STRENGTH_T *strength ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    // MMAL_PARAMETER_DRC_STRENGTH_MAX: just usa a value to allocate `param`
    MMAL_PARAMETER_DRC_T param  = {{MMAL_PARAMETER_DYNAMIC_RANGE_COMPRESSION, sizeof(param)}, MMAL_PARAMETER_DRC_STRENGTH_MAX};
    MMAL_STATUS_T        status = mmal_port_parameter_get(_camera_component->control, &param.hdr);  
    //update value
    *strength = param.strength;
    return mmal_status_to_int(status);
}

int FlashCam::setSharpness ( int  sharpness ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    if ( sharpness < -100 ) sharpness = -100;
    if ( sharpness >  100 ) sharpness =  100;
    MMAL_STATUS_T status = setParameterRational(MMAL_PARAMETER_SHARPNESS, sharpness);
    if ( status == MMAL_SUCCESS ) _params.sharpness = sharpness;
    return mmal_status_to_int(status);
}

int FlashCam::getSharpness ( int *sharpness ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    return getParameterRational( MMAL_PARAMETER_SHARPNESS, sharpness);        
}

int FlashCam::setContrast ( int  contrast ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    if ( contrast < -100 ) contrast = -100;
    if ( contrast >  100 ) contrast =  100;
    MMAL_STATUS_T status = setParameterRational(MMAL_PARAMETER_CONTRAST, contrast);
    if ( status == MMAL_SUCCESS ) _params.contrast = contrast;
    return mmal_status_to_int(status);
}

int FlashCam::getContrast ( int *contrast ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    return getParameterRational( MMAL_PARAMETER_CONTRAST, contrast);        
}

int FlashCam::setBrightness ( int  brightness ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    if ( brightness <    0 ) brightness =    0;
    if ( brightness >  100 ) brightness =  100;
    MMAL_STATUS_T status = setParameterRational(MMAL_PARAMETER_BRIGHTNESS, brightness);
    if ( status == MMAL_SUCCESS ) _params.brightness = brightness;
    return mmal_status_to_int(status);
}

int FlashCam::getBrightness ( int *brightness ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    return getParameterRational( MMAL_PARAMETER_BRIGHTNESS, brightness);        
}

int FlashCam::setSaturation ( int  saturation ) {        
    if (( !_camera_component ) || ( _capturing )) return 1;
    if ( saturation < -100 ) saturation = -100;
    if ( saturation >  100 ) saturation =  100;
    MMAL_STATUS_T status = setParameterRational(MMAL_PARAMETER_SATURATION, saturation);
    if ( status == MMAL_SUCCESS ) _params.saturation = saturation;
    return mmal_status_to_int(status);
}

int FlashCam::getSaturation ( int *saturation ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    return getParameterRational( MMAL_PARAMETER_SATURATION, saturation);        
}

int FlashCam::setISO ( unsigned int  iso ) {        
    if (( !_camera_component ) || ( _capturing )) return 1;
    if ( iso > 1600 ) iso = 1600;
    MMAL_STATUS_T status = mmal_port_parameter_set_uint32(_camera_component->control, MMAL_PARAMETER_ISO, iso); 
    if ( status == MMAL_SUCCESS ) _params.iso = iso;
    return mmal_status_to_int(status);
}

int FlashCam::getISO ( unsigned int *iso ) {        
    if (( !_camera_component ) || ( _capturing )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_get_uint32(_camera_component->control, MMAL_PARAMETER_ISO, iso);        
    return mmal_status_to_int(status);
}

int FlashCam::setShutterSpeed ( unsigned int  speed ) {        
    if (( !_camera_component ) || ( _capturing )) return 1;
    if ( speed > 330000 ) speed = 330000;
    MMAL_STATUS_T status = mmal_port_parameter_set_uint32(_camera_component->control, MMAL_PARAMETER_SHUTTER_SPEED, speed);  
    if ( status == MMAL_SUCCESS ) _params.speed = speed;
    return mmal_status_to_int(status);
}

int FlashCam::getShutterSpeed ( unsigned int *speed ) {        
    if (( !_camera_component ) || ( _capturing )) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_get_uint32(_camera_component->control, MMAL_PARAMETER_SHUTTER_SPEED, speed);        
    return mmal_status_to_int(status);
}

int FlashCam::setAWBGains ( float red , float blue ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
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
    if (!_camera_component) return 1;
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
    if (!_camera_component) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_set_boolean(_camera_component->control, MMAL_PARAMETER_STILLS_DENOISE, denoise);     
    if ( status == MMAL_SUCCESS ) _params.denoise = denoise;
    return mmal_status_to_int(status);
}

int FlashCam::getDenoise ( int *denoise ) {
    if (!_camera_component) return 1;
    MMAL_STATUS_T status = mmal_port_parameter_get_boolean(_camera_component->control, MMAL_PARAMETER_STILLS_DENOISE, denoise);        
    return mmal_status_to_int(status);
}


int FlashCam::setChangeEventRequest ( unsigned int id , int  request ) {
    if (!_camera_component) return 1;    
    MMAL_PARAMETER_CHANGE_EVENT_REQUEST_T param  = {{MMAL_PARAMETER_CHANGE_EVENT_REQUEST, sizeof(param)}, id, request};
    MMAL_STATUS_T                         status = mmal_port_parameter_set(_camera_component->control, &param.hdr);
    return mmal_status_to_int(status);
}

int FlashCam::getChangeEventRequest ( unsigned int id , int *request ) {
    if (( !_camera_component ) || ( _capturing )) return 1;
    // 0: just usa a value to allocate `param`
    MMAL_PARAMETER_CHANGE_EVENT_REQUEST_T param  = {{MMAL_PARAMETER_CHANGE_EVENT_REQUEST, sizeof(param)}, id, 0};
    MMAL_STATUS_T                         status = mmal_port_parameter_get(_camera_component->control, &param.hdr);  
    //update value
    *request = param.enable;
    return mmal_status_to_int(status);
}









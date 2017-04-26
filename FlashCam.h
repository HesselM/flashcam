
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

#ifndef FlashCam_h
#define FlashCam_h

#include <stdio.h>

#include "interface/mmal/mmal.h"
#include "interface/mmal/util/mmal_connection.h"

// Mode of FlashCam: it is either setup to do image-capturing, or it streams at a set fps images.
typedef enum {
    FLASHCAM_MODE_UNKOWN = 0,
    FLASHCAM_MODE_VIDEO,
    FLASHCAM_MODE_CAPTURE
} FLASHCAM_MODE_T;

// Function pointer for callback:
//  - unsigned char *frame  : pointer to frame containing frame data
//  - int width             : width of image
//  - int height            : height of image
typedef void (*FLASHCAM_CALLBACK_T) (unsigned char *, int, int);

/*
 * FLASHCAM_PARAMS_T
 * Tracker of all camera parameters.
 */
typedef struct {
    int rotation;                               // Camera rotation (degrees) : 0 / 90 / 180 / 270;
    MMAL_PARAM_AWBMODE_T awbmode;               // AWB mode. See: MMAL_PARAM_AWBMODE_T;
    MMAL_PARAM_FLASH_T flashmode;               // Flash mode. See: MMAL_PARAM_FLASH_T;
    MMAL_PARAM_MIRROR_T mirror;                 // Image Mirroring. See: MMAL_PARAM_MIRROR_T;
    unsigned int cameranum;                     // Index of used camera. 
    MMAL_PARAM_EXPOSUREMODE_T exposuremode;     // Exposure mode (e.g. night). See: MMAL_PARAM_EXPOSUREMODE_T;
    MMAL_PARAM_EXPOSUREMETERINGMODE_T metering; // Exposure metering. See: MMAL_PARAM_EXPOSUREMETERINGMODE_Tl;
    float framerate;                            // Frame rate (fps):    0.0f to    120.0f
    int stabilisation;                          // Video Stabilisation. On (1) or Off (0);
    MMAL_PARAMETER_DRC_STRENGTH_T drc;          // Dynamic Range Compression. See: MMAL_PARAMETER_DRC_STRENGTH_T;
    int sharpness;                              // Image Sharpness : -100    to    100
    int contrast;                               // Image Contrast  : -100    to    100
    int brightness;                             // Image Brightness:    0    to    100
    int saturation;                             // Image Saturation: -100    to    100
    unsigned int iso;                           // ISO             :    0    to   1600    (NOTE: 800+ might not work; 0=auto)
    unsigned int shutterspeed;                  // Shutterspeed    :    0    to 330000    (microseconds; fps in VideoMode)
    float awbgain_red;                          // AWB gain red    :    0.0f to      8.0f (NOTE: Only used when AWB=OFF)
    float awbgain_blue;                         // AWB gain blue   :    0.0f to      8.0f (NOTE: Only used when AWB=OFF)
    int denoise;                                // Image Denoiseing. On (1) or Off (0);
    
} FLASHCAM_PARAMS_T;

/*
 * FLASHCAM_SETTINGS_T
 * Camera settings. Used when camera is initialized.
 */
typedef struct {
    unsigned int width;                         // Width of image
    unsigned int height;                        // Height of image
    int verbose;                                // Verbose or not?
    int update;                                 // Register for updates from the camera when its internal settings are changed?
    FLASHCAM_MODE_T mode;                       // Capture-mode of camera 
} FLASHCAM_SETTINGS_T;


class FlashCam 
{

    
private:
    
    /*
     * FLASHCAM_PORT_USERDATA_T
     * used internally for communication and status-updates with the camera
     */
    typedef struct {
        FLASHCAM_PARAMS_T   *params;            // Pointer to param set
        FLASHCAM_SETTINGS_T *settings;          // Pointer to setting set
        MMAL_POOL_T         *camera_pool;       // Pool of buffers for camera
        unsigned char       *framebuffer;       // Buffer for final image   
        unsigned int         framebuffer_size;  // Size of buffer
        unsigned int         framebuffer_idx;   // Tracker to stitch imager properly from the camera-callback payloads
        VCOS_SEMAPHORE_T     sem_capture;       // Semaphore indicating the completion of a frame capture
        FLASHCAM_CALLBACK_T  callback;          // Callback to user function
    } FLASHCAM_PORT_USERDATA_T;
    
    //private variables
    bool                        _initialised        = false;    // Camera initialised?
    bool                        _active             = false;    // Camera currently active?
    FLASHCAM_PARAMS_T           _params             = {};
    FLASHCAM_SETTINGS_T         _settings           = {};
    MMAL_COMPONENT_T           *_camera_component   = NULL;
    MMAL_COMPONENT_T           *_preview_component  = NULL;
    MMAL_CONNECTION_T          *_preview_connection = NULL;
    MMAL_POOL_T                *_camera_pool        = NULL;
    unsigned char              *_framebuffer        = NULL;
    FLASHCAM_PORT_USERDATA_T    _userdata           = {};
    
    //Camera setup functions
    int resetCamera();
    int setupComponents();
    MMAL_STATUS_T setupComponentCamera();
    MMAL_STATUS_T setupComponentPreview();
    void destroyComponents();
    
    //callbacks for async image/update retrieval
    static void control_callback( MMAL_PORT_T *port , MMAL_BUFFER_HEADER_T *buffer );
    static void buffer_callback(  MMAL_PORT_T *port , MMAL_BUFFER_HEADER_T *buffer );
    MMAL_STATUS_T connectPorts( MMAL_PORT_T *output_port , MMAL_PORT_T *input_port , MMAL_CONNECTION_T **connection );
    
    //misc
    MMAL_STATUS_T setParameterRational( int id , int  val );
    MMAL_STATUS_T getParameterRational( int id , int *val );
    
public:
    
    // Constructor 
    //  - initialize the camera with default or specified settings
    FlashCam();
    FlashCam(FLASHCAM_SETTINGS_T *settings);
    
    // Destructor
    ~FlashCam();

    // capture image
    int startCapture();
    int stopCapture();
    
    //callback options --> for when a full frame is received
    void setFrameCallback(FLASHCAM_CALLBACK_T callback);
    void resetFrameCallback();
    

    
    /******************************************/
    /*********   GETTERS / SETTERS  ***********/
    /******************************************/
    
    
    /* Library Settings */
    
    // setting utilities
    static void getDefaultSettings(FLASHCAM_SETTINGS_T *settings);
    static void printSettings(FLASHCAM_SETTINGS_T *settings);

    // Set/Get library settings
    // Note: setting implicitly resets/configures camera
    int setSettings(FLASHCAM_SETTINGS_T *settings);
    int getSettings(FLASHCAM_SETTINGS_T *settings);

    int setSettingSize( unsigned int  width, unsigned int  height );
    int getSettingSize( unsigned int *width, unsigned int *height );

    int setSettingVerbose( int  verbose );
    int getSettingVerbose( int *verbose );
    
    int setSettingUpdate( int  update );
    int getSettingUpdate( int *update );
    
    int setSettingCaptureMode( FLASHCAM_MODE_T  mode );
    int getSettingCaptureMode( FLASHCAM_MODE_T *mode );

    
    /*
     * These MMAL_PARAMS_XXXXXX paramter-values are taken from 
     *   - "mmal/mmal_parameters_camera.h"
     *   - "mmal/mmal_parameters_common.h"
     *
     * Note that not all functions are implemented.
     *
     */
    
    // parameter utilities
    static void getDefaultParams(FLASHCAM_PARAMS_T *params);
    static void printParams(FLASHCAM_PARAMS_T *params);
    
    // set/retrieve currently set camera parameters
    int setParams(FLASHCAM_PARAMS_T *params);
    int getParams(FLASHCAM_PARAMS_T *params, bool mem);
    
    /********* mmal/mmal_parameters_camera.h ***********/

    /* 0 */
    //MMAL_PARAMETER_THUMBNAIL_CONFIGURATION    /**< Takes a @ref MMAL_PARAMETER_THUMBNAIL_CONFIG_T */
    //= MMAL_PARAMETER_GROUP_CAMERA,
    //MMAL_PARAMETER_CAPTURE_QUALITY,           /**< Unused? */
    
    /*
     * Property:  MMAL_PARAMETER_ROTATION
     *      < Takes a @ref MMAL_PARAMETER_INT32_T >
     *
     * Note: value in degrees. 
     *
     * -270 / 180 / -90 / 0 / 90 / 180 / 270
     */
    int setRotation ( int  rotation );
    int getRotation ( int *rotation );
    
    //MMAL_PARAMETER_EXIF_DISABLE,              /**< Takes a @ref MMAL_PARAMETER_BOOLEAN_T */
    //MMAL_PARAMETER_EXIF,                      /**< Takes a @ref MMAL_PARAMETER_EXIF_T */
    
    /*
     * Property:  MMAL_PARAMETER_AWB_MODE
     *      < Takes a @ref MMAL_PARAM_AWBMODE_T >
     *
     * MMAL_PARAM_AWBMODE_OFF,
     * MMAL_PARAM_AWBMODE_AUTO,
     * MMAL_PARAM_AWBMODE_SUNLIGHT,
     * MMAL_PARAM_AWBMODE_CLOUDY,
     * MMAL_PARAM_AWBMODE_SHADE,
     * MMAL_PARAM_AWBMODE_TUNGSTEN,
     * MMAL_PARAM_AWBMODE_FLUORESCENT,
     * MMAL_PARAM_AWBMODE_INCANDESCENT,
     * MMAL_PARAM_AWBMODE_FLASH,
     * MMAL_PARAM_AWBMODE_HORIZON,
     * MMAL_PARAM_AWBMODE_MAX = 0x7fffffff
     */
    int setAWBMode ( MMAL_PARAM_AWBMODE_T  awb );
    int getAWBMode ( MMAL_PARAM_AWBMODE_T *awb );
    
    //MMAL_PARAMETER_IMAGE_EFFECT,              /**< Takes a @ref MMAL_PARAMETER_IMAGEFX_T */
    //MMAL_PARAMETER_COLOUR_EFFECT,             /**< Takes a @ref MMAL_PARAMETER_COLOURFX_T */
    //MMAL_PARAMETER_FLICKER_AVOID,             /**< Takes a @ref MMAL_PARAMETER_FLICKERAVOID_T */
    
    /*
     * Property:  MMAL_PARAMETER_FLASH
     *      < Takes a @ref MMAL_PARAMETER_FLASH_T >
     *
     * MMAL_PARAM_FLASH_OFF,
     * MMAL_PARAM_FLASH_AUTO,
     * MMAL_PARAM_FLASH_ON,
     * MMAL_PARAM_FLASH_REDEYE,
     * MMAL_PARAM_FLASH_FILLIN,
     * MMAL_PARAM_FLASH_TORCH,
     * MMAL_PARAM_FLASH_MAX = 0x7FFFFFFF
     */
    int setFlashMode ( MMAL_PARAM_FLASH_T  flash );
    int getFlashMode ( MMAL_PARAM_FLASH_T *flash );
    
    //MMAL_PARAMETER_REDEYE,                    /**< Takes a @ref MMAL_PARAMETER_REDEYE_T */
    //MMAL_PARAMETER_FOCUS,                     /**< Takes a @ref MMAL_PARAMETER_FOCUS_T */
    //MMAL_PARAMETER_FOCAL_LENGTHS,             /**< Unused? */
    //MMAL_PARAMETER_EXPOSURE_COMP,             /**< Takes a @ref MMAL_PARAMETER_INT32_T or MMAL_PARAMETER_RATIONAL_T */
    //MMAL_PARAMETER_ZOOM,                      /**< Takes a @ref MMAL_PARAMETER_SCALEFACTOR_T */
    
    /*
     * Property:  MMAL_PARAMETER_MIRROR
     *      < Takes a @ref MMAL_PARAMETER_MIRROR_T >
     *
     * MMAL_PARAM_MIRROR_NONE,
     * MMAL_PARAM_MIRROR_VERTICAL,
     * MMAL_PARAM_MIRROR_HORIZONTAL,
     * MMAL_PARAM_MIRROR_BOTH,
     */
    int setMirror ( MMAL_PARAM_MIRROR_T  mirror );
    int getMirror ( MMAL_PARAM_MIRROR_T *mirror );
    
    /* 0x10 */

    /*
     * Property:  MMAL_PARAMETER_CAMERA_NUM
     *      < Takes a @ref MMAL_PARAMETER_UINT32_T >
     */
    int setCameraNum( unsigned int  num );
    int getCameraNum( unsigned int *num );
    
    /*
     * Property:  MMAL_PARAMETER_CAPTURE
     *      < Takes a @ref MMAL_PARAMETER_BOOLEAN_T >
     *
     * Note: This function is called by capture(). 
     *       Do not call setCapture(1) directly without the proper settings!
     *
     * 1 or 0
     */
    int setCapture( MMAL_PORT_T *port, int  capture );
    int getCapture( MMAL_PORT_T *port, int *capture );
    
    /*
     * Property: MMAL_PARAMETER_EXPOSURE_MODE
     *      < Takes a @ref MMAL_PARAMETER_EXPOSUREMODE_T >
     *
     * MMAL_PARAM_EXPOSUREMODE_OFF,
     * MMAL_PARAM_EXPOSUREMODE_AUTO,
     * MMAL_PARAM_EXPOSUREMODE_NIGHT,
     * MMAL_PARAM_EXPOSUREMODE_NIGHTPREVIEW,
     * MMAL_PARAM_EXPOSUREMODE_BACKLIGHT,
     * MMAL_PARAM_EXPOSUREMODE_SPOTLIGHT,
     * MMAL_PARAM_EXPOSUREMODE_SPORTS,
     * MMAL_PARAM_EXPOSUREMODE_SNOW,
     * MMAL_PARAM_EXPOSUREMODE_BEACH,
     * MMAL_PARAM_EXPOSUREMODE_VERYLONG,
     * MMAL_PARAM_EXPOSUREMODE_FIXEDFPS,
     * MMAL_PARAM_EXPOSUREMODE_ANTISHAKE,
     * MMAL_PARAM_EXPOSUREMODE_FIREWORKS,
     * MMAL_PARAM_EXPOSUREMODE_MAX = 0x7fffffff
     */
    int setExposureMode ( MMAL_PARAM_EXPOSUREMODE_T  exposure );
    int getExposureMode ( MMAL_PARAM_EXPOSUREMODE_T *exposure );
    
    /* 
     * Property: MMAL_PARAMETER_EXP_METERING_MODE
     *      < Takes a @ref MMAL_PARAMETER_EXPOSUREMETERINGMODE_T >
     *
     * MMAL_PARAM_EXPOSUREMETERINGMODE_AVERAGE,
     * MMAL_PARAM_EXPOSUREMETERINGMODE_SPOT,
     * MMAL_PARAM_EXPOSUREMETERINGMODE_BACKLIT,
     * MMAL_PARAM_EXPOSUREMETERINGMODE_MATRIX,
     * MMAL_PARAM_EXPOSUREMETERINGMODE_MAX = 0x7fffffff
     */
    int setMeteringMode ( MMAL_PARAM_EXPOSUREMETERINGMODE_T  metering );
    int getMeteringMode ( MMAL_PARAM_EXPOSUREMETERINGMODE_T *metering );
    
    //MMAL_PARAMETER_FOCUS_STATUS,              /**< Takes a @ref MMAL_PARAMETER_FOCUS_STATUS_T */
    
    /* 
     * Property: MMAL_PARAMETER_CAMERA_CONFIG
     *      < Takes a @ref MMAL_PARAMETER_CAMERA_CONFIG_T >
     */
    int setCameraConfig ( MMAL_PARAMETER_CAMERA_CONFIG_T *config );
    int getCameraConfig ( MMAL_PARAMETER_CAMERA_CONFIG_T *config );
    
    //MMAL_PARAMETER_CAPTURE_STATUS,            /**< Takes a @ref MMAL_PARAMETER_CAPTURE_STATUS_T */
    //MMAL_PARAMETER_FACE_TRACK,                /**< Takes a @ref MMAL_PARAMETER_FACE_TRACK_T */
    //MMAL_PARAMETER_DRAW_BOX_FACES_AND_FOCUS,  /**< Takes a @ref MMAL_PARAMETER_BOOLEAN_T */
    //MMAL_PARAMETER_JPEG_Q_FACTOR,             /**< Takes a @ref MMAL_PARAMETER_UINT32_T */
    
    /* 
     * Property: MMAL_PARAMETER_FRAME_RATE
     *      < Takes a @ref MMAL_PARAMETER_FRAME_RATE_T >
     *
     * Note: Transformed into rational with base 256
     *
     * 0 to 120
     */
    int setFrameRate ( float  fps );
    int getFrameRate ( float *fps );
    
    
    //MMAL_PARAMETER_USE_STC,                   /**< Takes a @ref MMAL_PARAMETER_CAMERA_STC_MODE_T */
    //MMAL_PARAMETER_CAMERA_INFO,               /**< Takes a @ref MMAL_PARAMETER_CAMERA_INFO_T */
    
    /* 
     * Property: MMAL_PARAMETER_VIDEO_STABILISATION
     *      < Takes a @ref MMAL_PARAMETER_BOOLEAN_T >
     *
     * 1 or 0
     */
    int setStabilisation ( int  stabilisation );
    int getStabilisation ( int *stabilisation );
    
    //MMAL_PARAMETER_FACE_TRACK_RESULTS,        /**< Takes a @ref MMAL_PARAMETER_FACE_TRACK_RESULTS_T */
    //MMAL_PARAMETER_ENABLE_RAW_CAPTURE,        /**< Takes a @ref MMAL_PARAMETER_BOOLEAN_T */
    
    /* 0x20 */
    //MMAL_PARAMETER_DPF_FILE,                  /**< Takes a @ref MMAL_PARAMETER_URI_T */
    //MMAL_PARAMETER_ENABLE_DPF_FILE,           /**< Takes a @ref MMAL_PARAMETER_BOOLEAN_T */
    //MMAL_PARAMETER_DPF_FAIL_IS_FATAL,         /**< Takes a @ref MMAL_PARAMETER_BOOLEAN_T */
    //MMAL_PARAMETER_CAPTURE_MODE,              /**< Takes a @ref MMAL_PARAMETER_CAPTUREMODE_T */
    //MMAL_PARAMETER_FOCUS_REGIONS,             /**< Takes a @ref MMAL_PARAMETER_FOCUS_REGIONS_T */
    //MMAL_PARAMETER_INPUT_CROP,                /**< Takes a @ref MMAL_PARAMETER_INPUT_CROP_T */    
    //MMAL_PARAMETER_SENSOR_INFORMATION,        /**< Takes a @ref MMAL_PARAMETER_SENSOR_INFORMATION_T */
    //MMAL_PARAMETER_FLASH_SELECT,              /**< Takes a @ref MMAL_PARAMETER_FLASH_SELECT_T */
    //MMAL_PARAMETER_FIELD_OF_VIEW,             /**< Takes a @ref MMAL_PARAMETER_FIELD_OF_VIEW_T */
    //MMAL_PARAMETER_HIGH_DYNAMIC_RANGE,        /**< Takes a @ref MMAL_PARAMETER_BOOLEAN_T */
    
    /* 
     * Property: MMAL_PARAMETER_DYNAMIC_RANGE_COMPRESSION
     *      < Takes a @ref MMAL_PARAMETER_DRC_T >
     * 
     * Note: Enabling DRC will `override fixed white balance`_ gains (set via :attr:`awb_gains` and :attr:`awb_mode`).
     * 
     * MMAL_PARAMETER_DRC_STRENGTH_OFF,
     * MMAL_PARAMETER_DRC_STRENGTH_LOW,
     * MMAL_PARAMETER_DRC_STRENGTH_MEDIUM,
     * MMAL_PARAMETER_DRC_STRENGTH_HIGH,
     * MMAL_PARAMETER_DRC_STRENGTH_MAX = 0x7fffffff
     */
    int setDRC ( MMAL_PARAMETER_DRC_STRENGTH_T  strength );
    int getDRC ( MMAL_PARAMETER_DRC_STRENGTH_T *strength );
    
    //MMAL_PARAMETER_ALGORITHM_CONTROL,         /**< Takes a @ref MMAL_PARAMETER_ALGORITHM_CONTROL_T */
    
    /* 
     * Property: MMAL_PARAMETER_SHARPNESS
     *      < Takes a @ref MMAL_PARAMETER_RATIONAL_T >
     *
     * Note: Transformed into rational with base 100
     *
     * -100 to 100
     */
    int setSharpness ( int  sharpness );                       
    int getSharpness ( int *sharpness );
    
    /* 
     * Property: MMAL_PARAMETER_CONTRAST
     *      < Takes a @ref MMAL_PARAMETER_RATIONAL_T >
     *
     * Note: Transformed into rational with base 100
     *
     * -100 to 100
     */
    int setContrast ( int  contrast ); 
    int getContrast ( int *contrast );
    
    /* 
     * Property: MMAL_PARAMETER_BRIGHTNESS
     *      < Takes a @ref MMAL_PARAMETER_RATIONAL_T >
     *
     * Note: Transformed into rational with base 100
     *
     *   0 to 100
     */
    int setBrightness ( int  brightness );    
    int getBrightness ( int *brightness );
    
    /* 
     * Property: MMAL_PARAMETER_SATURATION
     *      < Takes a @ref MMAL_PARAMETER_RATIONAL_T >
     *
     * Note: Transformed into rational with base 100
     *
     * -100 to 100
     */
    int setSaturation ( int  saturation ); 
    int getSaturation ( int *saturation );
    
    /* 
     * Property: MMAL_PARAMETER_ISO
     *      < Takes a @ref MMAL_PARAMETER_UINT32_T >
     *
     * Note: 800+ possibly not supported
     *
     *   0 to 1600
     */
    int setISO ( unsigned int  iso );
    int getISO ( unsigned int *iso );
    
    //MMAL_PARAMETER_ANTISHAKE,                 /**< Takes a @ref MMAL_PARAMETER_BOOLEAN_T */
    //MMAL_PARAMETER_IMAGE_EFFECT_PARAMETERS,   /**< Takes a @ref MMAL_PARAMETER_IMAGEFX_PARAMETERS_T */
    //MMAL_PARAMETER_CAMERA_BURST_CAPTURE,      /**< Takes a @ref MMAL_PARAMETER_BOOLEAN_T */
    //MMAL_PARAMETER_CAMERA_MIN_ISO,            /**< Takes a @ref MMAL_PARAMETER_UINT32_T */
    //MMAL_PARAMETER_CAMERA_USE_CASE,           /**< Takes a @ref MMAL_PARAMETER_CAMERA_USE_CASE_T */
    //MMAL_PARAMETER_CAPTURE_STATS_PASS,        /**< Takes a @ref MMAL_PARAMETER_BOOLEAN_T */
    //MMAL_PARAMETER_CAMERA_CUSTOM_SENSOR_CONFIG, /**< Takes a @ref MMAL_PARAMETER_UINT32_T */
    //MMAL_PARAMETER_ENABLE_REGISTER_FILE,      /**< Takes a @ref MMAL_PARAMETER_BOOLEAN_T */
    //MMAL_PARAMETER_REGISTER_FAIL_IS_FATAL,    /**< Takes a @ref MMAL_PARAMETER_BOOLEAN_T */
    //MMAL_PARAMETER_CONFIGFILE_REGISTERS,      /**< Takes a @ref MMAL_PARAMETER_CONFIGFILE_T */
    //MMAL_PARAMETER_CONFIGFILE_CHUNK_REGISTERS,/**< Takes a @ref MMAL_PARAMETER_CONFIGFILE_CHUNK_T */
    //MMAL_PARAMETER_JPEG_ATTACH_LOG,           /**< Takes a @ref MMAL_PARAMETER_BOOLEAN_T */
    //MMAL_PARAMETER_ZERO_SHUTTER_LAG,          /**< Takes a @ref MMAL_PARAMETER_ZEROSHUTTERLAG_T */
    //MMAL_PARAMETER_FPS_RANGE,                 /**< Takes a @ref MMAL_PARAMETER_FPS_RANGE_T */
    
    /*
     TODO: when using large FPS-values, update range accordingly:
            Note: both preview & capture port need to be updated
     
            if(state->camera_parameters.speed > 6000000) {
                MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
                    { 50, 1000 }, {166, 1000}};
                mmal_port_parameter_set(preview_port, &fps_range.hdr);
            } else if(state->camera_parameters.speed > 1000000) {
                MMAL_PARAMETER_FPS_RANGE_T fps_range = {{MMAL_PARAMETER_FPS_RANGE, sizeof(fps_range)},
                    { 166, 1000 }, {999, 1000}};
                mmal_port_parameter_set(preview_port, &fps_range.hdr);
            }
    */
    
    //MMAL_PARAMETER_CAPTURE_EXPOSURE_COMP,     /**< Takes a @ref MMAL_PARAMETER_INT32_T */
    
    /* 0x40 */
    //MMAL_PARAMETER_SW_SHARPEN_DISABLE,        /**< Takes a @ref MMAL_PARAMETER_BOOLEAN_T */
    //MMAL_PARAMETER_FLASH_REQUIRED,            /**< Takes a @ref MMAL_PARAMETER_BOOLEAN_T */
    //MMAL_PARAMETER_SW_SATURATION_DISABLE,     /**< Takes a @ref MMAL_PARAMETER_BOOLEAN_T */
    
    /* 
     * Property: MMAL_PARAMETER_SHUTTER_SPEED
     *      < Takes a @ref MMAL_PARAMETER_UINT32_T >
     *
     * Note: value in microseconds
     *
     *   0 to 330000
     */
    int setShutterSpeed ( unsigned int  speed );
    int getShutterSpeed ( unsigned int *speed );
    
    /* 
     * Property: MMAL_PARAMETER_CUSTOM_AWB_GAINS
     *      < Takes a @ref MMAL_PARAMETER_AWB_GAINS_T >
     *
     * Note: Transformed into rational with base 65536
     *       Only used when AWB=OFF
     *
     *  0.0 to 8.0
     */
    int setAWBGains ( float  red , float  blue );
    int getAWBGains ( float *red , float *blue );
    
    //MMAL_PARAMETER_CAMERA_SETTINGS,           /**< Takes a @ref MMAL_PARAMETER_CAMERA_SETTINGS_T */
    //MMAL_PARAMETER_PRIVACY_INDICATOR,         /**< Takes a @ref MMAL_PARAMETER_PRIVACY_INDICATOR_T */
    //MMAL_PARAMETER_VIDEO_DENOISE,             /**< Takes a @ref MMAL_PARAMETER_BOOLEAN_T */
    
    /* 
     * Property: MMAL_PARAMETER_STILLS_DENOISE
     *      < Takes a @ref MMAL_PARAMETER_BOOLEAN_T >
     *
     * 1 or 0
     */
    int setDenoise ( int  denoise );
    int getDenoise ( int *denoise );
    
    //MMAL_PARAMETER_ANNOTATE,                  /**< Takes a @ref MMAL_PARAMETER_CAMERA_ANNOTATE_T */
    //MMAL_PARAMETER_STEREOSCOPIC_MODE,         /**< Takes a @ref MMAL_PARAMETER_STEREOSCOPIC_MODE_T */
    
    /********* mmal/parameters_common.h ***********/
     
    //MMAL_PARAMETER_UNUSED                  /**< Never a valid parameter ID */
    // = MMAL_PARAMETER_GROUP_COMMON,
    // MMAL_PARAMETER_SUPPORTED_ENCODINGS,    /**< Takes a MMAL_PARAMETER_ENCODING_T */
    // MMAL_PARAMETER_URI,                    /**< Takes a MMAL_PARAMETER_URI_T */
    
    /* 
     * Property: MMAL_PARAMETER_CHANGE_EVENT_REQUEST
     *      < Takes a @ref MMAL_PARAMETER_CHANGE_EVENT_REQUEST_T >
     *
     * id = MMAL_PARAMETER_XXX id, request = 1 or 0
     */
    int setChangeEventRequest ( unsigned int id , int  request );
    int getChangeEventRequest ( unsigned int id , int *request );
    
    // MMAL_PARAMETER_ZERO_COPY,              /**< Takes a MMAL_PARAMETER_BOOLEAN_T */
    // MMAL_PARAMETER_BUFFER_REQUIREMENTS,    /**< Takes a MMAL_PARAMETER_BUFFER_REQUIREMENTS_T */
    // MMAL_PARAMETER_STATISTICS,             /**< Takes a MMAL_PARAMETER_STATISTICS_T */
    // MMAL_PARAMETER_CORE_STATISTICS,        /**< Takes a MMAL_PARAMETER_CORE_STATISTICS_T */
    // MMAL_PARAMETER_MEM_USAGE,              /**< Takes a MMAL_PARAMETER_MEM_USAGE_T */
    // MMAL_PARAMETER_BUFFER_FLAG_FILTER,     /**< Takes a MMAL_PARAMETER_UINT32_T */
    // MMAL_PARAMETER_SEEK,                   /**< Takes a MMAL_PARAMETER_SEEK_T */
    // MMAL_PARAMETER_POWERMON_ENABLE,        /**< Takes a MMAL_PARAMETER_BOOLEAN_T */
    // MMAL_PARAMETER_LOGGING,                /**< Takes a MMAL_PARAMETER_LOGGING_T */
    // MMAL_PARAMETER_SYSTEM_TIME,            /**< Takes a MMAL_PARAMETER_UINT64_T */
    // MMAL_PARAMETER_NO_IMAGE_PADDING,       /**< Takes a MMAL_PARAMETER_BOOLEAN_T */
    // MMAL_PARAMETER_LOCKSTEP_ENABLE         /**< Takes a MMAL_PARAMETER_BOOLEAN_T */
    
    
    
    
    
};

#endif /* FlashCam_h */

/*******************************************************/
/********* DEFAULT SETTING OF DIFFERENT LIBS ***********/
/*******************************************************/

/* 
 Python:PiCamera
 
 def _init_defaults(self):
 self.sharpness = 0
 self.contrast = 0
 self.brightness = 50
 self.saturation = 0
 self.iso = 0 # auto
 self.video_stabilization = False
 self.exposure_compensation = 0
 self.exposure_mode = 'auto'
 self.meter_mode = 'average'
 self.awb_mode = 'auto'
 self.image_effect = 'none'
 self.color_effects = None
 self.rotation = 0
 self.hflip = self.vflip = False
 self.zoom = (0.0, 0.0, 1.0, 1.0)
 */

/*
 CAMAVA:private_still
 
 width = 640;
 height = 480;
 encoding = CAMAVA_ENCODING_BMP;
 encoder = NULL;
 encoder_connection = NULL;
 sharpness = 0;
 contrast = 0;
 brightness = 50;
 quality = 85;
 saturation = 0;
 iso = 400;
 //videoStabilisation = 0;
 //exposureCompensation = 0;
 exposure = CAMAVA_EXPOSURE_AUTO;
 metering = CAMAVA_METERING_AVERAGE;
 awb = CAMAVA_AWB_AUTO;
 imageEffect = CAMAVA_IMAGE_EFFECT_NONE;
 //colourEffects.enable = 0;
 //colourEffects.u = 128;
 //colourEffects.v = 128;
 rotation = 0;
 changedSettings = true;
 horizontalFlip = false;
 verticalFlip = false;
 //roi.x = params->roi.y = 0.0;
 //roi.w = params->roi.h = 1.0;
 */

/*
 
 CAMAVA:private_still
 
 State.framerate 		= 10;
 State.width 			= 1280;      // use a multiple of 320 (640, 1280)
 State.height 			= 960;		// use a multiple of 240 (480, 960)
 State.sharpness = 0;
 State.contrast = 0;
 State.brightness = 50;
 State.saturation = 0;
 State.ISO = 400;
 State.videoStabilisation = false;
 State.exposureCompensation = 0;
 State.captureFtm=CAMAVA_FORMAT_RGB;
 State.rpc_exposureMode = CAMAVA_EXPOSURE_AUTO;
 State.rpc_exposureMeterMode = CAMAVA_METERING_AVERAGE;
 State.rpc_awbMode = CAMAVA_AWB_AUTO;
 State.rpc_imageEffect = CAMAVA_IMAGE_EFFECT_NONE;
 State.colourEffects.enable = 0;
 State.colourEffects.u = 128;
 State.colourEffects.v = 128;
 State.rotation = 0;
 State.hflip = State.vflip = 0;
 State.roi.x = State.roi.y = 0.0;
 State.roi.w = State.roi.h = 1.0;
 State.shutterSpeed=0;//auto
 State.awbg_red=1.0;
 State.awbg_blue=1.0;
 State.sensor_mode = 0; //do not set mode by default
 */

/*      
 USERLAND 
 
 params->sharpness = 0;
 params->contrast = 0;
 params->brightness = 50;
 params->saturation = 0;
 params->ISO = 0;                    // 0 = auto
 params->videoStabilisation = 0;
 params->exposureCompensation = 0;
 params->exposureMode = MMAL_PARAM_EXPOSUREMODE_AUTO;
 params->exposureMeterMode = MMAL_PARAM_EXPOSUREMETERINGMODE_AVERAGE;
 params->awbMode = MMAL_PARAM_AWBMODE_AUTO;
 params->imageEffect = MMAL_PARAM_IMAGEFX_NONE;
 params->colourEffects.enable = 0;
 params->colourEffects.u = 128;
 params->colourEffects.v = 128;
 params->rotation = 0;
 params->hflip = params->vflip = 0;
 params->roi.x = params->roi.y = 0.0;
 params->roi.w = params->roi.h = 1.0;
 params->shutter_speed = 0;          // 0 = auto
 params->awb_gains_r = 0;      // Only have any function if AWB OFF is used.
 params->awb_gains_b = 0;
 params->drc_level = MMAL_PARAMETER_DRC_STRENGTH_OFF;
 params->stats_pass = MMAL_FALSE;
 params->enable_annotate = 0;
 params->annotate_string[0] = '\0';
 params->annotate_text_size = 0;	//Use firmware default
 params->annotate_text_colour = -1;   //Use firmware default
 params->annotate_bg_colour = -1;     //Use firmware default
 params->stereo_mode.mode = MMAL_STEREOSCOPIC_MODE_NONE;
 params->stereo_mode.decimate = MMAL_FALSE;
 params->stereo_mode.swap_eyes = MMAL_FALSE;
 */

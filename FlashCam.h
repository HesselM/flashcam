
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

//#include "interface/vcos/vcos.h"

//#include "interface/vmcs_host/vc_vchi_gencmd.h"
//#include "interface/mmal/mmal_logging.h"
//#include "interface/mmal/util/mmal_util.h"
//#include "interface/mmal/util/mmal_util_params.h"
//#include "interface/mmal/util/mmal_default_components.h"


#include <stdio.h>

extern "C" {
#include "interface/mmal/mmal.h"
#include "types.h"
}

//namespace flashcam {
    
    typedef struct flashcam_params_s {
        int rotation;                               // Camera rotation (degrees) : -270 / -180 / -90 / 0 / 90 / 180 / 270;
        MMAL_PARAM_AWBMODE_T awb;                   // AWB mode. See: MMAL_PARAM_AWBMODE_T;
        MMAL_PARAM_FLASH_T flash;                   // Flash mode. See: MMAL_PARAM_FLASH_T;
        MMAL_PARAM_MIRROR_T mirror;                 // Image Mirroring. See: MMAL_PARAM_MIRROR_T;
        MMAL_PARAM_EXPOSUREMODE_T exposure;         // Exposure mode (e.g. night). See: MMAL_PARAM_EXPOSUREMODE_T;
        MMAL_PARAM_EXPOSUREMETERINGMODE_T metering; // Exposure metering. See: MMAL_PARAM_EXPOSUREMETERINGMODE_Tl;
        int stabilisation;                          // Video Stabilisation. On (1) or Off (0);
        MMAL_PARAMETER_DRC_STRENGTH_T strength;     // Dynamic Range Compression. See: MMAL_PARAMETER_DRC_STRENGTH_T
        int sharpness;                              // Image Sharpness : -100    to    100
        int contrast;                               // Image Contrast  : -100    to    100
        int brightness;                             // Image Brightness:    0    to    100
        int saturation;                             // Image Saturation: -100    to    100
        unsigned int iso;                           // ISO             :    0    to   1600    (NOTE: 800+ might not work)
        unsigned int speed;                         // Shutterspeed    :    0    to 330000    (microseconds)
        float awbgain_red;                          // AWB gain red    :    0.0f to      8.0f (NOTE: Only used when AWB=OFF)
        float awbgain_blue;                         // AWB gain blue   :    0.0f to      8.0f (NOTE: Only used when AWB=OFF)
        int denoise;                                // Image Denoiseing. On (1) or Off (0);
    } FLASHCAM_PARAMS_T;
    
    
    class FlashCam 
    {
        
    private:
        MMAL_COMPONENT_T *camera;
        
        int setParameterRational( int id , int  val );
        int getParameterRational( int id , int *val );
        
        
    public:
        
        /**Constructor
         */
        FlashCam();
        /**Destructor
         */
        ~FlashCam();
        
        //void commitSettings();
        //void loadSettings();
        
            
        void setCamera(MMAL_COMPONENT_T *camera);
        int setAllParams(FLASHCAM_PARAMS_T *params);
        int getAllParams(FLASHCAM_PARAMS_T *params);
        void printParams(FLASHCAM_PARAMS_T *params);
        
        /*
        
    TODO:
        -   params->imageEffect = MMAL_PARAM_IMAGEFX_NONE;
        -    params->exposureCompensation = 0;
        -    params->imageEffect = MMAL_PARAM_IMAGEFX_NONE;
        -    params->colourEffects.enable = 0;
        -   params->colourEffects.u = 128;
       -  params->colourEffects.v = 128;
        -    params->stats_pass = MMAL_FALSE;

        */
        
        
        
        
/*** MMAL_PARAMS ARE FROM "mmal/mmal_parameters_camera.h" in Userland. ***/
        
        /* 0 */
        //MMAL_PARAMETER_THUMBNAIL_CONFIGURATION    /**< Takes a @ref MMAL_PARAMETER_THUMBNAIL_CONFIG_T */
        //= MMAL_PARAMETER_GROUP_CAMERA,
        //MMAL_PARAMETER_CAPTURE_QUALITY,           /**< Unused? */
        // ---> not used/supported/directly controllable in this lib

        //MMAL_PARAMETER_ROTATION,                  /**< Takes a @ref MMAL_PARAMETER_INT32_T */
        // -270 / 180 / -90 / 0 / 90 / 180 / 270
        /*    
            int my_rotation = ((rotation % 360 ) / 90) * 90;
         */
        int setRotation ( int  rotation );
        int getRotation ( int *rotation );
        
        //MMAL_PARAMETER_EXIF_DISABLE,              /**< Takes a @ref MMAL_PARAMETER_BOOLEAN_T */
        //MMAL_PARAMETER_EXIF,                      /**< Takes a @ref MMAL_PARAMETER_EXIF_T */
        // ---> not used/supported/directly controllable in this lib

        //MMAL_PARAMETER_AWB_MODE,                  /**< Takes a @ref MMAL_PARAM_AWBMODE_T */
        /*
            MMAL_PARAM_AWBMODE_OFF,
            MMAL_PARAM_AWBMODE_AUTO,
            MMAL_PARAM_AWBMODE_SUNLIGHT,
            MMAL_PARAM_AWBMODE_CLOUDY,
            MMAL_PARAM_AWBMODE_SHADE,
            MMAL_PARAM_AWBMODE_TUNGSTEN,
            MMAL_PARAM_AWBMODE_FLUORESCENT,
            MMAL_PARAM_AWBMODE_INCANDESCENT,
            MMAL_PARAM_AWBMODE_FLASH,
            MMAL_PARAM_AWBMODE_HORIZON,
            MMAL_PARAM_AWBMODE_MAX = 0x7fffffff
        */
        int setAWBMode ( MMAL_PARAM_AWBMODE_T  awb );
        int getAWBMode ( MMAL_PARAM_AWBMODE_T *awb );

        //MMAL_PARAMETER_IMAGE_EFFECT,              /**< Takes a @ref MMAL_PARAMETER_IMAGEFX_T */
        //MMAL_PARAMETER_COLOUR_EFFECT,             /**< Takes a @ref MMAL_PARAMETER_COLOURFX_T */
        //MMAL_PARAMETER_FLICKER_AVOID,             /**< Takes a @ref MMAL_PARAMETER_FLICKERAVOID_T */
        // ---> not used/supported/directly controllable in this lib
        
        //MMAL_PARAMETER_FLASH,                     /**< Takes a @ref MMAL_PARAMETER_FLASH_T */
        /*
            MMAL_PARAM_FLASH_OFF,
            MMAL_PARAM_FLASH_AUTO,
            MMAL_PARAM_FLASH_ON,
            MMAL_PARAM_FLASH_REDEYE,
            MMAL_PARAM_FLASH_FILLIN,
            MMAL_PARAM_FLASH_TORCH,
            MMAL_PARAM_FLASH_MAX = 0x7FFFFFFF
        */
        int setFlashMode ( MMAL_PARAM_FLASH_T  flash );
        int getFlashMode ( MMAL_PARAM_FLASH_T *flash );

        //MMAL_PARAMETER_REDEYE,                    /**< Takes a @ref MMAL_PARAMETER_REDEYE_T */
        //MMAL_PARAMETER_FOCUS,                     /**< Takes a @ref MMAL_PARAMETER_FOCUS_T */
        //MMAL_PARAMETER_FOCAL_LENGTHS,             /**< Unused? */
        //MMAL_PARAMETER_EXPOSURE_COMP,             /**< Takes a @ref MMAL_PARAMETER_INT32_T or MMAL_PARAMETER_RATIONAL_T */
        //MMAL_PARAMETER_ZOOM,                      /**< Takes a @ref MMAL_PARAMETER_SCALEFACTOR_T */
        // ---> not used/supported/directly controllable in this lib

        //MMAL_PARAMETER_MIRROR,                    /**< Takes a @ref MMAL_PARAMETER_MIRROR_T */
        /*
            MMAL_PARAM_MIRROR_NONE,
            MMAL_PARAM_MIRROR_VERTICAL,
            MMAL_PARAM_MIRROR_HORIZONTAL,
            MMAL_PARAM_MIRROR_BOTH,
        */
        int setMirror ( MMAL_PARAM_MIRROR_T  mirror );
        int getMirror ( MMAL_PARAM_MIRROR_T *mirror );

        /* 0x10 */
        //MMAL_PARAMETER_CAMERA_NUM,                /**< Takes a @ref MMAL_PARAMETER_UINT32_T */
        //MMAL_PARAMETER_CAPTURE,                   /**< Takes a @ref MMAL_PARAMETER_BOOLEAN_T */
        // ---> not used/supported/directly controllable in this lib

        //MMAL_PARAMETER_EXPOSURE_MODE,             /**< Takes a @ref MMAL_PARAMETER_EXPOSUREMODE_T */
        /*
            MMAL_PARAM_EXPOSUREMODE_OFF,
            MMAL_PARAM_EXPOSUREMODE_AUTO,
            MMAL_PARAM_EXPOSUREMODE_NIGHT,
            MMAL_PARAM_EXPOSUREMODE_NIGHTPREVIEW,
            MMAL_PARAM_EXPOSUREMODE_BACKLIGHT,
            MMAL_PARAM_EXPOSUREMODE_SPOTLIGHT,
            MMAL_PARAM_EXPOSUREMODE_SPORTS,
            MMAL_PARAM_EXPOSUREMODE_SNOW,
            MMAL_PARAM_EXPOSUREMODE_BEACH,
            MMAL_PARAM_EXPOSUREMODE_VERYLONG,
            MMAL_PARAM_EXPOSUREMODE_FIXEDFPS,
            MMAL_PARAM_EXPOSUREMODE_ANTISHAKE,
            MMAL_PARAM_EXPOSUREMODE_FIREWORKS,
            MMAL_PARAM_EXPOSUREMODE_MAX = 0x7fffffff
         */
        int setExposureMode ( MMAL_PARAM_EXPOSUREMODE_T  exposure );
        int getExposureMode ( MMAL_PARAM_EXPOSUREMODE_T *exposure );

        //MMAL_PARAMETER_EXP_METERING_MODE,         /**< Takes a @ref MMAL_PARAMETER_EXPOSUREMETERINGMODE_T */
        /*
            MMAL_PARAM_EXPOSUREMETERINGMODE_AVERAGE,
            MMAL_PARAM_EXPOSUREMETERINGMODE_SPOT,
            MMAL_PARAM_EXPOSUREMETERINGMODE_BACKLIT,
            MMAL_PARAM_EXPOSUREMETERINGMODE_MATRIX,
            MMAL_PARAM_EXPOSUREMETERINGMODE_MAX = 0x7fffffff
        */
        int setMeteringMode ( MMAL_PARAM_EXPOSUREMETERINGMODE_T  metering );
        int getMeteringMode ( MMAL_PARAM_EXPOSUREMETERINGMODE_T *metering );

        //MMAL_PARAMETER_FOCUS_STATUS,              /**< Takes a @ref MMAL_PARAMETER_FOCUS_STATUS_T */
        //MMAL_PARAMETER_CAMERA_CONFIG,             /**< Takes a @ref MMAL_PARAMETER_CAMERA_CONFIG_T */
        //MMAL_PARAMETER_CAPTURE_STATUS,            /**< Takes a @ref MMAL_PARAMETER_CAPTURE_STATUS_T */
        //MMAL_PARAMETER_FACE_TRACK,                /**< Takes a @ref MMAL_PARAMETER_FACE_TRACK_T */
        //MMAL_PARAMETER_DRAW_BOX_FACES_AND_FOCUS,  /**< Takes a @ref MMAL_PARAMETER_BOOLEAN_T */
        //MMAL_PARAMETER_JPEG_Q_FACTOR,             /**< Takes a @ref MMAL_PARAMETER_UINT32_T */
        //MMAL_PARAMETER_FRAME_RATE,                /**< Takes a @ref MMAL_PARAMETER_FRAME_RATE_T */
        //MMAL_PARAMETER_USE_STC,                   /**< Takes a @ref MMAL_PARAMETER_CAMERA_STC_MODE_T */
        //MMAL_PARAMETER_CAMERA_INFO,               /**< Takes a @ref MMAL_PARAMETER_CAMERA_INFO_T */
        // ---> not used/supported/directly controllable in this lib
        
        //MMAL_PARAMETER_VIDEO_STABILISATION,       /**< Takes a @ref MMAL_PARAMETER_BOOLEAN_T */
        // 1 / 0
        int setStabilisation ( int  stabilisation );
        int getStabilisation ( int *stabilisation );
        
        //MMAL_PARAMETER_FACE_TRACK_RESULTS,        /**< Takes a @ref MMAL_PARAMETER_FACE_TRACK_RESULTS_T */
        //MMAL_PARAMETER_ENABLE_RAW_CAPTURE,        /**< Takes a @ref MMAL_PARAMETER_BOOLEAN_T */
        // ---> not used/supported/directly controllable in this lib

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
        // ---> not used/supported/directly controllable in this lib
        
        //MMAL_PARAMETER_DYNAMIC_RANGE_COMPRESSION, /**< Takes a @ref MMAL_PARAMETER_DRC_T */
        /*
            MMAL_PARAMETER_DRC_STRENGTH_OFF,
            MMAL_PARAMETER_DRC_STRENGTH_LOW,
            MMAL_PARAMETER_DRC_STRENGTH_MEDIUM,
            MMAL_PARAMETER_DRC_STRENGTH_HIGH,
            MMAL_PARAMETER_DRC_STRENGTH_MAX = 0x7fffffff
        */
        // Enabling DRC will `override fixed white balance`_ gains (set via :attr:`awb_gains` and :attr:`awb_mode`).

        int setDRC ( MMAL_PARAMETER_DRC_STRENGTH_T  strength );
        int getDRC ( MMAL_PARAMETER_DRC_STRENGTH_T *strength );
        
        //MMAL_PARAMETER_ALGORITHM_CONTROL,         /**< Takes a @ref MMAL_PARAMETER_ALGORITHM_CONTROL_T */
        // ---> not used/supported/directly controllable in this lib

        //MMAL_PARAMETER_SHARPNESS,                 /**< Takes a @ref MMAL_PARAMETER_RATIONAL_T */
        // -100 to 100
        /*
            MMAL_RATIONAL_T value = {sharpness, 100};
        */
        int setSharpness ( int  sharpness );                       
        int getSharpness ( int *sharpness );

        //MMAL_PARAMETER_CONTRAST,                  /**< Takes a @ref MMAL_PARAMETER_RATIONAL_T */
        // -100 to 100 
        /*
            MMAL_RATIONAL_T value = {contrast, 100};
         */
        int setContrast ( int  contrast ); 
        int getContrast ( int *contrast );

        //MMAL_PARAMETER_BRIGHTNESS,                /**< Takes a @ref MMAL_PARAMETER_RATIONAL_T */
        //    0 to 100 
        /*
            MMAL_RATIONAL_T value = {brightness, 100};
         */
        int setBrightness ( int  brightness );    
        int getBrightness ( int *brightness );

        //MMAL_PARAMETER_SATURATION,                /**< Takes a @ref MMAL_PARAMETER_RATIONAL_T */
        // -100 to 100  
        /*
            MMAL_RATIONAL_T value = {saturation, 100};
         */
        int setSaturation ( int  saturation ); 
        int getSaturation ( int *saturation );

        /* 0x30 */
        //MMAL_PARAMETER_ISO,                       /**< Takes a @ref MMAL_PARAMETER_UINT32_T */
        //    0 to 1600
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
        //MMAL_PARAMETER_CAPTURE_EXPOSURE_COMP,     /**< Takes a @ref MMAL_PARAMETER_INT32_T */
        // ---> not used/supported/directly controllable in this lib

        /* 0x40 */
        //MMAL_PARAMETER_SW_SHARPEN_DISABLE,        /**< Takes a @ref MMAL_PARAMETER_BOOLEAN_T */
        //MMAL_PARAMETER_FLASH_REQUIRED,            /**< Takes a @ref MMAL_PARAMETER_BOOLEAN_T */
        //MMAL_PARAMETER_SW_SATURATION_DISABLE,     /**< Takes a @ref MMAL_PARAMETER_BOOLEAN_T */
        // ---> not used/supported/directly controllable in this lib
        
        //MMAL_PARAMETER_SHUTTER_SPEED,             /**< Takes a @ref MMAL_PARAMETER_UINT32_T */
        //   0 to 330000 (microseconds)
        int setShutterSpeed ( unsigned int  speed );
        int getShutterSpeed ( unsigned int *speed );
        
        //MMAL_PARAMETER_CUSTOM_AWB_GAINS,          /**< Takes a @ref MMAL_PARAMETER_AWB_GAINS_T */
        // ONLY WHEN AWB=OFF
        // 0.0 to 8.0
        /*
            param.r_gain.num = (unsigned int)(r_gain * 65536);
            param.b_gain.num = (unsigned int)(b_gain * 65536);
            param.r_gain.den = param.b_gain.den = 65536;
        */
        int setAWBGains ( float  red , float  blue );
        int getAWBGains ( float *red , float *blue );
        
        //MMAL_PARAMETER_CAMERA_SETTINGS,           /**< Takes a @ref MMAL_PARAMETER_CAMERA_SETTINGS_T */
        //MMAL_PARAMETER_PRIVACY_INDICATOR,         /**< Takes a @ref MMAL_PARAMETER_PRIVACY_INDICATOR_T */
        //MMAL_PARAMETER_VIDEO_DENOISE,             /**< Takes a @ref MMAL_PARAMETER_BOOLEAN_T */
        // ---> not used/supported/directly controllable in this lib

        //MMAL_PARAMETER_STILLS_DENOISE,            /**< Takes a @ref MMAL_PARAMETER_BOOLEAN_T */
        // 1 or 0
        int setDenoise ( int  denoise );
        int getDenoise ( int *denoise );
        
        //MMAL_PARAMETER_ANNOTATE,                  /**< Takes a @ref MMAL_PARAMETER_CAMERA_ANNOTATE_T */
        //MMAL_PARAMETER_STEREOSCOPIC_MODE,         /**< Takes a @ref MMAL_PARAMETER_STEREOSCOPIC_MODE_T */
        // ---> not used/supported/directly controllable in this lib
    };
//}


#endif /* FlashCam_h */

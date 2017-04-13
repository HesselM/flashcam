#include "FlashCam.h"

#include "interface/mmal/util/mmal_util_params.h"


#define FLASHCAMOUTPUT 1

namespace flashcam {
    
    //FlashCam::loadSettings() {
        /*
        //Image properties
        unsigned int getSharpness ( void );
        unsigned int getContrast ( void );
        unsigned int getBrightness ( void );
        unsigned int getSaturation ( void );
        unsigned int getISO ( void );
        
        unsigned int getStabilisation ( void );
        unsigned int getExposureCompensation ( void );
        unsigned int getQuality ( void );
        MMAL_FOURCC_T getEncoding ( void );
        
        //Camera Modes
        MMAL_PARAM_EXPOSUREMODE_T getExposureMode ( void );
        MMAL_PARAM_EXPOSUREMETERINGMODE_T getMeteringMode ( void );
        MMAL_PARAM_AWBMODE_T getAWBMode ( void );
        MMAL_PARAM_FLASH_T getFlashMode ( void );
        
        //Camera Properties
        unsigned int getRotation ( void );
        unsigned int getFlipHorizontal ( void );
        unsigned int getFlipVertical ( void );
        FLASHCAM_PARAM_FLOAT_RECT_T getROI ( void );
        unsigned int getShutterSpeed ( void );
        
        //Camera Gains
        unsigned int getGainRed ( void );
        unsigned int getGainBlue ( void );
        MMAL_PARAMETER_DRC_STRENGTH_T getDRC ( void );
        
        //Effects
        MMAL_PARAM_IMAGEFX_T getImageFX ( void );
        MMAL_PARAM_COLOURFX_T getColourFX ( void );
        */
        
    
    /* GENERAL (PRIVATE) SETTER/GETTER */
    
    int FlashCam::setParameterRational( int id, int val ) {
        MMAL_RATIONAL_T rational = {val, 100};
        MMAL_STATUS_T   status   = mmal_port_parameter_set_rational(camera->control, id, rational );
        return mmal_status_to_int(status);
    }
    
    int FlashCam::getParameterRational( int id, int *val ) {
        MMAL_RATIONAL_T rational;        
        MMAL_STATUS_T   status = mmal_port_parameter_get_rational(camera->control, id, &rational);
        *val                   = rational.num;
        return mmal_status_to_int(status);        
    }
    
    /* PUBLIC SETTER/GETTER */
    
    int FlashCam::setRotation ( int rotation ) {
        rotation = ((rotation % 360 ) / 90) * 90;
        MMAL_STATUS_T status = mmal_port_parameter_set_int32(camera->output[FLASHCAMOUTPUT], MMAL_PARAMETER_ROTATION, rotation);        
        return mmal_status_to_int(status);
    }
    
    int FlashCam::getRotation ( int *rotation ) {
        MMAL_STATUS_T status = mmal_port_parameter_get_int32(camera->output[FLASHCAMOUTPUT], MMAL_PARAMETER_ROTATION, rotation);        
        return mmal_status_to_int(status);
    }
    
    int FlashCam::setAWBMode ( MMAL_PARAM_AWBMODE_T awb ) {
        MMAL_PARAMETER_AWBMODE_T param  = {{MMAL_PARAMETER_AWB_MODE, sizeof(param)}, awb};
        MMAL_STATUS_T            status = mmal_port_parameter_set(camera->control, &param.hdr);        
        return mmal_status_to_int(status);
    }
    
    int FlashCam::getAWBMode ( MMAL_PARAM_AWBMODE_T *awb ) {
        // MMAL_PARAM_AWBMODE_MAX: just usa a value to allocate `param`
        MMAL_PARAMETER_AWBMODE_T param  = {{MMAL_PARAMETER_AWB_MODE, sizeof(param)}, MMAL_PARAM_AWBMODE_MAX};
        MMAL_STATUS_T            status = mmal_port_parameter_get(camera->control, &param.hdr);
        //update value
        *awb = param.value;
        return mmal_status_to_int(status);
    }
    
    int FlashCam::setFlashMode ( MMAL_PARAM_FLASH_T flash ) {
        MMAL_PARAMETER_FLASH_T param  = {{MMAL_PARAMETER_FLASH, sizeof(param)}, flash};
        MMAL_STATUS_T          status = mmal_port_parameter_set(camera->control, &param.hdr);
        return mmal_status_to_int(status);
    }
     
    int FlashCam::getFlashMode ( MMAL_PARAM_FLASH_T *flash ) {
        // MMAL_PARAM_FLASH_MAX: just usa a value to allocate `param`
        MMAL_PARAMETER_FLASH_T param  = {{MMAL_PARAMETER_FLASH, sizeof(param)}, MMAL_PARAM_FLASH_MAX};
        MMAL_STATUS_T          status = mmal_port_parameter_get(camera->control, &param.hdr);
        //update value
        *flash = param.value;
        return mmal_status_to_int(status);
    }
    
    int FlashCam::setMirror ( MMAL_PARAM_MIRROR_T mirror ) {
        MMAL_PARAMETER_MIRROR_T param  = {{MMAL_PARAMETER_MIRROR, sizeof(param)}, mirror};
        MMAL_STATUS_T           status = mmal_port_parameter_set(camera->output[FLASHCAMOUTPUT], &param.hdr);
        return mmal_status_to_int(status);
    }
    
    int FlashCam::getMirror ( MMAL_PARAM_MIRROR_T *mirror ) {
        // MMAL_PARAM_MIRROR_NONE: just usa a value to allocate `param`
        MMAL_PARAMETER_MIRROR_T param  = {{MMAL_PARAMETER_MIRROR, sizeof(param)}, MMAL_PARAM_MIRROR_NONE};
        MMAL_STATUS_T           status = mmal_port_parameter_get(camera->output[FLASHCAMOUTPUT], &param.hdr);       
        //update value
        *mirror = param.value;
        return mmal_status_to_int(status);
    }

    int FlashCam::setExposureMode ( MMAL_PARAM_EXPOSUREMODE_T exposure ) {
        MMAL_PARAMETER_EXPOSUREMODE_T param  = {{MMAL_PARAMETER_EXPOSURE_MODE, sizeof(param)}, exposure};
        MMAL_STATUS_T                 status = mmal_port_parameter_set(camera->control, &param.hdr);
        return mmal_status_to_int(status);
    }
    
    int FlashCam::getExposureMode ( MMAL_PARAM_EXPOSUREMODE_T *exposure ) {
        // MMAL_PARAM_EXPOSUREMODE_MAX: just usa a value to allocate `param`
        MMAL_PARAMETER_EXPOSUREMODE_T param  = {{MMAL_PARAMETER_EXPOSURE_MODE, sizeof(param)}, MMAL_PARAM_EXPOSUREMODE_MAX};
        MMAL_STATUS_T                 status = mmal_port_parameter_get(camera->control, &param.hdr);       
        //update value
        *exposure = param.value;
        return mmal_status_to_int(status);
    }
    
    int FlashCam::setMeteringMode ( MMAL_PARAM_EXPOSUREMETERINGMODE_T  metering ) {
        MMAL_PARAMETER_EXPOSUREMETERINGMODE_T param  = {{MMAL_PARAMETER_EXP_METERING_MODE, sizeof(param)}, metering};        
        MMAL_STATUS_T                         status = mmal_port_parameter_set(camera->control, &param.hdr);
        return mmal_status_to_int(status);
    }
    
    int FlashCam::getMeteringMode ( MMAL_PARAM_EXPOSUREMETERINGMODE_T *metering ) {
        // MMAL_PARAM_EXPOSUREMETERINGMODE_MAX: just usa a value to allocate `param`
        MMAL_PARAMETER_EXPOSUREMETERINGMODE_T param  = {{MMAL_PARAMETER_EXP_METERING_MODE, sizeof(param)}, MMAL_PARAM_EXPOSUREMETERINGMODE_MAX};
        MMAL_STATUS_T                         status = mmal_port_parameter_get(camera->control, &param.hdr);       
        //update value
        *metering = param.value;
        return mmal_status_to_int(status);
    }

    int FlashCam::setStabilisation ( int stabilisation ) {
        MMAL_STATUS_T status = mmal_port_parameter_set_boolean(camera->control, MMAL_PARAMETER_VIDEO_STABILISATION, stabilisation);        
        return mmal_status_to_int(status);
    }
    
    int FlashCam::getStabilisation ( int *stabilisation ) {
        MMAL_STATUS_T status = mmal_port_parameter_get_boolean(camera->control, MMAL_PARAMETER_VIDEO_STABILISATION, stabilisation);        
        return mmal_status_to_int(status);
    }
    
    int FlashCam::setDRC ( MMAL_PARAMETER_DRC_STRENGTH_T  strength ) {
        MMAL_PARAMETER_DRC_T param  = {{MMAL_PARAMETER_DYNAMIC_RANGE_COMPRESSION, sizeof(param)}, strength};
        MMAL_STATUS_T        status = mmal_port_parameter_set(camera->control, &param.hdr);
        return mmal_status_to_int(status);
    }
    
    int FlashCam::getDRC ( MMAL_PARAMETER_DRC_STRENGTH_T *strength ) {
        // MMAL_PARAMETER_DRC_STRENGTH_MAX: just usa a value to allocate `param`
        MMAL_PARAMETER_DRC_T param  = {{MMAL_PARAMETER_DYNAMIC_RANGE_COMPRESSION, sizeof(param)}, MMAL_PARAMETER_DRC_STRENGTH_MAX};
        MMAL_STATUS_T        status = mmal_port_parameter_get(camera->control, &param.hdr);  
        //update value
        *strength = param.strength;
        return mmal_status_to_int(status);
    }
    
    int FlashCam::setSharpness ( int  sharpness ) {
        if ( sharpness < -100 ) sharpness = -100;
        if ( sharpness >  100 ) sharpness =  100;
        return setParameterRational(MMAL_PARAMETER_SHARPNESS, sharpness);
    }
    
    int FlashCam::getSharpness ( int *sharpness ) {
        return getParameterRational( MMAL_PARAMETER_SHARPNESS, sharpness);        
    }

    int FlashCam::setContrast ( int  contrast ) {
        if ( contrast < -100 ) contrast = -100;
        if ( contrast >  100 ) contrast =  100;
        return setParameterRational(MMAL_PARAMETER_CONTRAST, contrast);
    }
    
    int FlashCam::getContrast ( int *contrast ) {
        return getParameterRational( MMAL_PARAMETER_CONTRAST, contrast);        
    }
    
    int FlashCam::setBrightness ( int  brightness ) {
        if ( brightness <    0 ) brightness =    0;
        if ( brightness >  100 ) brightness =  100;
        return setParameterRational(MMAL_PARAMETER_BRIGHTNESS, brightness);
    }
    
    int FlashCam::getBrightness ( int *brightness ) {
        return getParameterRational( MMAL_PARAMETER_BRIGHTNESS, brightness);        
    }
    
    int FlashCam::setSaturation ( int  saturation ) {        
        if ( saturation < -100 ) saturation = -100;
        if ( saturation >  100 ) saturation =  100;
        return setParameterRational(MMAL_PARAMETER_SATURATION, saturation);
    }
    
    int FlashCam::getSaturation ( int *saturation ) {
        return getParameterRational( MMAL_PARAMETER_SATURATION, saturation);        
    }
    
    int FlashCam::setISO ( unsigned int  iso ) {        
        if ( iso > 1600 ) iso = 1600;
        MMAL_STATUS_T status = mmal_port_parameter_set_uint32(camera->control, MMAL_PARAMETER_ISO, iso);        
        return mmal_status_to_int(status);
    }
    
    int FlashCam::getISO ( unsigned int *iso ) {        
        MMAL_STATUS_T status = mmal_port_parameter_get_uint32(camera->control, MMAL_PARAMETER_ISO, iso);        
        return mmal_status_to_int(status);
    }
    
    int FlashCam::setShutterSpeed ( unsigned int  speed ) {        
        if ( speed > 330000 ) speed = 330000;
        MMAL_STATUS_T status = mmal_port_parameter_set_uint32(camera->control, MMAL_PARAMETER_SHUTTER_SPEED, speed);        
        return mmal_status_to_int(status);
    }
    
    int FlashCam::getShutterSpeed ( unsigned int *speed ) {        
        MMAL_STATUS_T status = mmal_port_parameter_get_uint32(camera->control, MMAL_PARAMETER_SHUTTER_SPEED, speed);        
        return mmal_status_to_int(status);
    }

    int FlashCam::setAWBGains ( float red , float blue ) {      
        //recompute gains
        MMAL_RATIONAL_T r, b;
        r.num = (unsigned int) (red  * 65536);
        b.num = (unsigned int) (blue * 65536);
        r.den = b.den = 65536;
        //setup param
        MMAL_PARAMETER_AWB_GAINS_T param  = {{MMAL_PARAMETER_CUSTOM_AWB_GAINS,sizeof(param)}, r, b};
        MMAL_STATUS_T              status = mmal_port_parameter_set(camera->control, &param.hdr);  
        return mmal_status_to_int(status);
    }
    
    int FlashCam::getAWBGains ( float *red , float *blue ) {      
        // {0,0}, {0,0}: just usa a value to allocate `param`
        MMAL_PARAMETER_AWB_GAINS_T param  = {{MMAL_PARAMETER_CUSTOM_AWB_GAINS, sizeof(param)}, {0,0}, {0,0}};
        MMAL_STATUS_T              status = mmal_port_parameter_get(camera->control, &param.hdr);  
        //update value
        *red  = ((float)param.r_gain.num) / ((float)param.r_gain.den) ;
        *blue = ((float)param.b_gain.num) / ((float)param.b_gain.den) ;
        return mmal_status_to_int(status);
    }
    
    int FlashCam::setDenoise ( int  denoise ) {
        MMAL_STATUS_T status = mmal_port_parameter_set_boolean(camera->control, MMAL_PARAMETER_STILLS_DENOISE, denoise);        
        return mmal_status_to_int(status);
    }
    
    int FlashCam::getDenoise ( int *denoise ) {
        MMAL_STATUS_T status = mmal_port_parameter_get_boolean(camera->control, MMAL_PARAMETER_STILLS_DENOISE, denoise);        
        return mmal_status_to_int(status);
    }
    
    
}

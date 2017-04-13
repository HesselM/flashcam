#include "FlashCam.h"

#include "interface/mmal/util/mmal_util_params.h"

/*


MMAL_STATUS_T mmal_port_parameter_get_boolean(MMAL_PORT_T *port, uint32_t id, MMAL_BOOL_T *value);
MMAL_STATUS_T mmal_port_parameter_set_boolean(MMAL_PORT_T *port, uint32_t id, MMAL_BOOL_T *value);

MMAL_STATUS_T mmal_port_parameter_set_uint64(MMAL_PORT_T *port, uint32_t id, uint64_t value);
MMAL_STATUS_T mmal_port_parameter_get_uint64(MMAL_PORT_T *port, uint32_t id, uint64_t *value);

MMAL_STATUS_T mmal_port_parameter_set_int64(MMAL_PORT_T *port, uint32_t id, int64_t value);
MMAL_STATUS_T mmal_port_parameter_get_int64(MMAL_PORT_T *port, uint32_t id, int64_t *value);

MMAL_STATUS_T mmal_port_parameter_set_uint32(MMAL_PORT_T *port, uint32_t id, uint32_t value);
MMAL_STATUS_T mmal_port_parameter_get_uint32(MMAL_PORT_T *port, uint32_t id, uint32_t *value);

MMAL_STATUS_T mmal_port_parameter_set_int32(MMAL_PORT_T *port, uint32_t id, int32_t value);
MMAL_STATUS_T mmal_port_parameter_get_int32(MMAL_PORT_T *port, uint32_t id, int32_t *value);

MMAL_STATUS_T mmal_port_parameter_set_rational(MMAL_PORT_T *port, uint32_t id, MMAL_RATIONAL_T value);
MMAL_STATUS_T mmal_port_parameter_get_rational(MMAL_PORT_T *port, uint32_t id, MMAL_RATIONAL_T *value);
*/

/** Helper function to set the value of a string parameter.
 * @param port   port on which to set the parameter
 * @param id     parameter id
 * @param value  null-terminated string value
 *
 * @return MMAL_SUCCESS or error
 */
//MMAL_STATUS_T mmal_port_parameter_set_string(MMAL_PORT_T *port, uint32_t id, const char *value);

/** Helper function to set the value of an array of bytes parameter.
 * @param port   port on which to set the parameter
 * @param id     parameter id
 * @param data   pointer to the array of bytes
 * @param size   size of the array of bytes
 *
 * @return MMAL_SUCCESS or error
 */
//MMAL_STATUS_T mmal_port_parameter_set_bytes(MMAL_PORT_T *port, uint32_t id,
//                                            const uint8_t *data, unsigned int size);

/** Helper function to set a MMAL_PARAMETER_URI_T parameter on a port.
 * @param port   port on which to set the parameter
 * @param uri    URI string
 *
 * @return MMAL_SUCCESS or error
 */
//MMAL_STATUS_T mmal_util_port_set_uri(MMAL_PORT_T *port, const char *uri);

/** Set the display region.
 * @param port   port to configure
 * @param region region
 *
 * @return MMAL_SUCCESS or error
 */
//MMAL_STATUS_T mmal_util_set_display_region(MMAL_PORT_T *port,
//                                           MMAL_DISPLAYREGION_T *region);

/** Tell the camera to use the STC for timestamps rather than the clock.
 *
 * @param port   port to configure
 * @param mode   STC mode to use
 * @return MMAL_SUCCESS or error
 */
//MMAL_STATUS_T mmal_util_camera_use_stc_timestamp(MMAL_PORT_T *port, MMAL_CAMERA_STC_MODE_T mode);

/** Get the MMAL core statistics for a given port.
 *
 * @param port  port to query
 * @param dir   port direction
 * @param reset reset the stats as well
 * @param stats filled in with results
 * @return MMAL_SUCCESS or error
 */
//MMAL_STATUS_T mmal_util_get_core_port_stats(MMAL_PORT_T *port, MMAL_CORE_STATS_DIR dir, MMAL_BOOL_T reset,
//                                            MMAL_CORE_STATISTICS_T *stats);


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
        
    
    int FlashCam::setRotation ( int rotation ) {
        MMAL_STATUS_T status = MMAL_SUCCESS;
        rotation = ((rotation % 360 ) / 90) * 90;
        
        status = mmal_port_parameter_set_int32(camera->output[0], MMAL_PARAMETER_ROTATION, rotation);
        status = mmal_port_parameter_set_int32(camera->output[1], MMAL_PARAMETER_ROTATION, rotation);
        status = mmal_port_parameter_set_int32(camera->output[2], MMAL_PARAMETER_ROTATION, rotation);
        
        return mmal_status_to_int(status);
    }
    
    int FlashCam::getRotation ( int *rotation ) {
        MMAL_STATUS_T status = MMAL_SUCCESS;
        
        status = mmal_port_parameter_get_int32(camera->output[0], MMAL_PARAMETER_ROTATION, rotation);
        
        return mmal_status_to_int(status);
    }
    
    int FlashCam::setAWBMode ( MMAL_PARAM_AWBMODE_T awb ) {
        MMAL_STATUS_T status = MMAL_SUCCESS;
        MMAL_PARAMETER_AWBMODE_T param = {{MMAL_PARAMETER_AWB_MODE, sizeof(param)}, awb};
        
        status = mmal_port_parameter_set(camera->control, &param.hdr);
        
        return mmal_status_to_int(status);
    }
    
    int FlashCam::getAWBMode ( MMAL_PARAM_AWBMODE_T *awb ) {
        MMAL_STATUS_T status = MMAL_SUCCESS;
        // MMAL_PARAM_AWBMODE_MAX: just usa a value to allocate `param`
        MMAL_PARAMETER_AWBMODE_T param = {{MMAL_PARAMETER_AWB_MODE, sizeof(param)}, MMAL_PARAM_AWBMODE_MAX};

        status = mmal_port_parameter_get(camera->control, &param.hdr);
        *awb   = param.value;
        
        return mmal_status_to_int(status);
    }
    
    int FlashCam::setFlashMode ( MMAL_PARAM_FLASH_T flash ) {
        MMAL_STATUS_T status = MMAL_SUCCESS;
        MMAL_PARAMETER_FLASH_T param = {{MMAL_PARAMETER_FLASH, sizeof(param)}, flash};

        status = mmal_port_parameter_set(camera->control, &param.hdr);
        
        return mmal_status_to_int(status);
    }
     
    int FlashCam::getFlashMode ( MMAL_PARAM_FLASH_T *flash ) {
        MMAL_STATUS_T status = MMAL_SUCCESS;
        // MMAL_PARAM_FLASH_MAX: just usa a value to allocate `param`
        MMAL_PARAMETER_FLASH_T param = {{MMAL_PARAMETER_FLASH, sizeof(param)}, MMAL_PARAM_FLASH_MAX};
        
        status = mmal_port_parameter_get(camera->control, &param.hdr);
        *flash = param.value;
        
        return mmal_status_to_int(status);
    }
    
    int FlashCam::setMirror ( MMAL_PARAM_MIRROR_T mirror ) {
        MMAL_STATUS_T status = MMAL_SUCCESS;
        MMAL_PARAMETER_MIRROR_T param = {{MMAL_PARAMETER_MIRROR, sizeof(param)}, mirror};
        
        status = mmal_port_parameter_set(camera->output[0], &param.hdr);
        status = mmal_port_parameter_set(camera->output[1], &param.hdr);
        status = mmal_port_parameter_set(camera->output[2], &param.hdr);
        
        return mmal_status_to_int(status);
    }
    
    int FlashCam::getMirror ( MMAL_PARAM_MIRROR_T *mirror ) {
        MMAL_STATUS_T status = MMAL_SUCCESS;
        // MMAL_PARAM_MIRROR_NONE: just usa a value to allocate `param`
        MMAL_PARAMETER_MIRROR_T param = {{MMAL_PARAMETER_MIRROR, sizeof(param)}, MMAL_PARAM_MIRROR_NONE};
                
        status  = mmal_port_parameter_get(camera->output[0], &param.hdr);       
        *mirror = param.value;
        
        return mmal_status_to_int(status);
    }

    int FlashCam::setExposureMode ( MMAL_PARAM_EXPOSUREMODE_T exposure ) {
        MMAL_STATUS_T status = MMAL_SUCCESS;
        MMAL_PARAMETER_EXPOSUREMODE_T param = {{MMAL_PARAMETER_EXPOSURE_MODE, sizeof(param)}, exposure};
        
        status = mmal_port_parameter_set(camera->control, &param.hdr);
        
        return mmal_status_to_int(status);
    }
    
    int FlashCam::getExposureMode ( MMAL_PARAM_EXPOSUREMODE_T *exposure ) {
        MMAL_STATUS_T status = MMAL_SUCCESS;
        // MMAL_PARAM_EXPOSUREMODE_MAX: just usa a value to allocate `param`
        MMAL_PARAMETER_EXPOSUREMODE_T param = {{MMAL_PARAMETER_EXPOSURE_MODE, sizeof(param)}, MMAL_PARAM_EXPOSUREMODE_MAX};
        
        status  = mmal_port_parameter_get(camera->control, &param.hdr);       
        *exposure = param.value;
        
        return mmal_status_to_int(status);
    }
    
    int FlashCam::setMeteringMode ( MMAL_PARAM_EXPOSUREMETERINGMODE_T  metering ) {
        MMAL_STATUS_T status = MMAL_SUCCESS;
        MMAL_PARAMETER_EXPOSUREMETERINGMODE_T param = {{MMAL_PARAMETER_EXP_METERING_MODE, sizeof(param)}, metering};
        
        status = mmal_port_parameter_set(camera->control, &param.hdr);
        
        return mmal_status_to_int(status);
    }
    
    int FlashCam::getMeteringMode ( MMAL_PARAM_EXPOSUREMETERINGMODE_T *metering ) {
        MMAL_STATUS_T status = MMAL_SUCCESS;
        // MMAL_PARAM_EXPOSUREMETERINGMODE_MAX: just usa a value to allocate `param`
        MMAL_PARAMETER_EXPOSUREMETERINGMODE_T param = {{MMAL_PARAMETER_EXP_METERING_MODE, sizeof(param)}, MMAL_PARAM_EXPOSUREMETERINGMODE_MAX};
        
        status  = mmal_port_parameter_get(camera->control, &param.hdr);       
        *metering = param.value;
        
        return mmal_status_to_int(status);
    }

    int FlashCam::setStabilisation ( int stabilisation ) {
        MMAL_STATUS_T status = MMAL_SUCCESS;

        status = mmal_port_parameter_set_boolean(camera->control, MMAL_PARAMETER_VIDEO_STABILISATION, stabilisation);
        
        return mmal_status_to_int(status);
    }
    
    int FlashCam::getStabilisation ( int *stabilisation ) {
        MMAL_STATUS_T status = MMAL_SUCCESS;
        
        status = mmal_port_parameter_get_boolean(camera->control, MMAL_PARAMETER_VIDEO_STABILISATION, stabilisation);
        
        return mmal_status_to_int(status);
    }
    
    int FlashCam::setDRC ( MMAL_PARAMETER_DRC_STRENGTH_T  strength ) {
        MMAL_STATUS_T status = MMAL_SUCCESS;
        MMAL_PARAMETER_DRC_T param = {{MMAL_PARAMETER_DYNAMIC_RANGE_COMPRESSION, sizeof(param)}, strength};
        
        status = mmal_port_parameter_set(camera->control, &param.hdr);
        
        return mmal_status_to_int(status);
    }
    
    int FlashCam::getDRC ( MMAL_PARAMETER_DRC_STRENGTH_T *strength ) {
        MMAL_STATUS_T status = MMAL_SUCCESS;
        // MMAL_PARAMETER_DRC_STRENGTH_MAX: just usa a value to allocate `param`
        MMAL_PARAMETER_DRC_T param = {{MMAL_PARAMETER_DYNAMIC_RANGE_COMPRESSION, sizeof(param)}, MMAL_PARAMETER_DRC_STRENGTH_MAX};
        
        status    = mmal_port_parameter_get(camera->control, &param.hdr);       
        *strength = param.strength;
        
        return mmal_status_to_int(status);
    }
    
    int FlashCam::setSharpness ( int  sharpness ) {
        MMAL_STATUS_T status = MMAL_SUCCESS;
        
        if ( sharpness < -100 ) sharpness = -100;
        if ( sharpness >  100 ) sharpness =  100;
        MMAL_RATIONAL_T rational = {sharpness, 100};

        status = mmal_port_parameter_set_rational(camera->control, MMAL_PARAMETER_SHARPNESS, rational );
        
        return mmal_status_to_int(status);
    }
    
    int FlashCam::getSharpness ( int *sharpness ) {
        MMAL_STATUS_T status = MMAL_SUCCESS;
        MMAL_RATIONAL_T rational;

        status    = mmal_port_parameter_get_rational(camera->control, MMAL_PARAMETER_SHARPNESS, &rational);
        *sharpness = rational.num;
        
        return mmal_status_to_int(status);
    }

    
}

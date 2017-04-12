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

#ifndef RASPICAMCONTROL_H_
#define RASPICAMCONTROL_H_

typedef struct xref_t
{
    char *mode;
    int mmal_mode;
} XREF_T;

// There isn't actually a MMAL structure for the following, so make one
typedef struct mmal_param_colourfx_s
{
   int enable;       /// Turn colourFX on or off
   int u,v;          /// U and V to use
} MMAL_PARAM_COLOURFX_T;

typedef struct mmal_param_thumbnail_config_s
{
   int enable;
   int width,height;
   int quality;
} MMAL_PARAM_THUMBNAIL_CONFIG_T;

typedef struct param_float_rect_s
{
   double x;
   double y;
   double w;
   double h;
} PARAM_FLOAT_RECT_T;

/// struct contain camera settings
typedef struct raspicam_camera_parameters_s
{
   int sharpness;             /// -100 to 100
   int contrast;              /// -100 to 100
   int brightness;            ///  0 to 100
   int saturation;            ///  -100 to 100
   int ISO;                   ///  TODO : what range?
   int videoStabilisation;    /// 0 or 1 (false or true)
   int exposureCompensation;  /// -10 to +10 ?
    
   MMAL_PARAM_EXPOSUREMODE_T exposureMode;
   MMAL_PARAM_EXPOSUREMETERINGMODE_T exposureMeterMode;
   MMAL_PARAM_AWBMODE_T awbMode;
   MMAL_PARAM_IMAGEFX_T imageEffect;
   MMAL_PARAMETER_IMAGEFX_PARAMETERS_T imageEffectsParameters;
   MMAL_PARAM_COLOURFX_T colourEffects;
    MMAL_PARAM_FLASH_T flashMode;
    
   int rotation;              /// 0-359
   int hflip;                 /// 0 or 1
   int vflip;                 /// 0 or 1
   PARAM_FLOAT_RECT_T  roi;   /// region of interest to use on the sensor. Normalised [0,1] values in the rect
   int shutter_speed;         /// 0 = auto, otherwise the shutter speed in ms
    
   float awb_gains_r;         /// AWB red gain
   float awb_gains_b;         /// AWB blue gain
    
   MMAL_PARAMETER_DRC_STRENGTH_T drc_level;  // Strength of Dynamic Range compression to apply
   MMAL_BOOL_T stats_pass;    /// Stills capture statistics pass on/off
    
} RASPICAM_CAMERA_PARAMETERS;

typedef enum {
    ZOOM_IN, ZOOM_OUT, ZOOM_RESET
} ZOOM_COMMAND_T;


int raspicamcontrol_set_all_parameters(MMAL_COMPONENT_T *camera, const RASPICAM_CAMERA_PARAMETERS *params);
int raspicamcontrol_get_all_parameters(MMAL_COMPONENT_T *camera, RASPICAM_CAMERA_PARAMETERS *params);
void raspicamcontrol_dump_parameters(const RASPICAM_CAMERA_PARAMETERS *params);

void raspicamcontrol_set_defaults(RASPICAM_CAMERA_PARAMETERS *params);

void raspicamcontrol_check_configuration(int min_gpu_mem);

// Individual setting functions
int raspicamcontrol_set_sharpness(MMAL_COMPONENT_T *camera, int sharpness);
int raspicamcontrol_set_contrast(MMAL_COMPONENT_T *camera, int contrast);
int raspicamcontrol_set_brightness(MMAL_COMPONENT_T *camera, int brightness);
int raspicamcontrol_set_saturation(MMAL_COMPONENT_T *camera, int saturation);
int raspicamcontrol_set_ISO(MMAL_COMPONENT_T *camera, int ISO);
int raspicamcontrol_set_video_stabilisation(MMAL_COMPONENT_T *camera, int vstabilisation);
int raspicamcontrol_set_exposure_compensation(MMAL_COMPONENT_T *camera, int exp_comp);
//
int raspicamcontrol_set_exposure_mode(MMAL_COMPONENT_T *camera, MMAL_PARAM_EXPOSUREMODE_T mode);
int raspicamcontrol_set_metering_mode(MMAL_COMPONENT_T *camera, MMAL_PARAM_EXPOSUREMETERINGMODE_T mode);
int raspicamcontrol_set_awb_mode(MMAL_COMPONENT_T *camera, MMAL_PARAM_AWBMODE_T awb_mode);
int raspicamcontrol_set_imageFX(MMAL_COMPONENT_T *camera, MMAL_PARAM_IMAGEFX_T imageFX);
//MMAL_PARAMETER_IMAGEFX_PARAMETERS_T ??
int raspicamcontrol_set_colourFX(MMAL_COMPONENT_T *camera, const MMAL_PARAM_COLOURFX_T *colourFX);
//TODO: flash / MMAL_PARAM_FLASH_T
//
int raspicamcontrol_set_rotation(MMAL_COMPONENT_T *camera, int rotation);
int raspicamcontrol_set_flips(MMAL_COMPONENT_T *camera, int hflip, int vflip);
int raspicamcontrol_set_ROI(MMAL_COMPONENT_T *camera, PARAM_FLOAT_RECT_T rect);
int raspicamcontrol_set_shutter_speed(MMAL_COMPONENT_T *camera, int speed_ms);
//
int raspicamcontrol_set_awb_gains(MMAL_COMPONENT_T *camera, float r_gain, float b_gain);
//
int raspicamcontrol_set_DRC(MMAL_COMPONENT_T *camera, MMAL_PARAMETER_DRC_STRENGTH_T strength);
int raspicamcontrol_set_stats_pass(MMAL_COMPONENT_T *camera, int stats_pass);
//
int raspicamcontrol_zoom_in_zoom_out(MMAL_COMPONENT_T *camera, ZOOM_COMMAND_T zoom_command, PARAM_FLOAT_RECT_T *roi);


//Individual getting functions
int raspicamcontrol_get_sharpness(MMAL_COMPONENT_T *camera);
int raspicamcontrol_get_contrast(MMAL_COMPONENT_T *camera);
int raspicamcontrol_get_brightness(MMAL_COMPONENT_T *camera);
int raspicamcontrol_get_saturation(MMAL_COMPONENT_T *camera);
int raspicamcontrol_get_ISO(MMAL_COMPONENT_T *camera);
int raspicamcontrol_get_video_stabilisation(MMAL_COMPONENT_T *camera);
int raspicamcontrol_get_exposure_compensation(MMAL_COMPONENT_T *camera);
//
MMAL_PARAM_EXPOSUREMODE_T raspicamcontrol_get_exposure_mode(MMAL_COMPONENT_T *camera);
MMAL_PARAM_EXPOSUREMETERINGMODE_T raspicamcontrol_get_metering_mode(MMAL_COMPONENT_T *camera);
MMAL_PARAM_AWBMODE_T raspicamcontrol_get_awb_mode(MMAL_COMPONENT_T *camera);
MMAL_PARAM_IMAGEFX_T raspicamcontrol_get_imageFX(MMAL_COMPONENT_T *camera);
//MMAL_PARAMETER_IMAGEFX_PARAMETERS_T ??
MMAL_PARAM_COLOURFX_T raspicamcontrol_get_colourFX(MMAL_COMPONENT_T *camera);
//TODO: flash / MMAL_PARAM_FLASH_T
//
//TODO: rotation
//TODO: flips
//TODO: ROI
//TODO: shutterspeed
//TODO: awbgains
//TODO: drc
//TODO: stats
//TODO: zoom

#endif /* RASPICAMCONTROL_H_ */

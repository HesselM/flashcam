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

#ifndef types_h
#define types_h

#include "interface/vcos/vcos.h"
#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"

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

//minimum video buffer
#define VIDEO_OUTPUT_BUFFERS_NUM 3

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
    float framerate;                            // Frame rate (fps):    0.0f to    120.0f   -> Updated to real fps when PLL is enabled.
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

#ifdef BUILD_FLASHCAM_WITH_PLL  
    // PLL: Phase Lock Loop ==> Allows the camera (in videomode) to send lightpulse/flash upon frameexposure.
    //                          The Raspberry firmware only support flash when in capture mode, hence this option.
    unsigned int pll_enabled;                   // Use PLL          : On (1) or Off (0)
    unsigned int pll_divider;                   // framerate / pll_divider = frequency of PLL signal
    float pll_pulsewidth;                       // Pulse width (ms) : 0 to 1/frequency   (NOTE: will be rounded according to available accuracy).
    
    //INTERNAL VARIABLES. CANNOT BE SET/READ. USED FOR PLL TRACKING
    float pll_freq;                             // [ copy of params.framerate ]
    uint64_t pll_starttime;                     // [ starttime of PLL in us ]
    uint64_t pll_startinterval;                 // [ Interval in which clock is started ]
    float pll_period;                          // [ period of a single pwm signal in us ]
    uint64_t pll_offset;                        // [ synchronisation moment; time between start and frame -> start of pwm; us ]
#endif
    
} FLASHCAM_SETTINGS_T;


class Status
{
private:
    
public:
    /**
     * Convert a MMAL status return value to a simple boolean of success
     * ALso displays a fault if code is not success
     *
     * @param status The error code to convert
     * @return 0 if status is success, 1 otherwise
     */
    static int mmal_to_int(MMAL_STATUS_T status);
};

#endif /* types_h */

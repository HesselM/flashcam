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

#ifdef BUILD_FLASHCAM_WITH_OPENGL
#include <vector>
#include "GLES2/gl2.h"
#include "EGL/egl.h"
#include "EGL/eglext.h"
#endif


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
#ifdef BUILD_FLASHCAM_WITH_OPENGL
typedef void (*FLASHCAM_CALLBACK_OPENGL_T) (GLuint texid, int w, int h, uint64_t pts, bool pll_state);
#endif



/*
 * FLASHCAM_PARAMS_T
 * Tracker of all camera parameters. This struct is Read Only.
 */
typedef struct {
    int rotation;                               // Camera rotation (degrees) : 0 / 90 / 180 / 270;
    MMAL_PARAM_AWBMODE_T awbmode;               // AWB mode. See: MMAL_PARAM_AWBMODE_T;
    MMAL_PARAM_FLASH_T flashmode;               // Flash mode. See: MMAL_PARAM_FLASH_T;
    MMAL_PARAM_MIRROR_T mirror;                 // Image Mirroring. See: MMAL_PARAM_MIRROR_T;
    unsigned int cameranum;                     // Index of used camera. 
    MMAL_PARAM_EXPOSUREMODE_T exposuremode;     // Exposure mode (e.g. night). See: MMAL_PARAM_EXPOSUREMODE_T;
    MMAL_PARAM_EXPOSUREMETERINGMODE_T metering; // Exposure metering. See: MMAL_PARAM_EXPOSUREMETERINGMODE_Tl;
    float framerate;                            // Frame rate (fps):    0.0f to    120.0f   (Updated to real fps when PLL is enabled.
    int stabilisation;                          // Video Stabilisation. On (1) or Off (0);
    MMAL_PARAMETER_DRC_STRENGTH_T drc;          // Dynamic Range Compression. See: MMAL_PARAMETER_DRC_STRENGTH_T;
    int sharpness;                              // Image Sharpness : -100    to    100
    int contrast;                               // Image Contrast  : -100    to    100
    int brightness;                             // Image Brightness:    0    to    100
    int saturation;                             // Image Saturation: -100    to    100
    unsigned int iso;                           // ISO             :    0    to   1600      (800+ might not work; 0=auto)
    unsigned int sensormode;                    // Sensor Mode. See: https://www.raspberrypi.org/documentation/raspbian/applications/camera.md
    unsigned int shutterspeed;                  // Shutterspeed    :    0    to 330000      (microseconds; limited by fps in VideoMode)
    float awbgain_red;                          // AWB gain red    :    0.0f to      8.0f   (Only used when AWB=OFF)
    float awbgain_blue;                         // AWB gain blue   :    0.0f to      8.0f   (Only used when AWB=OFF)
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
    unsigned int sensormode;                    // Sensor Mode. See: https://www.raspberrypi.org/documentation/raspbian/applications/camera.md
                                                //  - Note that `sensormode` is also defined as `params`. It is located at these locations as it
                                                //    it is a `param`, but needs to be defined before the camera is initialised.
    
    unsigned int pll_enabled;                   // Use PLL            : On (1) or Off (0)
    unsigned int opengl_enabled;                // Framecaptures are stored and provided in the callback via OpenGL textures instead of plain memory buffers.
                                                // Note: Captured frame data stays in GPU domain during texture creation.
                                                // Note: Only works in video mode.
#ifdef BUILD_FLASHCAM_WITH_PLL  
    // PLL: Phase Lock Loop ==> Allows the camera (in videomode) to send lightpulse/flash upon frameexposure.
    //                          The Raspberry firmware only support flash when in capture mode, hence this option.
    unsigned int pll_divider;                   // Frequency divider  : > 1                 (framerate / pll_divider = frequency of PLL signal)
    int pll_offset;                             // PWM / Camera offset: > 0                 (microseconds; target time between start of frame -> start of pwm
    float pll_pulsewidth;                       // Pulse width (ms)   : 0 to 1/frequency    (will be rounded according to available accuracy).
    unsigned int pll_fpsreducer_enabled;        // Use FPS-reducer    : On (1) or Off (0)
                                                // --> Tracks real fps of system and reduces the target fps if they do not match
                                                //     Disabling the reducer increases processing speed, but if the target fps is too high, no lock can be obtained.
#endif
#ifdef BUILD_FLASHCAM_WITH_OPENGL  
    unsigned int opengl_packed;                 // 1 or 0. Returned texture is packed: that is, 4x Lumiance is pushed into single RGBA pixel.
#endif
} FLASHCAM_SETTINGS_T;


/*
 * FLASHCAM_PORT_USERDATA_T
 * used internally for communication and status-updates with the camera
 */
typedef struct {
    FLASHCAM_PARAMS_T       *params;            // Pointer to param set
    FLASHCAM_SETTINGS_T     *settings;          // Pointer to setting set
    MMAL_POOL_T             *camera_pool;       // Pool of buffers for camera
    unsigned char           *framebuffer;       // Buffer for final image   
    unsigned int             framebuffer_size;  // Size of buffer
    unsigned int             framebuffer_idx;   // Tracker to stitch imager properly from the camera-callback payloads
    VCOS_SEMAPHORE_T         sem_capture;       // Semaphore indicating the completion of a frame capture 
    //      - In Capturemode: used to indicate completion of frame
    //      - In VideoMode + EGL: used to signal EGL-worker to process frame
    FLASHCAM_CALLBACK_T      callback;          // Callback to user function
#ifdef BUILD_FLASHCAM_WITH_OPENGL  
    MMAL_QUEUE_T            *opengl_queue;      // Pointer to OpenGL Queue
    FLASHCAM_CALLBACK_OPENGL_T  callback_egl;      // OpenGL Callback to user function
#endif
} FLASHCAM_PORT_USERDATA_T;




/*
 * Structure used to add user data to the MMAL_HEADER_BUFFER which is passed to the FlashCamOpenGL functions
 *  While a MMAL_HEADER_BUFFER allows setting a pointer to user_data, the MMAL usage on the RPi uses this to pass
 *   around OMX data pointers, hence we cannot used it. Therefore, FlashCamOpenGL creates a pool of FlashCamBuffers
 *   in which the actual frame data is stored. As the communication between FlashCam and the FlashCamOpenGL-worker thread
 *   works via the thread-safe mmal_queue implementation, a little hack is applied to ensure the below defined structure
 *   can be passed in the mmal_queu:  each structure holds a pointer to the actual frame and a fixed alloaction of a queue-buffer.
 *   This `queuebuf` is initialised with user_data, pointing the a OpenGLBuf-structure in which it is allocated, so, when 
 *   pushing it into the queue, we can retrieve the actual frame data, while tracking our own parameters.
 */


#ifdef BUILD_FLASHCAM_WITH_OPENGL  
typedef struct {
    MMAL_BUFFER_HEADER_T  glb_mmal_buffer;      // glb_mmal_buffer = &FLASHCAM_OPENGL_BUF_T
                                                // so in essence: glb_mmal_buffer == glb_mmal_buffer.user_data->glb_mmal_buffer
    VCOS_SEMAPHORE_T      lock;                 // Buffer locked?
    MMAL_BUFFER_HEADER_T *buffer;               // Frame data
    unsigned int          id;
#ifdef BUILD_FLASHCAM_WITH_PLL  
    bool                  pll_state;            // PLL active in frame?
#endif
} FLASHCAM_OPENGL_BUF_T;
#endif


//Number of samples for the array's for error estimations
#define FLASHCAM_PLL_JITTER 5
#define FLASHCAM_PLL_SAMPLES 10

//Stepresone..
#define FLASHCAM_PLL_STEPRESPONSE_STEPS     2


/*
 * FLASHCAM_INTERNAL_STATE_T
 * PLL results and test data. Only user for intern-tracking
 */
typedef struct { 
    //values managed by FlashCam.cpp
    MMAL_PORT_T                 *port;
    FLASHCAM_SETTINGS_T         *settings;
    FLASHCAM_PARAMS_T           *params;
    FLASHCAM_PORT_USERDATA_T    *userdata;
    
    //values managed by FlashCam_pll.cpp
#ifdef BUILD_FLASHCAM_WITH_PLL  
    bool pll_active;
    bool pll_initialised;
    
    //timing
    float pll_framerate;                        // User set target framerate of camera.
    float pll_pwm_period;                       // Period of the set PWM signal in microseconds. 
    uint64_t pll_starttime_gpu;                 // Starttime of PWM-pulse in microseconds (us), determined by the GPU-clock
    uint64_t pll_startinterval_gpu;             // Accuracy of starttime: starttime \in [starttime, startime+interval]
    
    // state:timing
    uint64_t    pll_last_frametime_gpu;         // Last recorded timestamp of frame.
    float       pll_pid_framerate;              // Framerate proposed by PID controller.
    
    // state:PID
    float       pll_last_error;                 // Last recorded error value: [-0.5 -- 0.5] * 100 = percentage error of period
    int64_t     pll_last_error_us;              // Last recorded error value: [-0.5 -- 0.5] * 100 = percentage error of period
    float       pll_integral;                   // integral of PID tuner
        
    //error. All is in microseconds
    unsigned int pll_error_idx_jitter;                  // jitter index for circular buffers
    unsigned int pll_error_idx_sample;                  // Sample index for circular buffers
    float pll_error_sum;                                // sum of contents of `error[]`
    float pll_error[FLASHCAM_PLL_JITTER];               // Circular buffer. Holds the relative timing error (microseconds) between frame and pwm-pulse.
    float pll_error_avg_last;                           // copy of last element added to `error_avg[]`
    float pll_error_avg_sum;                            // sum of contents of `error_avg[]`
    float pll_error_avg[FLASHCAM_PLL_SAMPLES];          // Circular buffer. Holds the running average error of `error[]` over a window of `FLASHCAM_PLL_SAMPLES_JITTER` elements
                                                        //      Used for jitter reduction
    float pll_error_avg_dt_last;                        // copy of last element added to `error_avg_dt[]`
    float pll_error_avg_dt_sum;                         // sum of contents of `error_avg_dt[]`
    float pll_error_avg_dt[FLASHCAM_PLL_SAMPLES];       // Circular buffer. Holds the derivative of `error_avg[]`
                                                        //      When stable, value should be close to 0. 
    float pll_error_avg_dt_avg_last;                    // copy of last element added to `error_avg_dt_avg[]`
    float pll_error_avg_dt_avg_sum;                     // sum of contents of `error_avg_dt_avg[]`
    float pll_error_avg_dt_avg[FLASHCAM_PLL_SAMPLES];   // Circular buffer. Holds the running average error of `error_avg_dt[]` over a window of `FLASHCAM_PLL_SAMPLES_AVGDT` elements.
                                                        //      Used to determine stability. When zero, error does not deviate and hence stays constant (or oscilates)   
    float pll_error_avg_std_last;                       // copy of last element added to `error_avg_std[]`
    float pll_error_avg_std_sum;                        // sum of contents of `error_avg_std[]`
    float pll_error_avg_std[FLASHCAM_PLL_SAMPLES];      // Circular buffer. Holds the standard deviation of `error_avg[]`
                                                        //      Used to determine stability. When (close to) zero, error-variation is constant.  
#ifdef PLLTUNE
    float P;
    float I;
    float D;
#endif
    
#ifdef STEPRESPONSE
    unsigned int pll_frames;                    // internal counter
    unsigned int pll_frames_next;               // number of frames after which the framerate is changed
    unsigned int pll_step_idx;                  // internal index of selecte framerate
    float        pll_steps[FLASHCAM_PLL_STEPRESPONSE_STEPS]; //array of framerates.
#endif
    
#endif /* BUILD_FLASHCAM_WITH_PLL */

    
    //values managed by FlashCam_pll.cpp
#ifdef BUILD_FLASHCAM_WITH_OPENGL  
    bool            opengl_worker_stop;     // worker action: terminate
    VCOS_THREAD_T   opengl_worker_thread;   // Thread processing queue
    //MMAL texture result
    GLuint          opengl_tex_id;          // Target Texture
    EGLImageKHR     opengl_tex_data;        // Target EGLImage
    std::vector<FLASHCAM_OPENGL_BUF_T>      opengl_buffer_pool;
#endif /* BUILD_FLASHCAM_WITH_OPENGL */
    
} FLASHCAM_INTERNAL_STATE_T;






#endif /* types_h */

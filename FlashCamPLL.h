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

#ifndef FlashCamPLL_h
#define FlashCamPLL_h

#include "types.h"


//Number of samples for the array's for error estimations
#define FLASHCAM_PLL_JITTER 5
#define FLASHCAM_PLL_SAMPLES 10

//Stepresone..
#define FLASHCAM_PLL_STEPRESPONSE_STEPS     2

/*
 * FLASHCAM_PLL_PARAMS_T
 * PLL results and test data. Only user for intern-tracking
 */
typedef struct { 
    //timing
    float framerate;                        // User set target framerate of camera.
    float pwm_period;                       // Period of the set PWM signal in microseconds. 
    uint64_t starttime_gpu;                 // Starttime of PWM-pulse in microseconds (us), determined by the GPU-clock
    uint64_t startinterval_gpu;             // Accuracy of starttime: starttime \in [starttime, startime+interval]
    
    // state:timing
    uint64_t    last_frametime_gpu;         // Last recorded timestamp of frame.
    uint64_t    locktime;                   // Timestamp at which PLL was locked.
    float       pid_framerate;              // Framerate proposed by PID controller.
    
    // state:PID
    float       last_error;                 // Last recorded error value: [-0.5 -- 0.5] * 100 = percentage error of period
    int64_t     last_error_us;              // Last recorded error value: [-0.5 -- 0.5] * 100 = percentage error of period
    float       integral;                   // integral of PID tuner

#ifdef PLLTUNE
    float P;
    float I;
    float D;
#endif
    
#ifdef STEPRESPONSE
    unsigned int frames;                    // internal counter
    unsigned int frames_next;               // number of frames after which the framerate is changed
    unsigned int step_idx;                  // internal index of selecte framerate
    float        steps[FLASHCAM_PLL_STEPRESPONSE_STEPS]; //array of framerates.
#endif
        
    //error. All is in microseconds
    unsigned int error_idx_jitter;                  // jitter index for circular buffers
    unsigned int error_idx_sample;                  // Sample index for circular buffers
    float error_sum;                                // sum of contents of `error[]`
    float error[FLASHCAM_PLL_JITTER];               // Circular buffer. Holds the relative timing error (microseconds) between frame and pwm-pulse.
    float error_avg_last;                           // copy of last element added to `error_avg[]`
    float error_avg_sum;                            // sum of contents of `error_avg[]`
    float error_avg[FLASHCAM_PLL_SAMPLES];          // Circular buffer. Holds the running average error of `error[]` over a window of `FLASHCAM_PLL_SAMPLES_JITTER` elements
                                                    //      Used for jitter reduction
    float error_avg_dt_last;                        // copy of last element added to `error_avg_dt[]`
    float error_avg_dt_sum;                         // sum of contents of `error_avg_dt[]`
    float error_avg_dt[FLASHCAM_PLL_SAMPLES];       // Circular buffer. Holds the derivative of `error_avg[]`
                                                    //      When stable, value should be close to 0. 
    float error_avg_dt_avg_last;                    // copy of last element added to `error_avg_dt_avg[]`
    float error_avg_dt_avg_sum;                     // sum of contents of `error_avg_dt_avg[]`
    float error_avg_dt_avg[FLASHCAM_PLL_SAMPLES];   // Circular buffer. Holds the running average error of `error_avg_dt[]` over a window of `FLASHCAM_PLL_SAMPLES_AVGDT` elements.
                                                    //      Used to determine stability. When zero, error does not deviate and hence stays constant (or oscilates)   
    float error_avg_std_last;                       // copy of last element added to `error_avg_std[]`
    float error_avg_std_sum;                        // sum of contents of `error_avg_std[]`
    float error_avg_std[FLASHCAM_PLL_SAMPLES];      // Circular buffer. Holds the standard deviation of `error_avg[]`
                                                    //      Used to determine stability. When (close to) zero, error-variation is constant.  
} FLASHCAM_PLL_PARAMS_T;


class FlashCamPLL
{
    
private:
    bool                    _error  = false;
    bool                    _active = false;
    
    //Reset GPIO & PWM 
    void resetGPIO();
    
    //reset PLL parameters
    void clearParams();
public:       
    // Constructor / Destructor
    FlashCamPLL();
    ~FlashCamPLL();

    //start/stop PLL mechanism. Functions are invoked when camera is started/stopped in FlashCam.
    int start( MMAL_PORT_T *videoport, FLASHCAM_SETTINGS_T *settings, FLASHCAM_PARAMS_T *params );
    int stop( FLASHCAM_SETTINGS_T *settings, FLASHCAM_PARAMS_T *params );
    
    //update Lock. This function is a callback from FlashCam when a frame is recieved.
    static int update(MMAL_PORT_T *port, FLASHCAM_SETTINGS_T *settings, FLASHCAM_PARAMS_T *params, uint64_t pts);
    
    //settings..
    static void getDefaultSettings( FLASHCAM_SETTINGS_T *settings );
    static void printSettings( FLASHCAM_SETTINGS_T *settings );    
};


#endif /* FlashCamPLL_h */

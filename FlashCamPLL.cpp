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

#include "FlashCamPLL.h"
#include "FlashCam.h"

#include <stdio.h>
#include <wiringPi.h>


//PLL settings

// 19.2Mhz is seemingly the basefrequency of the GPIO?
// -> https://www.raspberrypi.org/documentation/hardware/raspberrypi/schematics/RPI-ZERO-V1_3_reduced.pdf
// -> https://pinout.xyz/pinout/gpclk
// -> https://raspberrypi.stackexchange.com/questions/4906/control-hardware-pwm-frequency
// -> TODO: function to determine real frequency
#define RPI_BASE_FREQ 19200000

// WiringPi pin to which PLL-Laser is connected
// ( equals GPIO-18 = hardware PWM )
#define PLL_PIN 1

// WiringPi pin to which reset is connected
#define RESET_PIN 0


FlashCamPLL::FlashCamPLL() {
    _error   = false;
    
    // Check if we have root access.. otherwise system will crash!
    if (getuid()) {
        fprintf(stderr, "%s: FlashCamPLL/WiringPi requires root. Please run with 'sudo'.\n", __func__);
        _error = true;
        
    //if we are root, init WiringPi
    } else if (wiringPiSetup () == -1) {
        fprintf(stderr, "%s: Cannot init WiringPi.\n", __func__);
        _error = true;
    }
    
    
    // Set pin-functions
    pinMode( PLL_PIN, PWM_OUTPUT );
    pinMode( RESET_PIN, OUTPUT );
    resetGPIO();
}



FlashCamPLL::~FlashCamPLL() {
    //stop pwm
    pwmWrite(PLL_PIN, 0);    
}

void FlashCamPLL::resetGPIO(){
    digitalWrite(RESET_PIN, 1);
    usleep(100);
    digitalWrite(RESET_PIN, 0);
}

void FlashCamPLL::update( FLASHCAM_SETTINGS_T *settings, FLASHCAM_PARAMS_T *params, uint64_t buffertime) {
    //if (settings->verbose)
        //fprintf(stdout, "time:%d\n", buffertime);
}

int FlashCamPLL::start( FLASHCAM_SETTINGS_T *settings, FLASHCAM_PARAMS_T *params ) {

    //initialisation error?
    if (_error) {
        fprintf(stderr, "%s: FlashCamPLL incorrectly initialised.\n", __func__);
        return 1;
    }
    
    if (_active) {
        fprintf(stderr, "%s: PLL already running\n", __func__);
        return 1;
    }
    
    if (settings->pll_enabled) {
        if (settings->verbose)
            fprintf(stdout, "%s: FlashCamPLL starting..\n", __func__);

        // Computations based on:
        // - https://www.raspberrypi.org/forums/viewtopic.php?p=957382#p957382
        // - https://pastebin.com/xTH5jnes
        //
        // From links:
        //    1/f = pwm_range * pwm_clock / RPI_BASE_FREQ
        //
        // RPI_BASE_FREQ : base frequency of PWM system of RPi. 
        //                 Assumed to be fixed. Might be adjustable by user??
        // pwm_range     : is number of bits within a period.
        // pwm_clock     : dividor for RPI_BASE_FREQ to set proper frequency
        // pwm_duty      : number of withs within a period that signal is high.
        //
        // The given dutycycle (pll_duty) is 0-100% with 1% resolution ==> 100 steps.
        // Therefore, error of the PWM signal can be described as 100 / pwm_range.
        //
        // Maximum frequency of the camera is 120Hz
        //
        // According to the datasheets / libraries, maximum values are:
        // pwm_range     : 32 bits
        // pwm_clock     : 12 bits (2 - 4095)
        // pwm_duty      : 32 bits 
        //
        // Assuming an error of max 0.01% :
        // pwm_range = 100/0.01 = 10000
        //
        // With f=120Hz  : pwm_clock = RPI_BASE_FREQ / 120 / 10000 ~   16
        // With f=  1Hz  : pwm_clock = RPI_BASE_FREQ /   1 / 10000 ~ 1920
        //
        // So, with pwm_clock=16, we can satisfy the max error for 1 to 120Hz
        //
        // Validation:
        //
        // pwm_clock = 16
        // pwm_duty  = 0 to 100  ==> steps = 100
        //
        // With f=120Hz  : pwm_range  = RPI_BASE_FREQ / 120 / 16 = 10000
        //               : error      = steps / pwm_range        = 0.01 %
        // With f=  1Hz  : pwm_range  = RPI_BASE_FREQ /   1 / 16 = 1200000
        //               : error      = steps / pwm_range        = 0.000083 %
        //
        // error @ dutycycle: +/- 0.083 ms
        //
        //
        // If: 
        // pwm_clock = 2:
        //
        // With f=120Hz  : pwm_range  = RPI_BASE_FREQ / 120 / 2 = 80000
        //               : error      = steps / pwm_range       = 0.00125 %
        // With f=  1Hz  : pwm_range  = RPI_BASE_FREQ /   1 / 2 = 9600000
        //               : error      = steps / pwm_range       = 0.000010 %
        //
        // error @ dutycycle: +/- 0.01 ms
        //
        // So, it turns out that a clock-dividor of 2 satifies our need easily.
        
        //reset GPIO
        resetGPIO();
        
        // Setup PWM pin
        pinMode( PLL_PIN, PWM_OUTPUT );
        // We do not want the balanced-pwm mode.
        pwmSetMode( PWM_MODE_MS );        
        // Set targeted fps
        settings->pll_freq     = params->framerate;
        float target_frequency = settings->pll_freq / settings->pll_divider;
        
        // clock & range
        unsigned int pwm_clock = 2;
        unsigned int pwm_range = ( RPI_BASE_FREQ / target_frequency ) / pwm_clock; 
        
        // Determine maximum pulse length
        float target_period = 1000.0f/target_frequency; //ms
        
        // Limit pulsewidth to period
        if ( settings->pll_pulsewidth > target_period) 
            settings->pll_pulsewidth = target_period;
        if ( settings->pll_pulsewidth < 0) 
            settings->pll_pulsewidth = 0;        
        
        // Map pulsewidth to RPi-range
        float dutycycle     = settings->pll_pulsewidth / target_period;
        unsigned int pwm_pw = dutycycle * pwm_range; 
                
        // Show computations?
        if ( settings->verbose ) {            
            float real_pw  = ( pwm_pw * target_period) / pwm_range;
            float error    = (settings->pll_pulsewidth - real_pw) / settings->pll_pulsewidth;
            float accuracy = target_period / pwm_range;
            
            fprintf(stdout, "%s: Framerate     : %f\n", __func__, settings->pll_freq);
            fprintf(stdout, "%s: PWM frequency : %f\n", __func__, target_frequency);
            fprintf(stdout, "%s: RPi PWM-clock : %d\n", __func__, pwm_clock);
            fprintf(stdout, "%s: RPi PWM-range : %d\n", __func__, pwm_range);
            fprintf(stdout, "%s: Dutycycle     : %.6f %%\n", __func__, dutycycle * 100);
            fprintf(stdout, "%s: PLL pulsewidth: %.6f ms\n", __func__, settings->pll_pulsewidth);
            fprintf(stdout, "%s: PWM pulsewidth: %d\n", __func__, pwm_pw);
            fprintf(stdout, "%s:     --> in ms : %.6f ms\n", __func__, real_pw);
            fprintf(stdout, "%s: Error (pulse) : %.6f %%\n", __func__, error );
            fprintf(stdout, "%s: Accuracy      : %.6f ms\n", __func__, accuracy );
        }
        
        // Set pwm values
        pwmSetRange(pwm_range);
        pwmWrite(PLL_PIN, pwm_pw);

        // Try to get an accurate starttime 
        // --> we are not in a RTOS, so operations might get interrupted. 
        // --> keep setting clock (resetting pwm) untill we get an accurate estimate
        // --> `accurate` ~ 200us

        unsigned int iter = 0;
        struct timespec t1, t2;
        uint64_t t1_us, t2_us, tdiff;
        
        do {
            // get start-time
            clock_gettime(CLOCK_MONOTONIC, &t1);

            // set clock
            pwmSetClock(pwm_clock);
            // get finished-time
            clock_gettime(CLOCK_MONOTONIC, &t2);

            // compute difference..
            t1_us = ((uint64_t) t1.tv_sec) * 1000000 + ((uint64_t) t1.tv_nsec) / 1000;
            t2_us = ((uint64_t) t2.tv_sec) * 1000000 + ((uint64_t) t2.tv_nsec) / 1000;
            tdiff = t2_us - t1_us;
            
//            fprintf(stdout, "%s: t1         : %" PRIu64 " us\n", __func__, t1_us);
//            fprintf(stdout, "%s: t2         : %" PRIu64 " us\n", __func__, t2_us);
//            fprintf(stdout, "%s: tdiff      : %" PRIu64 " us\n", __func__, tdiff);

            //track iterations
            iter++;
        } while (tdiff > 200);
  
        // started!
        settings->pll_starttime     = t1_us;
        settings->pll_startinterval = tdiff;
        _active = true;
        
        if ( settings->verbose ) {
            fprintf(stdout, "%s: starttime : %" PRIu64 "us\n", __func__, settings->pll_starttime);
            fprintf(stdout, "%s: interval  : %" PRIu64 "us\n", __func__, settings->pll_startinterval);
            fprintf(stdout, "%s: iterations: %d\n", __func__, iter);
        }
            
    } else {
        if (settings->verbose)
            fprintf(stdout, "%s: FlashCamPLL disabled.\n", __func__);
    }
    
    if ( settings->verbose )
        fprintf(stdout, "%s: Succes.\n", __func__);

    return 0;
}

int FlashCamPLL::stop( FLASHCAM_SETTINGS_T *settings, FLASHCAM_PARAMS_T *params ) {
    
    //initialisation error?
    if (_error) {
        fprintf(stderr, "%s: FlashCamPLL incorrectly initialised.\n", __func__);
        return 1;
    }
    
    if (!_active) {
        fprintf(stderr, "%s: PLL not running\n", __func__);
        return 1;
    }
    
    if (settings->verbose)
        fprintf(stdout, "%s: stopping PLL..\n", __func__);

    //stop PWM
    pwmWrite(PLL_PIN, 0);
    //reset fps
    params->framerate = settings->pll_freq;
    //reset active-flag
    _active = false;
    
    if ( settings->verbose )
        fprintf(stdout, "%s: Succes.\n", __func__);

    return 0;
}

void FlashCamPLL::getDefaultSettings(FLASHCAM_SETTINGS_T *settings) {
    settings->pll_enabled       = 0;
    settings->pll_divider       = 1;                            // use camera frequency.
    settings->pll_pulsewidth    = 0.5f / VIDEO_FRAME_RATE_NUM;  // 50% duty cycle with default framerate
    
    //internals
    settings->pll_freq          = 0;
    settings->pll_starttime     = 0;
    settings->pll_startinterval = 0;
}

void FlashCamPLL::printSettings(FLASHCAM_SETTINGS_T *settings) {
    fprintf(stderr, "PLL Enabled   : %d\n", settings->pll_enabled);
    fprintf(stderr, "PLL Divider   : %d\n", settings->pll_divider);
    fprintf(stderr, "PLL Pulsewidth: %0.5f ms\n", settings->pll_pulsewidth);
}

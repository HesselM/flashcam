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
        // SYSTEM PARAMETERS:
        // -----
        // RPI_BASE_FREQ : base frequency of PWM system of RPi. 
        //                 Assumed to be fixed. Might be adjustable by user??
        // pwm_range     : is number of bits within a period.
        // pwm_clock     : divider for RPI_BASE_FREQ to set PWM base frequency.
        //                 Higher basefrequency means more pulses per PWM-Period and hence more accurate signal.
        // pwm_pw        : number of bits within a period (pwm_range) that signal is high.
        //
        // USER DEFINED PARAMETERS:
        // -----
        // frequency     : targeted frequency (Hz) at which a full pwm cycle (high & low) should be produced.
        //                 This frequency is a real-number limited between 0.0 to 120.0 Hz.
        // pulsewidth    : number of milliseconds that the pwm pulse should be high. 
        //                 When 0, a flat line (LOW) is observed. 
        //                 When it equals 1000/frequency, a flat line (HIGH) is observed.
        //                 pulsewidth is clipped to 0 and 1000/frequency.
        //
        // USER PARAMETERS -> SYSTEM PARAMETERS
        // ------
        //
        // According to the datasheets / libraries, maximum values are:
        // pwm_range     : unsigned int, 32 bits (2 - 4294967295)
        // pwm_clock     : unsigned int, 12 bits (2 - 4095)
        // pwm_pw        : unsigned int, 32 bits (0 - 4294967295) 
        //
        // user values:
        // frequency     : float, 0.0 - 120.0
        // pulsewidth    : float, 0.0 - 1000.0/frequency
        //
        // These values are related as following:
        // 
        //    1/frequency = pwm_range * pwm_clock / RPI_BASE_FREQ
        //    dutycycle   = pulsewidth * frequency / 1000
        //    pwm_pw      = dutycycle * pwm_range
        // 
        // Or, assuming pwm_clock and frequency are given values:
        //
        //    (uint)  pwm_range   = RPI_BASE_FREQ / ( pwm_clock * frequency )
        //    (float) dutycycle   = pulsewidth * frequency / 1000
        //    (uint)  pwm_pw      = dutycycle * pwm_range
        //
        // From these computations it can be observed that truncation (float -> int)
        //  occures several steps. Hence, the resulting PWM signal can have a slight
        //  error / inaccuracy when compared to the required settings. 
        // 
        // Some observations:
        // - a large pwm_range will result in a more `true` pwm_pw. 
        // - a low clock and frequency will give a larger pwm_range.
        // - the smallest clock value (pwm_clock) is 2.
        // - the largest range value (pwm_range) is 4294967295.
        // - the maximum frequency is 120Hz.
        //
        // Hence, we can compute the minimal frequency and pulsewidth-resolution.
        //
        // minimal frquency:
        // frequency_min            = 1 / (pwm_range * pwm_clock / RPI_BASE_FREQ)
        // frequency_min            = RPI_BASE_FREQ / ( pwm_range * pwm_clock )
        //                          = 19200000 / ( 4294967295 * 2 )
        //                          ~ 0.00224 Hz.
        //
        // minimal resolution:
        // range_max @120Hz         = RPI_BASE_FREQ / ( pwm_clock * frequency )
        //                          = 19200000 / ( 2 * 120 )
        //                          = 80000
        // resolution @120Hz        = period / pwm_range;
        //                          = 1000 / (frequency * pwm_range);
        //                          = 1000 / (120 * 80000);
        //                          ~ 0.0001042 ms
        //                          ~ 0.1042    us
        //
        // resolution error         = 100 * (resolution / period)
        //                          = 100 * ((period / pwm_range) / period)
        //                          = 100 / pwm_range
        //  - @120Hz                = 0.00125%     
        //  - @0.00224Hz            = 0.0000000234%
        //
        // When pwm_clock is increases with a factor `x`:
        //  - minimal resolution error will increase with `x`.
        //  - minimal frequency will decrease with a factor `x`.
        //
        // Therefore, pwm_clock = 2 is sufficient for PLL.
        
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
        unsigned int pwm_range = RPI_BASE_FREQ / ( target_frequency * pwm_clock ); 
        
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
            float resolution = target_period / pwm_range;
            
            fprintf(stdout, "%s: PLL/PWL SETTINGS\n", __func__);
            fprintf(stdout, " - Framerate     : %f\n", settings->pll_freq);
            fprintf(stdout, " - PWM frequency : %f\n", target_frequency);
            fprintf(stdout, " - PWM resolution: %.6f ms\n", resolution );
            fprintf(stdout, " - RPi PWM-clock : %d\n", pwm_clock);
            fprintf(stdout, " - RPi PWM-range : %d\n", pwm_range);
            fprintf(stdout, " - PLL Dutycycle : %.6f %%\n", dutycycle * 100);
            fprintf(stdout, " - PLL Pulsewidth: %.6f ms\n", settings->pll_pulsewidth);
            fprintf(stdout, " - PWM Pulsewidth: %d / %d\n", pwm_pw, pwm_range);
            fprintf(stdout, " -     --> in ms : %.6f ms\n", real_pw);
            fprintf(stdout, " - Pulsewidth err: %.6f %%\n", error );
        }
        
        // Set pwm values
        pwmSetRange(pwm_range);
        pwmWrite(PLL_PIN, pwm_pw);

        // Try to get an accurate starttime 
        // --> we are not in a RTOS, so operations might get interrupted. 
        // --> keep setting clock (resetting pwm) untill we get an accurate estimate
        //
        // When investigating the sourcecode of WiringPi, it shows that `pwmSetClock`
        //  already has a buildin-delays of atleast 110us + 1us. 
        // Therefore `max_locktime` should be at least 111us. 200us is a safe bet.
        unsigned int max_locktime_us = 200;

        //loop trackers..
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
            
            //track iterations
            iter++;
        } while (tdiff > max_locktime_us);
  
        // By now we have a lock and PWM has started!
        
        // As `pwmSetClock` takes a mimimum time of 111 us to activate the PWM, 
        //  we can adjust starttime and narrow down the start-interval to more accurate values.
        settings->pll_starttime     = t1_us + 111;
        settings->pll_startinterval = tdiff - 111;
        
        // PLL is activated..
        _active = true;
        if ( settings->verbose ) {
            //clock resolution
            struct timespec tres;
            clock_getres(CLOCK_MONOTONIC, &tres);
            uint64_t res =  ((uint64_t) tres.tv_sec) * 1000000000 + ((uint64_t) tres.tv_nsec);            
            fprintf(stdout, "%s: PLL/PWM start values\n", __func__);
            fprintf(stdout, " - Starttime     : %" PRIu64 "us\n", settings->pll_starttime);
            fprintf(stdout, " - Resolution    : %" PRIu64 "ns\n", res);
            fprintf(stdout, " - Interval      : %" PRIu64 "us\n", settings->pll_startinterval);
            fprintf(stdout, " - Iterations    : %d\n", iter);
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

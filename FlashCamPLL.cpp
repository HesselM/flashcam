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
#define RPI_BASE_FREQ 1920000
// WiringPi pin on which PLL-Laser is connected
// ( equals GPIO-18 = hardware PWM )
#define PLL_PIN 1


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
}



FlashCamPLL::~FlashCamPLL() {
    
    
}

void FlashCamPLL::update( FLASHCAM_SETTINGS_T *settings, FLASHCAM_PARAMS_T *params, uint64_t buffertime) {
    if (settings->verbose)
        fprintf(stdout, "time:%d\n", buffertime);
}

int FlashCamPLL::start( FLASHCAM_SETTINGS_T *settings, FLASHCAM_PARAMS_T *params ) {

    //initialisation error?
    if (_error) {
        fprintf(stderr, "%s: FlashCamPLL incorrectly initialised.\n", __func__);
        return 1;
    }
    
    if (settings->pll_enabled) {
        if (settings->verbose)
            fprintf(stdout, "%s: FlashCamPLL enabled.\n", __func__);

        //setup PWM pin
        pinMode( PLL_PIN, PWM_OUTPUT );
        pwmSetMode( PWM_MODE_MS );

        
    /*
        //wiringPiISR (1, INT_EDGE_RISING, &pwm_int);
        int sclock = 19;
        uint32_t range = 19.2e6/sclock/40 - 19;
        pwmSetRange (range);
        pwmWrite (1, 3);
        struct timespec t1, t2;
        uint64_t tdiff = 0;
        int i = 0;
        
        
        //get lock on starttime
        do
        {
            clock_gettime(CLOCK_MONOTONIC, &t1);
            pwmSetClock (19);
            clock_gettime(CLOCK_MONOTONIC, &t2);
            tdiff = (uint64_t)t2.tv_sec*1000000 + t2.tv_nsec / 1000 - (uint64_t)t1.tv_sec*1000000 - t1.tv_nsec / 1000;
            ++i;
        } while (tdiff > 200);
        std::cout << "tdiff=" << tdiff << "  i=" << i << "\n";
        
        pwm_start_time = ((uint64_t)t1.tv_sec)*1000000 + t1.tv_nsec / 1000;
        pwm_period = range*sclock/19.2e6;
        
        std::this_thread::sleep_for(std::chrono::microseconds(10000));
        std::thread mmal_thread (start_camera);
     */
            
    } else {
        if (settings->verbose)
            fprintf(stdout, "%s: FlashCamPLL disabled.\n", __func__);
    }
    
    return 0;
}

int FlashCamPLL::stop( FLASHCAM_SETTINGS_T *settings ) {
    
    //initialisation error?
    if (_error) {
        fprintf(stderr, "%s: FlashCamPLL incorrectly initialised.\n", __func__);
        return 1;
    }

    
    return 0;
}

void FlashCamPLL::getDefaultSettings(FLASHCAM_SETTINGS_T *settings) {
    settings->pll_enabled   = 0;
    settings->pll_freq      = VIDEO_FRAME_RATE_NUM;
    settings->pll_duty      = 50; // 50% duty cycle
}

void FlashCamPLL::printSettings(FLASHCAM_SETTINGS_T *settings) {
    fprintf(stderr, "PLL Enabled  : %d\n", settings->pll_enabled);
    fprintf(stderr, "PLL Frequency: %d\n", settings->pll_freq);
    fprintf(stderr, "PLL Dutycycle: %d\n", settings->pll_duty);
}

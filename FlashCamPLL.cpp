//
//  FlashCam_PLL.cpp
//  
//
//  Created by Hessel van der Molen on 02/05/17.
//
//

#include "FlashCamPLL.h"
#include "FlashCam.h"

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



void FlashCamPLL::update( FLASHCAM_SETTINGS_T *settings, FLASHCAM_PARAMS_T *params, uint64_t buffertime) {
    if (settings->verbose)
        fprintf(stdout, "time:%d\n", buffertime);
}

void FlashCamPLL::start( FLASHCAM_SETTINGS_T *settings, FLASHCAM_PARAMS_T *params ) {
    
    
    
    
    
    
}

void FlashCamPLL::stop() {
    
    
    
}

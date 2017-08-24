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


//
// Tool for tuning PLL
// - Manual tuning 
// - Auto-test PID-paramaters for `LOOPTEST_ITER` iterations, where each iteration lasts `LOOPTEST_TIME` seconds
//      => results are written to csv file to analyse stability
// - Auto-test different P-values. Only works when `#define LOOPTES_P` is uncommented.
//      => per P-value a different log is created
//

#include "FlashCam.h"
#include "terminal.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>


// comment if loop-testing should only do 1 P-value. If multiple values need to be checked, uncomment LOOPTEST_P.
// -> it runs for each value of `P` from the displayed value, with P_STEPS to P_MAX the LOOPTEST program. 
//     generating a logfile for LOOPTEST_ITER iterations of a duration of LOOPTEST_TIME seconds
#define LOOPTEST_P

// 0-7
#define SENSORMODE   5
// Pixels
#define FRAME_WIDTH  1640
#define FRAME_HEIGHT 922
// microseconds - shutterspeed of imager
#define SHUTTERSPEED 15000
// Hz - framerate of imager
#define FRAMERATE    30.0f
// 1/0 - Do we use PLL?
#define USEPLL        1
// What frequency? (FRAMERATE / PLLDIVIDER = PLLFREQUENCY)
#define PLLDIVIDER    2
// milliseconds - Pulsewidth
// (mode 5 requires a minimum of 17.8ms: see rpi/camera documentation)
#define PLLPULSEWIDTH 17.8
// millieseconds - Time that framecapture trails behing pulse.
#define PLLOFFSET     0

//used keys
#define CHAR_Q 113
#define CHAR_R 114
#define CHAR_I 105
#define CHAR_J 106
#define CHAR_O 111
#define CHAR_K 107
#define CHAR_P 112
#define CHAR_L 108
#define CHAR_T 116

//Looptest defination
#define LOOPTEST_ITER 50
#define LOOPTEST_TIME 5.0f

//P-value test params
#ifdef LOOPTEST_P
#define LOOPTEST_P_MAX 10.0f
#define LOOPTEST_P_STEP 1.0f
#endif


//TIMER PARAMS
static double prev_tick;
static double time_sum;
static int frames;

//FlashCam settings&parameters
static FLASHCAM_PLL_PARAMS_T *pllparams; //pointer to PLL internal parameters
static FLASHCAM_SETTINGS_T     settings; //copy of settings structure

//Looptest params
static unsigned int      looptest_iteration;
static std::stringstream looptest_logname_stream;
static std::string       looptest_logname;
static std::ofstream     looptest_logfile;
static bool              looptest_loopdone;

//state
static bool active;             //flashcam active?
static bool active_looptest;    //testloop active?


void initScreen() {
    //clear screen
    CLEAR;
    printf( TPOS( 1,1) "FLASHCAM: PLL TUNING           " TCL);
    printf( TPOS( 2,1) "-------------------------------" TCL);
    printf( TPOS( 3,1) " q = Stop tuning / quit program" TCL);
    printf( TPOS( 4,1) " r = Restart PLL/camera        " TCL);
#ifdef LOOPTEST_P
    printf( TPOS( 5,1) " t = Looptest (%4d x %4.1fs) - P_LOOP: step:%5.3f/max:%5.3f " TCL, LOOPTEST_ITER, LOOPTEST_TIME, LOOPTEST_P_STEP, LOOPTEST_P_MAX);
#else 
    printf( TPOS( 5,1) " t = Looptest (%4d x %4.1fs)   " TCL, LOOPTEST_ITER, LOOPTEST_TIME);
#endif    
    printf( TPOS( 6,1) "-------------------------------" TCL);
}

void printScreen() {     
    printf( TPOS( 7,1) " i = P+      j = P-\n          " TCL);
    printf( TPOS( 8,1) " o = I+      k = I-\n          " TCL);
    printf( TPOS( 9,1) " p = D+      l = D-\n          " TCL);
    printf( TPOS(10,1) "-------------------------------" TCL);
    printf( TPOS(11,1) " P: %06.2f   I: %08.4f   D: %06.2f          " TCL, pllparams->P, pllparams->I, pllparams->D);
    printf( TPOS(12,1) "-------------------------------             " TCL);
    printf( TPOS(13,1) " Error                                      " TCL);
    printf( TPOS(14,1) "  - Percentual        : %11.5f %%           " TCL, 100 * pllparams->last_error);
    printf( TPOS(15,1) "  - Absolute          : %11" PRId64 " us    " TCL, pllparams->last_error_us);
    printf( TPOS(16,1) "  - jitter (=avg)     : %11.5f us (%d)      " TCL, pllparams->error_sum             / FLASHCAM_PLL_JITTER , FLASHCAM_PLL_JITTER);
    printf( TPOS(17,1) "  - dt(jitter)        : %11.5f us (%d)      " TCL, pllparams->error_avg_dt_sum      / FLASHCAM_PLL_SAMPLES, FLASHCAM_PLL_SAMPLES);
    printf( TPOS(18,1) "  - avg(dt(jitter))   : %11.5f us (%d)      " TCL, pllparams->error_avg_dt_avg_sum  / FLASHCAM_PLL_SAMPLES, FLASHCAM_PLL_SAMPLES);
    printf( TPOS(19,1) "  - std(jitter)       : %11.5f us (%d)      " TCL, pllparams->error_avg_std_sum     / FLASHCAM_PLL_SAMPLES, FLASHCAM_PLL_SAMPLES);
    printf( TPOS(20,1) "  - Integral          : %11.5f us           " TCL, pllparams->integral);
    printf( TPOS(21,1) " Frequency                                  " TCL);
    printf( TPOS(22,1) "  - PWM               : %11.5f Hz           " TCL, 1000000.0f / pllparams->pwm_period );
    printf( TPOS(23,1) "  - Camera Target     : %11.5f Hz           " TCL, pllparams->framerate);
    printf( TPOS(24,1) "  - PID proposed      : %11.5f Hz           " TCL, pllparams->pid_framerate);
    printf( TPOS(25,1) "  - CPU Measured      : %11.5f Hz           " TCL, frames/time_sum);
    printf( TPOS(26,1) " PWM interval         : %11" PRId64 " us    " TCL, pllparams->startinterval_gpu);
    printf( TPOS(27,1) " Sensor Mode          : %11d                " TCL, SENSORMODE);
    printf( TPOS(28,1) " Width x Height       : %4d x %4d Pixels    " TCL, settings.width, settings.height);
    printf( TPOS(29,1) " Looptest                                   " TCL);
    printf( TPOS(30,1) "  - Iteration         : %11d                " TCL, looptest_iteration);
    printf( TPOS(31,1) "  - Elapsed           : %11.5f s            " TCL, time_sum);
    printf( TPOS(32,1) "  - Logfile           : %s                  " TCL, looptest_logname.c_str());
}

void logData(bool header) {
    if (header) {
        looptest_logfile << "iteration";
        looptest_logfile << ",frame";
        looptest_logfile << ",time";
        looptest_logfile << ",P";
        looptest_logfile << ",pwm_interval";
        looptest_logfile << ",pid_framerate";
        looptest_logfile << ",cpu_framerate";
        looptest_logfile << ",err_percent";
        looptest_logfile << ",err_absolute";
        looptest_logfile << ",err_jitter";
        looptest_logfile << ",err_dt_jitter";
        looptest_logfile << ",err_avg_dt_jitter";
        looptest_logfile << ",err_std_jitter";
        looptest_logfile << "\n";
    } else {    
        looptest_logfile << ""  << looptest_iteration;
        looptest_logfile << "," << frames;
        looptest_logfile << "," << time_sum;
        looptest_logfile << "," << pllparams->P;
        looptest_logfile << "," << pllparams->startinterval_gpu;
        looptest_logfile << "," << pllparams->pid_framerate;
        looptest_logfile << "," << (frames/time_sum);
        looptest_logfile << "," << pllparams->last_error;
        looptest_logfile << "," << pllparams->last_error_us;
        looptest_logfile << "," << (pllparams->error_sum            / FLASHCAM_PLL_JITTER);
        looptest_logfile << "," << (pllparams->error_avg_dt_sum     / FLASHCAM_PLL_SAMPLES);
        looptest_logfile << "," << (pllparams->error_avg_dt_avg_sum / FLASHCAM_PLL_SAMPLES);
        looptest_logfile << "," << (pllparams->error_avg_std_sum    / FLASHCAM_PLL_SAMPLES);
        looptest_logfile << " \n";
    }
}

//CALLBACK SETTINGS
void flashcam_callback(unsigned char *frame, int w, int h) {
    //Measure update frequency
    double tick    = cv::getTickCount();
    if (prev_tick != 0) {
        double time_elapsed = double ( tick - prev_tick ) / double ( cv::getTickFrequency() ); //time in second
        time_sum += time_elapsed;
        frames++;
    }
    prev_tick = tick;   
    
    //Are we running multiple tests?
    if (active_looptest && !looptest_loopdone) {
        //write data to file
        logData(false);
        
        //Iteration done?
        if (time_sum > LOOPTEST_TIME )
            looptest_loopdone = true;
    } 
    printScreen();
}


void updateLogName() {
    looptest_logname_stream.str("");
    looptest_logname_stream.clear();
    looptest_logname_stream << "P" << std::fixed << std::setprecision(2) << pllparams->P;
    looptest_logname_stream << "I" << std::fixed << std::setprecision(2) << pllparams->I;
    looptest_logname_stream << "D" << std::fixed << std::setprecision(2) << pllparams->D;
    looptest_logname_stream << ".csv";
    looptest_logname = looptest_logname_stream.str();
}


void initFlashCam() {
    //get default params & settings
    FlashCam::getDefaultSettings( &settings );
    
    FLASHCAM_PARAMS_T params = {};
    FlashCam::getDefaultParams( &params );
    
    //update params
    settings.width=FRAME_WIDTH;
    settings.height=FRAME_HEIGHT;
    settings.verbose=0;
    settings.update=0;
    settings.mode=FLASHCAM_MODE_VIDEO;
    settings.sensormode=SENSORMODE;
    
    //create camera with params
    FlashCam::get().setSettings( &settings );
    //set callback
    FlashCam::get().setFrameCallback( &flashcam_callback );
    
    //set camera params
    FlashCam::get().setExposureMode(MMAL_PARAM_EXPOSUREMODE_SPORTS);
    FlashCam::get().setFrameRate(FRAMERATE);
    FlashCam::get().setShutterSpeed(SHUTTERSPEED);
    FlashCam::get().setRotation(0);
    
    //PLL parameters
    FlashCam::get().setPLLEnabled(USEPLL);
    FlashCam::get().setPLLDivider(PLLDIVIDER); //every frame
    FlashCam::get().setPLLPulseWidth(PLLPULSEWIDTH);
    FlashCam::get().setPLLOffset(PLLOFFSET);
    
    //get & print params
    FlashCam::get().getParams( &params , true);   
    //FlashCam::printParams( &params ); 
    FlashCam::get().getSettings( &settings);   
    //FlashCam::printSettings( &settings ); 

    //retrieve PLL internal parameters
    FlashCam::get().getPLLParams(&pllparams);
}


void resetFlashCam() {
    float P,I,D;
    
    if (active) {
        P = pllparams->P;
        I = pllparams->I;
        D = pllparams->D;
        FlashCam::get().stopCapture();
        
        //reset singleton
        FlashCam::get().clear(); 
        initFlashCam();
        //reset PID
        pllparams->P = P;
        pllparams->I = I;
        pllparams->D = D;
    }
    
    //reset parameters
    time_sum    = 0;
    frames      = 0;
    prev_tick   = 0;
    active      = false;
}


int main(int argc, const char **argv) {
    initFlashCam(); 

    // setup screen
    terminal_clear();
    initScreen();
    printScreen();
    
    //not active..
    active          = false;
    active_looptest = false;
    
    //reset looptest params
    looptest_iteration  = 0;
    looptest_logname    = "";
    looptest_loopdone   = false;
    
    //looper variables 
    char  c = 0;                //char from user input

    //Loop while no exit-request is send
    while (c != CHAR_Q) {
        
        //check keyboard updates
        if (read(STDIN_FILENO, &c, sizeof(c)) == sizeof(c)) {            
            switch (c) {
                case CHAR_I: //i
                    if (!active_looptest)
                        pllparams->P += 0.1;
                    break;
                case CHAR_J: //j 
                    if (!active_looptest)
                        pllparams->P -= 0.1;
                    break;
                case CHAR_O: //o
                    if (!active_looptest)
                        pllparams->I += 0.0001;
                    break;
                case CHAR_K: //k
                    if (!active_looptest)
                        pllparams->I -= 0.0001;
                    break;
                case CHAR_P: //p
                    if (!active_looptest)
                        pllparams->D += 0.1;
                    break;
                case CHAR_L: //l
                    if (!active_looptest)
                        pllparams->D -= 0.1;
                    break;
                case CHAR_R: //restart
                    if (!active_looptest) {
                        resetFlashCam();
                        //start capture
                        active = true;
                        FlashCam::get().startCapture();   
                    }
                    break;
                
                case CHAR_T: //start test
                    resetFlashCam();
                
                    //update logname
                    updateLogName();
                
                    //close log    
                    if (looptest_logfile.is_open())
                        looptest_logfile.close();
                    //new logfile + print header
                    looptest_logfile.open(looptest_logname);
                    logData(true);
                
                    //start testloop
                    looptest_iteration  = 0;
                    active              = true;
                    active_looptest     = true;
                    FlashCam::get().startCapture();   

                    break;
            }
            
            
            if (!active) 
                printScreen();
        }
        
        if (active_looptest && looptest_loopdone) {
            //reset + next iteration
            resetFlashCam();
            looptest_iteration++;

            //start new loop?
            looptest_loopdone = false;
            if (looptest_iteration < LOOPTEST_ITER) {
                initScreen();
                active          = true;
                active_looptest = true;
                FlashCam::get().startCapture();                   
            }  else {
                //close logdile
                if (looptest_logfile.is_open())
                    looptest_logfile.close();
                //reset flags
                active          = false;
                active_looptest = false;                

#ifdef LOOPTEST_P
                // Do we start new iteration with new P-value?
                pllparams->P += LOOPTEST_P_STEP;
                if (pllparams->P <= LOOPTEST_P_MAX) {
                    //new logfile name
                    updateLogName();
                    //new logfile + print header
                    looptest_logfile.open(looptest_logname);
                    logData(true);
                    //start new iteration
                    looptest_iteration  = 0;
                    active              = true;
                    active_looptest     = true;
                    initScreen();
                    FlashCam::get().startCapture();                    
                }
#endif
            }
            printScreen();
        }

        //0.01s sleep to reduce load
        usleep(100000); 
    }

    if (active)
        FlashCam::get().stopCapture();   

    if (looptest_logfile.is_open())
        looptest_logfile.close();

    
    // reset input adjustments
    terminal_restore();
    
    return 0;
}

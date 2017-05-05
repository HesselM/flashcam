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

#include "FlashCam.h"

#include <stdlib.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <time.h>
#include <unistd.h>

static cv::Mat Y;
static cv::Mat U;
static cv::Mat V;

static double prev;
static int  frames;
static double sum;

void flashcam_callback(unsigned char *frame, int w, int h) {
    //fprintf(stdout, "callback: %p (%d x %d)\n", frame, w, h);
    //update opencv image
    //Y.data = (uchar*) &(frame[0]);
    //U.data = (uchar*) &(frame[(w*h*1)>>0]);
    //V.data = (uchar*) &(frame[(w*h*5)>>2]);
    memcpy(Y.data, &(frame[0])         , w*h);
    memcpy(U.data, &(frame[(w*h*1)>>0]), (w>>1)*(h>>1));
    memcpy(V.data, &(frame[(w*h*5)>>2]), (w>>1)*(h>>1));
    
    //timing
    double tick    = cv::getTickCount();
    double elapsed = double ( tick - prev ) / double ( cv::getTickFrequency() ); //time in second
    sum += elapsed;
    frames++;
    prev = tick;
    
    if ((frames % 120) == 0)
        fprintf(stdout, "Timing: %f (avg: %f; frames: %d; fps:%f)\n", elapsed, sum/frames, frames, 1.0/(sum/frames));
}


int main(int argc, const char **argv) {
    fprintf(stdout, "\n -- VIDEO-TEST: PLL -- \n\n");
    
    //get default params & settings
    FLASHCAM_SETTINGS_T settings = {};
    FlashCam::getDefaultSettings( &settings );
    
    FLASHCAM_PARAMS_T params = {};
    FlashCam::getDefaultParams( &params );
    //update params
    settings.width=640;
    settings.height=480;
    settings.verbose=1;
    settings.update=0;
    settings.mode=FLASHCAM_MODE_VIDEO;

    //create camera with params
    FlashCam::get().setSettings( &settings );
    
    //set callback
    FlashCam::get().setFrameCallback( &flashcam_callback );
    
    //set camera params
    FlashCam::get().setExposureMode(MMAL_PARAM_EXPOSUREMODE_SPORTS);
    //camera.setShutterSpeed(350);
    //camera.setFlashMode(MMAL_PARAM_FLASH_ON);
    //camera.setDenoise(0);
    //camera.setISO(800);
    FlashCam::get().setRotation(270);
    
    float pll_fps = 30;
    int   pll_div = 1;
    //float pll_pw  = 500.0f / (pll_fps / pll_div); //50% dutycycle
    //float pll_pw  = 33.333333; // ms
    float pll_pw  = 0.124; // ms
    
    FlashCam::get().setFrameRate(pll_fps);

    //enable PLL
    FlashCam::get().setPLLEnabled(1);
    FlashCam::get().setPLLDivider(pll_div);
    FlashCam::get().setPLLPulseWidth(pll_pw);
    
    //get & print params
    FlashCam::get().getParams( &params , true);   
    FlashCam::printParams( &params ); 
    
    //get & print params
    FlashCam::get().getSettings( &settings);   
    FlashCam::printSettings( &settings ); 
    
    //create openCV window
    Y.create( settings.height, settings.width, CV_8UC1 );
    cv::namedWindow( "Y", cv::WINDOW_AUTOSIZE );
    U.create( settings.height >> 1, settings.width >> 1, CV_8UC1 );
    //cv::namedWindow( "U", cv::WINDOW_AUTOSIZE );
    V.create( settings.height >> 1, settings.width >> 1, CV_8UC1 );
    //cv::namedWindow( "V", cv::WINDOW_AUTOSIZE );
    
    //init timings
    prev   = cv::getTickCount();
    frames = 0;
    sum    = 0;
    
    //start stream image
    FlashCam::get().startCapture();   
    
    //wait
    int show =  0;  //show image with opencv?
    int time =  1;  //total time (seconds) of streaming/running
    int fps  =  5;  //refreshrate of window
    
    //refresh loop
    for (int i=0; i<(time*fps); i++) {
        if (show) {
            cv::imshow ("Y", Y);
            cv::imshow ("U", U);
            cv::imshow ("V", V);
            cv::waitKey(1000/fps);
        } else
            usleep(1000000/fps);
    }
    
    //start stream image
    FlashCam::get().stopCapture();   
    
    //show params
    FlashCam::get().getParams( &params , true);   
    FlashCam::printParams( &params ); 
    //show settings
    FlashCam::get().getSettings( &settings);   
    FlashCam::printSettings( &settings ); 

    return 0;
}

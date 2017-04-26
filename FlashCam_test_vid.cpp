
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
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <time.h>

static cv::Mat Y;
static cv::Mat U;
static cv::Mat V;

void flashcam_callback(unsigned char *frame, int w, int h) {
    fprintf(stderr, "callback: %p (%d x %d)\n", frame, w, h);
    //update opencv image
    Y.data = (uchar*) &(frame[0]);
    U.data = (uchar*) &(frame[(w*h*1)>>0]);
    V.data = (uchar*) &(frame[(w*h*5)>>2]);
    
}


int main(int argc, const char **argv) {
    fprintf(stdout, "\n -- VIDEO-TEST -- \n\n");
    
    //get default params & settings
    FLASHCAM_SETTINGS_T settings = {};
    FlashCam::getDefaultSettings( &settings );
    
    FLASHCAM_PARAMS_T params = {};
    FlashCam::getDefaultParams( &params );
    
    //update params
    settings.width=320;
    settings.height=240;
    settings.verbose=1;
    settings.update=0;
    settings.mode=FLASHCAM_MODE_CAPTURE;
    
    //create camera with params
    FlashCam camera = FlashCam( &settings );
    
    //set callback
    camera.setFrameCallback( &flashcam_callback );
    
    //set camera params
    camera.setExposureMode(MMAL_PARAM_EXPOSUREMODE_SPORTS);
    camera.setShutterSpeed(350);
    camera.setFlashMode(MMAL_PARAM_FLASH_ON);
    camera.setDenoise(0);
    camera.setISO(800);
    camera.setRotation(270);
    
    //get & print params
    camera.getParams( &params , true);   
    FlashCam::printParams( &params ); 
    
    //get & print params
    camera.getSettings( &settings);   
    FlashCam::printSettings( &settings ); 
    
    //create openCV window
    Y.create( settings.height, settings.width, CV_8UC1 );
    cv::namedWindow( "Y", cv::WINDOW_AUTOSIZE );
    U.create( settings.height >> 1, settings.width >> 1, CV_8UC1 );
    cv::namedWindow( "U", cv::WINDOW_AUTOSIZE );
    V.create( settings.height >> 1, settings.width >> 1, CV_8UC1 );
    cv::namedWindow( "V", cv::WINDOW_AUTOSIZE );
    
    double start   = 0;
    double elapsed = 0;
    double sum     = 0;
    
    for (int i=0; i<10; i++) {
        
        start=cv::getTickCount();
        
        //capture image
        camera.startCapture();   
        
        elapsed = double ( cv::getTickCount() - start ) / double ( cv::getTickFrequency() ); //time in second
        sum    += elapsed;
        
        fprintf(stderr, "elapsed: %f (avg: %f)\n", elapsed, sum/double(i+1));
        
        cv::imshow ("Y", Y);
        cv::imshow ("U", U);
        cv::imshow ("V", V);
        cv::waitKey(0);
    }
    
    return 0;
}

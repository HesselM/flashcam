
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

#include "FlashCam_test.h"

#include "FlashCam.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

static cv::Mat cvimage;

void test(unsigned char *frame, int w, int h) {
    fprintf(stderr, "callback: %p (%d x %d)\n", frame, w, h);
    
    //build cv image
    cvimage.create( h * 1.5, w, CV_8UC1 );
    cvimage.data = (uchar*) frame;
    
    cv::imshow ("cvwindow", cvimage);
    cv::waitKey(0);
    cvimage.release();
}


int main(int argc, const char **argv) {
    //create camera
    FlashCam camera = FlashCam();
    camera.setFrameCallback(&test);
    
    //get & print params
    FLASHCAM_PARAMS_T params = {};
    camera.getAllParams( &params , true);
    FlashCam::printParams( &params );

    camera.getAllParams( &params , false);
    FlashCam::printParams( &params );

    //create openCV window
    cv::namedWindow( "cvwindow", cv::WINDOW_AUTOSIZE );

    //create image
    camera.capture();
    
    //cv::imshow ("cvwindow", cvimage);
    //cv::waitKey(0);
    //cvimage.release();
    
    //get image
    camera.capture();
    
    //cv::imshow ("cvwindow", cvimage);
    //cv::waitKey(0);
    //cvimage.release();

    //get image
    camera.capture();
    
    //cv::imshow ("cvwindow", cvimage);
    //cv::waitKey(0);
    //cvimage.release();

    return 0;
}

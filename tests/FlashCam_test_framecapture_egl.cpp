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
// Capture frame from sensor in VideoMode
// Frames are displayed via OpenGL Textures, useing OpenCV
// Write frames to file for e.g. calibration.
//

#include "FlashCam.h"
#include "FlashCamEGL.h"
#include "FlashCamEGLUtil.h"

#include <stdlib.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <time.h>
#include <unistd.h>

#include <GLES/gl.h>
#include <GLES/glext.h>
#include "GLES2/gl2.h"
#include "EGL/egl.h"
#include "EGL/eglext.h"
#include "EGL/eglext_brcm.h"

#define  eglcheck() assert(glGetError() == 0)


// 0-7
#define SENSORMODE   5
// Pixels
#define FRAME_WIDTH  1640
#define FRAME_HEIGHT 922
// microseconds
#define SHUTTERSPEED 15000

//used keys
#define CHAR_ESC 27
#define CHAR_P   112

static cv::Mat Y;
static unsigned int captured;

void flashcam_callback(GLuint texid, EGLImageKHR *img, int w, int h) {
    fprintf(stdout, "frame (id:%d %dx%d)\n", texid, w, h);
    
    // copy frame to opencv-structure
    if (captured == 0) {
        fprintf(stdout, "..PROCESSING..\n");
        captured = 1;

        fprintf(stdout, "frame (%d)\n", captured);
        GLuint result_texid = FlashCamEGL::createTexture(); eglcheck();
        
        FlashCamEGL::textureOES2rgb(texid, result_texid); eglcheck();
        glBindTexture(GL_TEXTURE_2D, result_texid); eglcheck();

        fprintf(stdout, "Starting to read..\n");

        glReadPixels(0, 0,
                     w, h,
                     GL_RGBA,
                     GL_UNSIGNED_BYTE,
                     Y.data);
        FlashCamEGL::eglDispError();
        eglcheck();

        fprintf(stdout, "Texture read\n");

        //done processing
        captured = 2;
    } else {
        fprintf(stdout, "..BLOCK..\n");
    }
}


int main(int argc, const char **argv) {
    fprintf(stdout, "\n -- FRAMECAPTURE OpenGL -- \n\n");
    fprintf(stdout, "ESC  : stop\n");
    fprintf(stdout, "ENTER: next frame\n");
    fprintf(stdout, "P    : write frame to file\n");
    fprintf(stdout, " ------------------ \n");
    
    //get default params & settings
    FLASHCAM_SETTINGS_T settings = {};
    FlashCam::getDefaultSettings( &settings );
    
    FLASHCAM_PARAMS_T params = {};
    FlashCam::getDefaultParams( &params );
    
    //update settings
    settings.width=FRAME_WIDTH;
    settings.height=FRAME_HEIGHT;
    settings.verbose=1;
    settings.update=0;
    settings.mode=FLASHCAM_MODE_VIDEO;
    settings.sensormode=SENSORMODE;
    
    //Use OpenGL
    settings.useOpenGL=1;
    
    //create camera with params
    FlashCam::get().setSettings( &settings );
    //set callback
    FlashCam::get().setFrameCallback( &flashcam_callback );
    
    //set camera params
    FlashCam::get().setExposureMode(MMAL_PARAM_EXPOSUREMODE_SPORTS);
    FlashCam::get().setShutterSpeed(SHUTTERSPEED);
    FlashCam::get().setRotation(0);
    FlashCam::get().setFrameRate(1);        //1 Hz
    
    //PLL parameters
    FlashCam::get().setPLLDivider(1);       //every frame
    FlashCam::get().setPLLPulseWidth(1000); //1000ms = 100% dutycycle @ 1Hz
    FlashCam::get().setPLLOffset(0);        //no offset
    
    //get & print params
    FlashCam::get().getParams( &params , true);   
    FlashCam::printParams( &params ); 
    
    //get & print settings
    FlashCam::get().getSettings( &settings);   
    FlashCam::printSettings( &settings ); 
    
    //create openCV window
    Y.create( settings.height, settings.width, CV_8UC4 );
    cv::namedWindow( "Y", cv::WINDOW_NORMAL );
    
    //intit
    char key = 0;
    captured = 0; //0=capturing, 1=captured, 2=copied data to memory
    
    //variable to store frame to file
    char filename[20];
    unsigned int frameid = 0;
    unsigned int toggle = 0;
    
    //continue while capturing isn't the ESC-key
    while (key != CHAR_ESC) {
        //start stream image
        FlashCam::get().setPLLEnabled(1);
        FlashCam::get().startCapture();   
        
        //wait until captured
        while (captured != 2)
            usleep(100000); //sleep 100ms
        
        //stop capturing
        FlashCam::get().stopCapture();   
        
        fprintf(stdout, "Awaiting key..\n");

        //display image
        cv::imshow ("Y", Y);
        key = (char) cv::waitKey(0);   
        fprintf(stdout, "Key pressed: %c\n", key);
        
        //write to file?
        if (key == CHAR_P) {
            sprintf(filename, "frame-%d.jpg", frameid);
            imwrite(filename, Y);
            fprintf(stdout, "Written frame: %s\n", filename);
            frameid++;
        }
        
        //reset capture
        captured = 0;
    } 
    
    return 0;
}

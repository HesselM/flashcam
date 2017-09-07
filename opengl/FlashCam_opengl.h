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

#ifndef FlashCam_opengl_h
#define FlashCam_opengl_h

#include "FlashCam_types.h"

#include <GLES/gl.h>
#include <GLES/glext.h>
#include "GLES2/gl2.h"
#include "EGL/egl.h"
#include "EGL/eglext.h"
#include "EGL/eglext_brcm.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/util/mmal_connection.h"


namespace FlashCamOpenGL {
    
    typedef struct {
        bool                        update;             // worker action: update texture
        bool                        stop;               // worker action: terminate
        MMAL_PORT_T                 *port;              // Video port reference
        FLASHCAM_PORT_USERDATA_T    *userdata;          // Reference to userdata
        VCOS_THREAD_T               worker_thread;      // Thread processing queue
        
        // texture/EGLImage to be written to
        GLuint                      texture;            // Target Texture
        EGLImageKHR                 img;                // Target EGLImage
        
        // Buffer used but EGLImage
        MMAL_BUFFER_HEADER_T *buffer_img;
        
        //OpenGL settings
        EGLDisplay display;                 /// The current EGL display
        EGLSurface surface;                 /// The current EGL surface
        EGLContext context;                 /// The current EGL context
        
    } FLASHCAM_EGL_t;
    
    int init();
    int start(MMAL_PORT_T *port, FLASHCAM_PORT_USERDATA_T *userdata);
    void stop();
    void destroy();    
}

#endif /* FlashCam_opengl_h */

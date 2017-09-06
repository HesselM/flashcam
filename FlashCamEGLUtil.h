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

#ifndef FlashCamEGLUtil_h
#define FlashCamEGLUtil_h

#include "types.h"
#include "FlashCamEGL.h"

#include <GLES/gl.h>
#include <GLES/glext.h>
#include "GLES2/gl2.h"
#include "EGL/egl.h"
#include "EGL/eglext.h"
#include "EGL/eglext_brcm.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/util/mmal_connection.h"

#define  eglcheck() assert(glGetError() == 0)
#define reglcheck() assert(EGL_FALSE != result)

namespace FlashCamEGL {
    
    //initialize OpenGL
    void initOpenGL(FLASHCAM_EGL_t* state);
    void clearOpenGL();
    
    //transform a MMAL-buffer into a OpenGL texture.
    // ==> Used internal state representation. 
    //      Result is given via UserCall-back. 
    //      See FlashCamEGL_framecapture.cpp
    void mmalbuf2TextureOES_internal(MMAL_BUFFER_HEADER_T *buffer);
    
    //transform OES texture (=Lumiance only) to RGB
    void textureOES2rgb(GLuint input_texid, GLuint result_texid);
    void textureRGBblur(GLuint input_texid, GLuint result_texid);      

    //generate new texture. Texture-ID is returned.
    GLuint createTexture();
    
    //print OpenGL errors (if existing)
    void eglDispError();
}


#endif /* FlashCamEGLUtil_h */

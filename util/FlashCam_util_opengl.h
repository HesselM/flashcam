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

#ifndef FlashCam_util_opengl_h
#define FlashCam_util_opengl_h

#include "FlashCam_types.h"
#include "FlashCam_opengl.h"

#include <string>

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

namespace FlashCamUtilOpenGL {
    
    typedef struct {
        unsigned int default_width;
        unsigned int default_height;
        EGLDisplay display;
        EGLSurface surface;
        EGLContext context;
    } FLASHCAM_OPENGL_STATE_T;
    
    
    //user callback
    typedef void (*FLASHCAM_CALLBACK_OPENGL_DESTROY_T) (void);

    //initialize/stop OpenGL for use with FlashCam lib.
    void init(int w, int h);    
    bool isInitialised();
    
    void destroyEGLImage(EGLImageKHR *targetimg);
    void destroy();
    
    //transform a MMAL-buffer into a OpenGL texture.
    // ==> Used internal state representation. 
    //      Result is given via UserCall-back. 
    //      See FlashCam_test_vid_opengl_framecapture.cpp
    void mmalbuf2TextureOES(MMAL_BUFFER_HEADER_T *buffer, GLuint targetid, EGLImageKHR *targetimg);
    
    //transform OES texture (=Lumiance only) to RGB
    void textureOES2rgb(GLuint input_texid, GLuint result_texid);
    void textureRGBblur(GLuint input_texid, GLuint result_texid);   
    void textureRGBsobel(GLuint input_texid, GLuint result_texid);
    //Apply-shader program to texture and fetch result as texture.
    void texture2texture(GLuint input_texid, GLenum input_target,  GLuint result_texid, GLuint progid);   


    // request & bind OpenCL texture with given type.
    // Note: this does not do any initialisation. ID is returned.
    GLuint generateTexture(GLenum type);
    
    //create RGBA TEXTURE_2D image with size `default_width` x `default_height` and RGBA8 internal format.
    // Note: no data is assigned; ID is returned.
    GLuint createTexture();
    
    //create RGBA texture with specified width, height and internal dataformat.
    // Note: no data is assigned; ID is returned.
    GLuint createTexture(int w, int h, GLenum dataformat);
    
    //Retrieve OpenGL state parameter to allow user-defined framebuffers and all.
    // This however requires a call back in case of OpenGL being deinitialised, requiring existing textures and objects to be released.
    FLASHCAM_OPENGL_STATE_T getOpenGLState( FLASHCAM_CALLBACK_OPENGL_DESTROY_T callback);
    
    //create a shader program of a vertics & fragment shader.
    GLuint createShaderProgram(std::string v, std::string f);    

    //print OpenGL errors (if existing)
    void eglDispError();
}


#endif /* FlashCam_util_opengl_h */

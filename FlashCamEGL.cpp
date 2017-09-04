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

#include "FlashCamEGL.hpp"

#include <assert.h>

#define  check() assert(glGetError() == 0)


namespace FlashCamEGL {

    typdef struct {
        bool                        update;             // worker action: update texture
        bool                        stop;               // worker action: terminate
        MMAL_PORT_T                 *port;              // Video port reference
        FLASHCAM_PORT_USERDATA_T    *userdata;          // Reference to userdata
        VCOS_THREAD_T               *worker_thread;     // Thread processing queue
        
        //OpenGL settings
        EGLDisplay display;                 /// The current EGL display
        EGLSurface surface;                 /// The current EGL surface
        EGLContext context;                 /// The current EGL context

    } FLASHCAM_EGL_t;

    //struct tracking the state of all OpenGL / EGL threads, settings, etc.
    FLASHCAM_EGL_t state;
    
    
    
    
    
    
    
    
    //worker thread: processed captured frames
    static int worker(void *arg) {
        FLASHCAM_EGL_t* state = arg;

        MMAL_BUFFER_HEADER_T *buffer;
        MMAL_STATUS_T st;
        
//        vcos_log_trace("%s: port %p", VCOS_FUNCTION, preview_port);

        
        
        //init OpenGL
        // ...

        
        
        // process buffer
        while (!state->stop) {
            
            
            
            // Send empty buffers to camera port
            while ((buffer = mmal_queue_get(state->userdata->camera_pool->queue)) != NULL) {
                if ((status = mmal_port_send_buffer(state->port, buffer)) != MMAL_SUCCESS) {
                    vcos_log_error("Failed to send buffer to %s", state->port->name);
                }
            }
            
            
            // Process elements from buffer
            while ((buffer = mmal_queue_get(state->userdata->opengl_queue)) != NULL) {
                
                // OPAQUE ==> TEXTURE
                EGLClientBuffer *data = buffer->data;
                
                //get texture id
                Gluint texid;
                glBindTexture(GL_TEXTURE_EXTERNAL_OES, &texid); check();
                
                //only luminance (Y)
                EGLImageKHR egl_image = eglCreateImageKHR(state->display, EGL_NO_CONTEXT, EGL_IMAGE_BRCM_MULTIMEDIA_Y, data, NULL);
                check();
                
                glEGLImageTargetTexture2DOES(GL_TEXTURE_EXTERNAL_OES, &egl_image);
                check();

                
                /* Now return the PREVIOUS MMAL buffer header back to the camera preview. */
                if (state->preview_buf)
                    mmal_buffer_header_release(state->preview_buf);
                
                state->preview_buf = buf;
                
            }
        }
        
        // Make sure all buffers are returned on exit //
        while ((buffer = mmal_queue_get(state->userdata->opengl_queue)) != NULL)
            mmal_buffer_header_release(buffer);
        
//        /* Tear down GL */
//        state->ops.gl_term(state);
//        vcos_log_trace("Exiting preview worker");
//        return NULL;
        
    }

    
    
    
    
    
    int init() {
        //setup videocore-logging and semaphores
        
        VCOS_STATUS_T status;
        vcos_init();

        vcos_log_register("FlashCam-EGL", VCOS_LOG_CATEGORY);
        vcos_log_set_level(VCOS_LOG_CATEGORY, VCOS_LOG_INFO); //VCOS_LOG_WARN
        vcos_log_trace("%s", VCOS_FUNCTION);
        
        //status = vcos_semaphore_create(&state->capture.start_sem, "glcap_start_sem", 1);
        //if (status != VCOS_SUCCESS) {
        //    vcos_log_error("%s: failed", VCOS_FUNCTION);
        //    return -1;
        //}
        
        status = vcos_semaphore_create(&state->capture.completed_sem, "glcap_completed_sem", 0);
        if (status != VCOS_SUCCESS) {
            vcos_log_error("%s: failed", VCOS_FUNCTION);
            return -1;
        }
        
        
        
        return 0;
    }
    
    
    
    int start(MMAL_PORT_T *port, FLASHCAM_PORT_USERDATA_T *userdata) {
        VCOS_STATUS_T status;

        //set basic settings
        FlashCamEGL::state.stop     = false;
        FlashCamEGL::state.update   = true;
        FlashCamEGL::state.port     = port;
        FlashCamEGL::state.userdata = userdata;

        //start worker thread
        status = vcos_thread_create(&(FlashCamEGL::state.worker_thread), "FlashCamEGL-worker", NULL, FlashCamEGL::worker, &(FlashCamEGL::state));
        if (status != VCOS_SUCCESS)
            vcos_log_error("%s: Failed to start `FlashCamEGL-worker` (%d)", VCOS_FUNCTION, status);

        //return
        return (status == VCOS_SUCCESS ? 0 : -1);
    }

    
    
    
    void stop() {        
        // STOP SIGNAL
        if (!FlashCamEGL::state.stop) {
            //vcos_log_trace("Stopping GL preview");
            FlashCamEGL::state.stop = true;
            vcos_thread_join(&(FlashCamEGL::state.worker_thread), NULL);
        }
    }
    
    
    
}

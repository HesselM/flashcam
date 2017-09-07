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

#include "FlashCam_opengl.h"
#include "FlashCam_util_opengl.h"

#include <assert.h>
#include <bcm_host.h>

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>


namespace FlashCamOpenGL {
    
    //internal-struct tracking the state of all OpenGL / EGL threads, settings, etc.
    static FLASHCAM_EGL_t state;
        
    //worker thread: processed captured frames
    static void *worker(void *arg) {
        FLASHCAM_EGL_t* state = (FLASHCAM_EGL_t*) arg;
        
        MMAL_BUFFER_HEADER_T *buffer;
        MMAL_STATUS_T status;
        
        //init OpenGL
        initOpenGL(state);
        
        // process buffer
        while (!state->stop) {
            
            //wait for update
            vcos_semaphore_wait(&(state->userdata->sem_capture));
            
            // Do we need to continue or are we done?
            // --> as we are using countin-semaphores, only process a single buffer
            if ((!state->stop) && ((buffer = mmal_queue_get(state->userdata->opengl_queue)) != NULL)) {      
                
                mmal_buffer_header_mem_lock(buffer);

                // OPAQUE ==> TEXTURE
                mmalbuf2TextureOES_internal(buffer);
                
                //callback user..
                if (state->userdata->callback_egl)
                    state->userdata->callback_egl( state->texture, &(state->img) , state->userdata->settings->width , state->userdata->settings->height);
                
                //unlock & release buffer
                mmal_buffer_header_mem_unlock(buffer);
                mmal_buffer_header_release(buffer);
                
                //send buffer pack to pool
                if ((status = mmal_port_send_buffer(state->port, buffer)) != MMAL_SUCCESS) {
                    vcos_log_error("Failed to send buffer to %s", state->port->name);
                }
            }
        }
        
        fprintf(stdout, "Worker: releasing..\n");
        // Make sure all buffers are returned on exit
        while ((buffer = mmal_queue_get(state->userdata->opengl_queue)) != NULL)
            mmal_buffer_header_release(buffer);   
        fprintf(stdout, "Worker: releasing.. done\n");
    }
    
    
    int init() {
        fprintf(stdout, "EGL:init..\n");

        //setup videocore-logging and semaphores
        bcm_host_init();

        VCOS_STATUS_T status;
        vcos_init();

        vcos_log_register("FlashCam-EGL", VCOS_LOG_CATEGORY);
        vcos_log_set_level(VCOS_LOG_CATEGORY, VCOS_LOG_INFO); //VCOS_LOG_WARN
        vcos_log_trace("%s", VCOS_FUNCTION);
        return 0;
    }
    
    
    
    int start(MMAL_PORT_T *port, FLASHCAM_PORT_USERDATA_T *userdata) {
        VCOS_STATUS_T status;

        fprintf(stdout, "EGL:starting..\n");

        //set basic settings
        FlashCamOpenGL::state.stop     = false;
        FlashCamOpenGL::state.update   = true;
        FlashCamOpenGL::state.port     = port;
        FlashCamOpenGL::state.userdata = userdata;
        FlashCamOpenGL::state.img      = EGL_NO_IMAGE_KHR;
                        
        //clear queue..
        fprintf(stdout, "- resetting queue..\n");
        while (mmal_queue_get(FlashCamOpenGL::state.userdata->opengl_queue) != NULL);                

        //reset semaphore
        fprintf(stdout, "- resetting semaphore..\n");
        while (vcos_semaphore_trywait(&(FlashCamOpenGL::state.userdata->sem_capture)) != VCOS_EAGAIN);
        
        
        //start worker thread
        fprintf(stdout, "- starting worker..\n");
        status = vcos_thread_create(&(FlashCamOpenGL::state.worker_thread), "FlashCamOpenGL-worker", NULL, FlashCamOpenGL::worker, &(FlashCamOpenGL::state));
        if (status != VCOS_SUCCESS)
            vcos_log_error("%s: Failed to start `FlashCamOpenGL-worker` (%d)", VCOS_FUNCTION, status);

        fprintf(stdout, "- done.\n");

        //return
        return (status == VCOS_SUCCESS ? 0 : -1);
    }

    
    
    
    void stop() {      
        fprintf(stdout, "EGL:stopping..\n");
        
        // STOP SIGNAL
        if (!FlashCamOpenGL::state.stop) {
            //vcos_log_trace("Stopping GL preview");
            fprintf(stdout, "- worker is running\n");

            //notify worker we are done. 
            //  As the worker blocks due to the sempahore, we need to set the status and post an update
            FlashCamOpenGL::state.stop = true;
            vcos_semaphore_post(&(FlashCamOpenGL::state.userdata->sem_capture));
            
            fprintf(stdout, "- Waiting for worker\n");

            //Wait for worker to terminate.
            vcos_thread_join(&(FlashCamOpenGL::state.worker_thread), NULL);
            
            //clear OpenGL
            FlashCamOpenGL::clearOpenGL();
        
            fprintf(stdout, "- Done\n");
        }
    }
        
    void destroy() {
        fprintf(stdout, "EGL:destroying..\n");

        //vcos_semaphore_delete(&(FlashCamOpenGL::sem_captyr));
    }    
}

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
        
    //private & static parameterlist
    static FLASHCAM_INTERNAL_STATE_T *_state;

    //worker thread: processed captured frames
    static void *worker(void *arg) {
        FLASHCAM_INTERNAL_STATE_T* state = (FLASHCAM_INTERNAL_STATE_T*) arg;
        
        FLASHCAM_OPENGL_BUF_T   *glb;
        MMAL_BUFFER_HEADER_T    *glb_mmal_buffer;
        MMAL_BUFFER_HEADER_T    *buffer;
        MMAL_STATUS_T status;
        
        //init OpenGL
        FlashCamUtilOpenGL::init(state->settings->width, state->settings->height);
        //init texture;
        state->opengl_tex_id = FlashCamUtilOpenGL::generateTexture(GL_TEXTURE_EXTERNAL_OES);
        
        // process buffer
        while (!state->opengl_worker_stop) {
            
            //wait for update
            vcos_semaphore_wait(&(state->userdata->sem_capture));
            
            // Do we need to continue or are we done?
            // --> as we are using countin-semaphores, only process a single buffer
            if ((!state->opengl_worker_stop) && ((glb_mmal_buffer = mmal_queue_get(state->userdata->opengl_queue)) != NULL)) {      
                
                //get frame data.
                glb     = (FLASHCAM_OPENGL_BUF_T*) glb_mmal_buffer->user_data;
                buffer  = glb->buffer; 
                mmal_buffer_header_mem_lock(buffer);

                // OPAQUE ==> TEXTURE
                FlashCamUtilOpenGL::mmalbuf2TextureOES(buffer, state->opengl_tex_id, &(state->opengl_tex_data));
                
                //callback user..
                if (state->userdata->callback_egl)
                    state->userdata->callback_egl( state->opengl_tex_id, state->userdata->settings->width, state->userdata->settings->height, buffer->pts, glb->pll_state);
                
                
                //release opengl buffer back to pool
                vcos_semaphore_post(&(glb->lock));
                
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
    
    
    int init(FLASHCAM_INTERNAL_STATE_T *state) {
        fprintf(stdout, "EGL:init..\n");
        FlashCamOpenGL::_state = state;
        
        //setup videocore-logging and semaphores
        bcm_host_init();
        vcos_init();
        vcos_log_register("FlashCam-EGL", VCOS_LOG_CATEGORY);
        vcos_log_set_level(VCOS_LOG_CATEGORY, VCOS_LOG_INFO); //VCOS_LOG_WARN
        vcos_log_trace("%s", VCOS_FUNCTION);
        
        return 0;
    }
    
    
    void initOpenGLBufferPool() {
        //setup opengl buffer pool
        for (unsigned int i=0; i<FlashCamOpenGL::_state->port->buffer_num; i++) {
            //new buffer
            FLASHCAM_OPENGL_BUF_T b;
            //create lock
            if (vcos_semaphore_create(&b.lock, "FlashCam_opengl_bufferlock", 1) != VCOS_SUCCESS)
                vcos_log_error("%s: Failed to create semaphore", __func__);
            //set data
            b.buffer                = NULL;
            b.id                    = i+10;
            //Note: address of glb_mmal_buffer can only be set after item is pushed to vector. 
#ifdef BUILD_FLASHCAM_WITH_PLL  
            b.pll_state             = false;
#endif
            //push to vector
            FlashCamOpenGL::_state->opengl_buffer_pool.push_back(b);
            fprintf(stdout, "Created buffer in OpenGL pool (%d) \n", i);
        }
    }
    
    void destroyOpenGLBufferPool() {
        while (!(FlashCamOpenGL::_state->opengl_buffer_pool.empty())) {
            //remove element from pool
            FLASHCAM_OPENGL_BUF_T b = FlashCamOpenGL::_state->opengl_buffer_pool.back();
            FlashCamOpenGL::_state->opengl_buffer_pool.pop_back();

            //clear semaphore
            vcos_semaphore_delete(&b.lock);
        }
    }

    //try to get a buffer. Return NULL if none are available
    FLASHCAM_OPENGL_BUF_T* getOpenGLBuffer() {
        for (unsigned int i=0; i<FlashCamOpenGL::_state->opengl_buffer_pool.size(); i++) {
            if (vcos_semaphore_trywait(&(FlashCamOpenGL::_state->opengl_buffer_pool.at(i).lock)) == VCOS_SUCCESS) {
                FLASHCAM_OPENGL_BUF_T* b = &(FlashCamOpenGL::_state->opengl_buffer_pool.at(i));
                //set proper address of `glb_mmal_buffer`
                b->glb_mmal_buffer.user_data = b;
                return b;    
            } else {
                fprintf(stdout, "Locked.. (%d) \n", i);
            }
        }
        return NULL;
    }
    
    int start() {
        VCOS_STATUS_T status;

        fprintf(stdout, "EGL:starting..\n");

        //set basic settings
        FlashCamOpenGL::_state->opengl_worker_stop  = false;
        FlashCamOpenGL::_state->opengl_tex_id       = 0;
        FlashCamOpenGL::_state->opengl_tex_data     = EGL_NO_IMAGE_KHR;
                
        //init pool
        destroyOpenGLBufferPool();
        initOpenGLBufferPool();
        
        //clear queue..
        fprintf(stdout, "- resetting queue..\n");
        while (mmal_queue_get(FlashCamOpenGL::_state->userdata->opengl_queue) != NULL);                

        //reset semaphore
        fprintf(stdout, "- resetting semaphore..\n");
        while (vcos_semaphore_trywait(&(FlashCamOpenGL::_state->userdata->sem_capture)) != VCOS_EAGAIN);
        
        //start worker thread
        fprintf(stdout, "- starting worker..\n");
        status = vcos_thread_create( &(FlashCamOpenGL::_state->opengl_worker_thread), "FlashCamOpenGL-worker", NULL, FlashCamOpenGL::worker, FlashCamOpenGL::_state);
        if (status != VCOS_SUCCESS)
            vcos_log_error("%s: Failed to start `FlashCamOpenGL-worker` (%d)", VCOS_FUNCTION, status);

        fprintf(stdout, "- done.\n");

        //return
        return (status == VCOS_SUCCESS ? 0 : -1);
    }

    
    
    
    void stop() {      
        fprintf(stdout, "EGL:stopping..\n");
        
        // STOP SIGNAL
        if (!FlashCamOpenGL::_state->opengl_worker_stop) {
            //vcos_log_trace("Stopping GL preview");
            fprintf(stdout, "- worker is running\n");

            //notify worker we are done. 
            //  As the worker blocks due to the sempahore, we need to set the status and post an update
            FlashCamOpenGL::_state->opengl_worker_stop = true;
            vcos_semaphore_post(&(FlashCamOpenGL::_state->userdata->sem_capture));
            
            fprintf(stdout, "- Waiting for worker\n");

            //Wait for worker to terminate.
            vcos_thread_join(&(FlashCamOpenGL::_state->opengl_worker_thread), NULL);
            
            //clear OpenGL
            FlashCamUtilOpenGL::destroyEGLImage(&(FlashCamOpenGL::_state->opengl_tex_data));            
            FlashCamUtilOpenGL::destroy();
        
            //destroy bufferpool
            destroyOpenGLBufferPool();
            
            fprintf(stdout, "- Done\n");
        }
    }
        
    void destroy() {
        fprintf(stdout, "EGL:destroying..\n");

        //vcos_semaphore_delete(&(FlashCamOpenGL::sem_captyr));
    }    
}

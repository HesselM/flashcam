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

#include "FlashCamEGL.h"

#include <assert.h>
#include <bcm_host.h>

#define  check() assert(glGetError() == 0)
#define rcheck() assert(EGL_FALSE != result)


namespace FlashCamEGL {

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

    //struct tracking the state of all OpenGL / EGL threads, settings, etc.
    static FLASHCAM_EGL_t state;
    
    
    static void initOpenGL(FLASHCAM_EGL_t* state) {
        EGLBoolean result;
        
        /**** OPENGL SETTINGS ****/
        static const EGLint configAttr[] = {
            EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
            EGL_BLUE_SIZE, 8,
            EGL_GREEN_SIZE, 8,
            EGL_RED_SIZE, 8,
            EGL_ALPHA_SIZE, 8,
            EGL_DEPTH_SIZE, 1,
            EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
            EGL_NONE
        };
        
        static const EGLint pbufferAttr[] = {
            EGL_WIDTH,  (const EGLint) state->userdata->settings->width,
            EGL_HEIGHT, (const EGLint) state->userdata->settings->height,
            EGL_NONE,
        };
        
        static const EGLint contextAttr[] = {
            EGL_CONTEXT_CLIENT_VERSION, 2,
            EGL_NONE
        };
        
        /*************************/
        
        // 1. Get an EGL display connection
        state->display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
        assert(state->display != EGL_NO_DISPLAY);
        check();
        
        // 2. Initialize the EGL display connection
        EGLint major, minor;
        result = eglInitialize(state->display, &major, &minor);
        rcheck();
        check();
        
        // 3. Select an appropriate configuration
        EGLint numConfigs;
        EGLConfig config;
        result = eglChooseConfig(state->display, configAttr, &config, 1, &numConfigs);
        rcheck();
        check();
        //eglDispConfig(config);
        
        // 4. Create a surface
        state->surface = eglCreatePbufferSurface(state->display, config, pbufferAttr);
        assert(state->surface != EGL_NO_SURFACE);
        check();
        
        // 5. Bind the API
        result = eglBindAPI(EGL_OPENGL_ES_API);
        rcheck();
        check();
        
        // 6. Create a context and make it current
        state->context = eglCreateContext(state->display, config, EGL_NO_CONTEXT, contextAttr);
        assert(state->context != EGL_NO_CONTEXT);
        check();
        
        // 7. Make it current
        result = eglMakeCurrent(state->display, state->surface, state->surface, state->context);
        rcheck();
        check();
        
        //No depth in textures/renderer
        glDisable(GL_DEPTH_TEST); check();
        //Allow alpha-channel
        glEnable (GL_BLEND); check();
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);check();
        
        glEnable(GL_TEXTURE_EXTERNAL_OES);
        check();
        
        //create texture
        glGenTextures(1, &state->texture);check();
        glBindTexture(GL_TEXTURE_EXTERNAL_OES, state->texture);check();
        //Scaling: nearest (=no) interpolation for scaling down and up.
        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER, GL_NEAREST);check();
        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER, GL_NEAREST);check();
        //Wrapping: repeat. Only use (s,t) as we are using a 2D texture
        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_S, GL_REPEAT);check();
        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_T, GL_REPEAT);check();

        // 8. Set background color and clear buffers
        //glClearColor(1.0f, 0.0f, 0.0f, 1.0f);
        //glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        //check();
    }
    
    //worker thread: processed captured frames
    static void *worker(void *arg) {
        FLASHCAM_EGL_t* state = (FLASHCAM_EGL_t*) arg;

        MMAL_BUFFER_HEADER_T *buffer;
        MMAL_STATUS_T status;
        
//        vcos_log_trace("%s: port %p", VCOS_FUNCTION, preview_port);

        //init OpenGL
        initOpenGL(state);
        
        // process buffer
        while (!state->stop) {
            
            //wait for update
            vcos_semaphore_wait(&(state->userdata->sem_capture));
            
            // Do we need to continue?
            if (!state->stop) {
                
                // Send empty buffers to camera port
                while ((buffer = mmal_queue_get(state->userdata->camera_pool->queue)) != NULL) {
                    if ((status = mmal_port_send_buffer(state->port, buffer)) != MMAL_SUCCESS) {
                        vcos_log_error("Failed to send buffer to %s", state->port->name);
                    }
                }
                
                
                // Process elements from buffer
                while ((buffer = mmal_queue_get(state->userdata->opengl_queue)) != NULL) {
                    
                    // OPAQUE ==> TEXTURE
                    
                    //set texture id
                    glBindTexture(GL_TEXTURE_EXTERNAL_OES, state->texture); check();
                    
                    //destroy previous image
                    if (state->img != EGL_NO_IMAGE_KHR) {
                        eglDestroyImageKHR(state->display, state->img);
                        state->img = EGL_NO_IMAGE_KHR;
                    }

                    //Create new image
                    state->img = eglCreateImageKHR(state->display, EGL_NO_CONTEXT, EGL_IMAGE_BRCM_MULTIMEDIA_Y, (EGLClientBuffer) buffer->data, NULL);
                    check();
                    
                    glEGLImageTargetTexture2DOES(GL_TEXTURE_EXTERNAL_OES, state->img);
                    check();

                    // release and lock buffer of current EGLImage
                    if (state->buffer_img)
                        mmal_buffer_header_release(state->buffer_img);
                    state->buffer_img = buffer;
                    
                    //callback user..
                    if (state->userdata->callback_egl)
                        state->userdata->callback_egl( state->texture, &(state->img) , state->userdata->settings->width , state->userdata->settings->height);
                }
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

        //set basic settings
        FlashCamEGL::state.stop     = false;
        FlashCamEGL::state.update   = true;
        FlashCamEGL::state.port     = port;
        FlashCamEGL::state.userdata = userdata;
        FlashCamEGL::state.img      = EGL_NO_IMAGE_KHR;
        
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

            //notify worker we are done. 
            //  As the worker blocks due to the sempahore, we need to set the status and post an update
            FlashCamEGL::state.stop = true;
            vcos_semaphore_post(&(FlashCamEGL::state.userdata->sem_capture));
            
            //Wait for worker to terminate.
            vcos_thread_join(&(FlashCamEGL::state.worker_thread), NULL);
            
            //destroy texture
            glDeleteTextures(1, &FlashCamEGL::state.texture);

            //destroy image
            if (FlashCamEGL::state.img != EGL_NO_IMAGE_KHR) {
                eglDestroyImageKHR(FlashCamEGL::state.display, FlashCamEGL::state.img);
                FlashCamEGL::state.img = EGL_NO_IMAGE_KHR;
            }
            
            //detroy openGL
            eglMakeCurrent(FlashCamEGL::state.display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
            eglDestroyContext(FlashCamEGL::state.display, FlashCamEGL::state.context);
            eglDestroySurface(FlashCamEGL::state.display, FlashCamEGL::state.surface);
            eglTerminate(FlashCamEGL::state.display);
        }
    }
        
    void destroy() {
        //vcos_semaphore_delete(&(FlashCamEGL::sem_captyr));
    }


    
}

//
//  FlashCamEGLUtil.cpp
//  
//
//  Created by Hessel van der Molen on 05/09/17.
//
//

#include "FlashCamEGL.h"
#include "FlashCamEGLUtil.h"

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>

namespace FlashCamEGL {
    
    //Pointer to internal-state structure
    // ==> Used for future calls for rendering
    static FLASHCAM_EGL_t *state;
    
    static GLuint progid_oes2rgb;
    static GLuint progid_rgbblur;
    static GLuint vbufid;
    static GLuint fbufid;
    
    static std::vector<GLuint> textures;
    
    
    // 2D vertex shader.    
#define SRC_VSHADER_2D \
    "attribute vec2 position;\n" \
    "varying vec2 texcoord;\n" \
    "void main(void) {\n" \
    "   texcoord = position * vec2(0.5) + vec2(0.5);\n" \
    "   gl_Position = vec4(position, 0.0, 1.0);\n" \
    "}\n"
    
    // EGL_IMAGE_BRCM_MULTIMEDIA_Y is a one byte per pixel greyscale GL_LUMINANCE.
    // TODO-OPTIMISATION: 4-grayscale pixels to 1-rgba pixel
#define SRC_FSHADER_OES2RGB \
    "#extension GL_OES_EGL_image_external : require\n" \
    "uniform samplerExternalOES tex;\n" \
    "varying vec2 texcoord;\n" \
    "void main(void) {\n" \
    "    gl_FragColor = texture2D(tex, texcoord);\n" \
    "}\n"    
    
    //blur shader
#define SRC_FSHADER_RGBBLUR \
    "varying vec2 texcoord;\n" \
    "uniform sampler2D tex;\n" \
    "uniform vec2 texelsize;\n" \
    "void main(void) {\n" \
    "    vec4 col = vec4(0);\n" \
    "    float total_added = 0.0;\n" \
    "    for(int xoffset = -2; xoffset <= 2; xoffset++) {\n" \
    "        for(int yoffset = -2; yoffset <= 2; yoffset++) {\n" \
    "            vec2 offset = vec2(xoffset,yoffset);\n" \
    "            float prop = 1.0/(offset.x*offset.x+offset.y*offset.y+1.0);\n" \
    "            total_added += prop;\n" \
    "            col += prop*texture2D(tex,texcoord+offset*texelsize);\n" \
    "        }\n" \
    "    }\n" \
    "    col /= total_added;\n" \
    "    gl_FragColor = clamp(col,vec4(0),vec4(1));\n" \
    "}\n"
    
    
    //Source: http://www.nexcius.net/2012/11/20/how-to-load-a-glsl-shader-in-opengl-using-c/
    GLuint loadShader(std::string v, std::string f) {        
        GLuint vertShader = glCreateShader(GL_VERTEX_SHADER);
        GLuint fragShader = glCreateShader(GL_FRAGMENT_SHADER);
        
        // Read shaders
        const char *vertShaderSrc = v.c_str();
        const char *fragShaderSrc = f.c_str();
        
        GLint result = GL_FALSE;
        int logLength;
        
        // Compile vertex shader
        std::cout << "Compiling vertex shader." << std::endl;
        glShaderSource(vertShader, 1, &vertShaderSrc, NULL);
        glCompileShader(vertShader);
        
        // Check vertex shader
        glGetShaderiv(vertShader, GL_COMPILE_STATUS, &result);
        glGetShaderiv(vertShader, GL_INFO_LOG_LENGTH, &logLength);
        eglcheck();
        
        // Compile fragment shader
        std::cout << "Compiling fragment shader." << std::endl;
        glShaderSource(fragShader, 1, &fragShaderSrc, NULL);
        glCompileShader(fragShader);
        
        // Check fragment shader
        glGetShaderiv(fragShader, GL_COMPILE_STATUS, &result);
        glGetShaderiv(fragShader, GL_INFO_LOG_LENGTH, &logLength);
        eglcheck();
        
        std::cout << "Linking program" << std::endl;
        GLuint program = glCreateProgram();
        glAttachShader(program, vertShader);
        glAttachShader(program, fragShader);
        glLinkProgram(program);
        
        glGetProgramiv(program, GL_LINK_STATUS, &result);
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &logLength);
        eglcheck();
        
        glDeleteShader(vertShader);
        glDeleteShader(fragShader);
        
        return program;
    }
    
    GLuint loadVertixBuffer() {
        // Screen/framebuffer positions. As we use full screen, mapping to each corner of screen is used
        static const GLfloat vertices[] = {
            -1.0f, -1.0f,
             1.0f, -1.0f,
            -1.0f,  1.0f,
             1.0f,  1.0f
        };
        
        GLuint id;
        glGenBuffers(1, &id); eglcheck();
        glBindBuffer(GL_ARRAY_BUFFER, id); eglcheck();
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW); eglcheck();    
        return id;
    }
    
    GLuint loadFrameBuffer() {
        GLuint id;
        glGenFramebuffers(1, &id); eglcheck();
        glBindFramebuffer(GL_FRAMEBUFFER, id); eglcheck();
        return id;
    }
    
    void initOpenGL(FLASHCAM_EGL_t* state) {
        //copy state
        FlashCamEGL::state = state;
        
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
        eglcheck();
        
        // 2. Initialize the EGL display connection
        EGLint major, minor;
        result = eglInitialize(state->display, &major, &minor);
        reglcheck();
        eglcheck();
        
        // 3. Select an appropriate configuration
        EGLint numConfigs;
        EGLConfig config;
        result = eglChooseConfig(state->display, configAttr, &config, 1, &numConfigs);
        reglcheck();
        eglcheck();
        //eglDispConfig(config);
        
        // 4. Create a surface
        state->surface = eglCreatePbufferSurface(state->display, config, pbufferAttr);
        assert(state->surface != EGL_NO_SURFACE);
        eglcheck();
        
        // 5. Bind the API
        result = eglBindAPI(EGL_OPENGL_ES_API);
        reglcheck();
        eglcheck();
        
        // 6. Create a context and make it current
        state->context = eglCreateContext(state->display, config, EGL_NO_CONTEXT, contextAttr);
        assert(state->context != EGL_NO_CONTEXT);
        eglcheck();
        
        // 7. Make it current
        result = eglMakeCurrent(state->display, state->surface, state->surface, state->context);
        reglcheck();
        eglcheck();
        
        //No depth in textures/renderer
        glDisable(GL_DEPTH_TEST); eglcheck();
        //Allow alpha-channel
        glEnable (GL_BLEND); eglcheck();
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);eglcheck();
        
        //glEnable(GL_TEXTURE_EXTERNAL_OES);
        //eglDispError();
        //eglcheck();
        
        //create texture
        glGenTextures(1, &state->texture);eglcheck();
        glBindTexture(GL_TEXTURE_EXTERNAL_OES, state->texture);eglcheck();
        
        // 8. Set background color and clear buffers
        //glClearColor(1.0f, 0.0f, 0.0f, 1.0f);
        //glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        //eglcheck();
        
        //Load Shaders
        //load program/buffers upon first call
        FlashCamEGL::progid_oes2rgb = loadShader(SRC_VSHADER_2D, SRC_FSHADER_OES2RGB);   
        FlashCamEGL::progid_rgbblur = loadShader(SRC_VSHADER_2D, SRC_FSHADER_RGBBLUR);   

        //setup buffers
        FlashCamEGL::vbufid         = loadVertixBuffer();   
        FlashCamEGL::fbufid         = loadFrameBuffer();   
    }

    void clearOpenGL() {
        std::cout << "Clear OpenGL." << std::endl;

        //destroy textures
        glDeleteTextures(1, &FlashCamEGL::state->texture);
        for (std::vector<GLuint>::iterator it = FlashCamEGL::textures.begin() ; it != FlashCamEGL::textures.end(); ++it)
            glDeleteTextures(1, (GLuint*) &*it);

        //destroy image
        if (FlashCamEGL::state->img != EGL_NO_IMAGE_KHR) {
            eglDestroyImageKHR(FlashCamEGL::state->display, FlashCamEGL::state->img);
            FlashCamEGL::state->img = EGL_NO_IMAGE_KHR;
        }
        
        //destroy buffers;
        glDeleteFramebuffers(1, &fbufid);
        glDeleteBuffers(1, &vbufid);
        glDeleteProgram(progid_oes2rgb);
        glDeleteProgram(progid_rgbblur);
        
        //detroy openGL
        eglMakeCurrent(FlashCamEGL::state->display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        eglDestroyContext(FlashCamEGL::state->display, FlashCamEGL::state->context);
        eglDestroySurface(FlashCamEGL::state->display, FlashCamEGL::state->surface);
        eglTerminate(FlashCamEGL::state->display);
    }
    
    
    
    
    //transform a MMAL-buffer into a OpenGL texture.
    void mmalbuf2TextureOES_internal(MMAL_BUFFER_HEADER_T *buffer) {
        
        //set texture id
        glBindTexture(GL_TEXTURE_EXTERNAL_OES, FlashCamEGL::state->texture); eglcheck();
        
        //destroy previous image
        if (FlashCamEGL::state->img != EGL_NO_IMAGE_KHR) {
            eglDestroyImageKHR(FlashCamEGL::state->display, state->img);
            FlashCamEGL::state->img = EGL_NO_IMAGE_KHR;
        }
        
        //Create new image
        state->img = eglCreateImageKHR(FlashCamEGL::state->display, EGL_NO_CONTEXT, EGL_IMAGE_BRCM_MULTIMEDIA_Y, (EGLClientBuffer) buffer->data, NULL);
        eglcheck();
        
        glEGLImageTargetTexture2DOES(GL_TEXTURE_EXTERNAL_OES, FlashCamEGL::state->img);
        eglcheck();
        
        //Scaling: nearest (=no) interpolation for scaling down and up.
        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER, GL_NEAREST);eglcheck();
        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER, GL_NEAREST);eglcheck();
        
        /*
         http://www.khronos.org/registry/gles/extensions/OES/OES_EGL_image_external.txt
         "3.7.14 External Textures
         ...
         The default s and t wrap modes are CLAMP_TO_EDGE and it is an
         INVALID_ENUM error to set the wrap mode to any other value.
         */
        //Wrapping: repeat. Only use (s,t) as we are using a 2D texture
        //                    glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_S, GL_REPEAT);eglcheck();
        //                    glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_T, GL_REPEAT);eglcheck();
        
        
        // release and lock buffer of current EGLImage
        //if (FlashCamEGL::state->buffer_img)
        //    mmal_buffer_header_release(FlashCamEGL::state->buffer_img);
        //FlashCamEGL::state->buffer_img = buffer;
    }
    

    void texture2texture(GLuint input_texid, GLenum input_target,  GLuint result_texid, GLuint progid) {        
        int width  = FlashCamEGL::state->userdata->settings->width;
        int height = FlashCamEGL::state->userdata->settings->height;
    
        //select proper progam
        glUseProgram(progid); eglcheck();        
        //Set framebuffer and render postitions
        glBindBuffer(GL_ARRAY_BUFFER, FlashCamEGL::vbufid); eglcheck();
        glBindFramebuffer(GL_FRAMEBUFFER, FlashCamEGL::fbufid); eglcheck();
        
        //set output texture
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, result_texid, 0); eglcheck();
        glViewport (0, 0, width, height );

        //Set active texture.
        glActiveTexture(GL_TEXTURE0); eglcheck();
        glBindTexture(input_target, input_texid); eglcheck();
        
        //Set render/shader-parameters
        glUniform1i(glGetUniformLocation(progid,"tex"), 0); //assign GL_TEXTURE0
        glUniform2f(glGetUniformLocation(progid,"texelsize"), 1.f/width, 1.f/height);
        eglcheck();
        
        //setup format of texture
        GLuint pos = glGetAttribLocation(progid,"position");
        // pos      : postion in texture
        // 2        : only x/y values (z/d is ignored)
        // GL_FLOAT : position is floating point
        // 0        : not normalised postion
        // 8        : number of bytes for each postion ( 2 * sizeof(float) = 2 * 4 bytes = 8)
        // 0        : bufferoffset, we start at first byter of array
        glVertexAttribPointer(pos, 2, GL_FLOAT, 0, 8, 0);	eglcheck();
        glEnableVertexAttribArray(pos);	eglcheck();
        
        // activate shader / start OpenGL processing
        glDrawArrays ( GL_TRIANGLE_STRIP, 0, 4 ); eglcheck();
    }
    
    
    void textureOES2rgb(GLuint input_texid, GLuint result_texid) {        
        texture2texture(input_texid, GL_TEXTURE_EXTERNAL_OES,  result_texid, progid_oes2rgb);       
    }
    
    void textureRGBblur(GLuint input_texid, GLuint result_texid) {        
        texture2texture(input_texid, GL_TEXTURE_2D, result_texid, progid_rgbblur);    
    }
    
    GLuint createTexture() {
        GLuint id;
        //request ID for texture..
        glGenTextures(1, &id);eglcheck();
        //Texture is a 2D texture
        glBindTexture(GL_TEXTURE_2D, id);eglcheck();
        //Scaling: nearest (=no) interpolation for scaling down and up.
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);eglcheck();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);eglcheck();
        //Wrapping: repeat. Only use (s,t) as we are using a 2D texture
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);eglcheck();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);eglcheck();
        
        int width  = FlashCamEGL::state->userdata->settings->width;
        int height = FlashCamEGL::state->userdata->settings->height;
        
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL); eglcheck();
        
        textures.push_back(id);
        return id;
    }

    

    
    void eglDispError() {
        EGLint err = eglGetError();
        if (err != EGL_SUCCESS) {
            switch (err) {
                case EGL_NOT_INITIALIZED:
                    fprintf(stdout, "EGL error: EGL_NOT_INITIALIZED, %d\n", err);
                    break;
                case EGL_BAD_ACCESS:
                    fprintf(stdout, "EGL error: EGL_BAD_ACCESS, %d\n", err);
                    break;
                case EGL_BAD_ALLOC:
                    fprintf(stdout, "EGL error: EGL_BAD_ALLOC, %d\n", err);
                    break;
                case EGL_BAD_ATTRIBUTE:
                    fprintf(stdout, "EGL error: EGL_BAD_ATTRIBUTE, %d\n", err);
                    break;
                case EGL_BAD_CONTEXT:
                    fprintf(stdout, "EGL error: EGL_BAD_CONTEXT, %d\n", err);
                    break;
                case EGL_BAD_CONFIG:
                    fprintf(stdout, "EGL error: EGL_BAD_CONFIG, %d\n", err);
                    break;
                case EGL_BAD_CURRENT_SURFACE:
                    fprintf(stdout, "EGL error: EGL_BAD_CURRENT_SURFACE, %d\n", err);
                    break;
                case EGL_BAD_DISPLAY:
                    fprintf(stdout, "EGL error: EGL_BAD_DISPLAY, %d\n", err);
                    break;
                case EGL_BAD_SURFACE:
                    fprintf(stdout, "EGL error: EGL_BAD_SURFACE, %d\n", err);
                    break;
                case EGL_BAD_MATCH:
                    fprintf(stdout, "EGL error: EGL_BAD_MATCH, %d\n", err);
                    break;
                case EGL_BAD_PARAMETER:
                    fprintf(stdout, "EGL error: EGL_BAD_PARAMETER, %d\n", err);
                    break;
                case EGL_BAD_NATIVE_PIXMAP:
                    fprintf(stdout, "EGL error: EGL_BAD_NATIVE_PIXMAP, %d\n", err);
                    break;
                case EGL_BAD_NATIVE_WINDOW:
                    fprintf(stdout, "EGL error: EGL_BAD_NATIVE_WINDOW, %d\n", err);
                    break;
                case EGL_CONTEXT_LOST:
                    fprintf(stdout, "EGL error: EGL_CONTEXT_LOST, %d\n", err);
                    break;
                default:
                    fprintf(stdout, "EGL error: Unkown (%d)\n", err);
                    break;
            }
        }
    }

}

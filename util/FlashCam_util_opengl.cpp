//
//  FlashCam_util_opengl.cpp
//  
//
//  Created by Hessel van der Molen on 05/09/17.
//
//

#include "FlashCam_opengl.h"
#include "FlashCam_util_opengl.h"

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>

namespace FlashCamUtilOpenGL {
        
    static GLuint progid_oes2rgb    = 0;
    static GLuint progid_rgbblur    = 0;
    static GLuint progid_rgbsobel   = 0;
    static GLuint vbufid            = 0;
    static GLuint fbufid            = 0;
    
    static std::vector<GLuint> textures;
    
    //OpenGL settings
    static EGLDisplay _display;                 /// The current EGL display
    static EGLSurface _surface;                 /// The current EGL surface
    static EGLContext _context;                 /// The current EGL context
    //size
    static unsigned int _width      = 0;
    static unsigned int _height     = 0;
    
    static bool initialised         = false;
    
    static FLASHCAM_CALLBACK_OPENGL_DESTROY_T _callback;
    
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
    
    //sobel shader
#define SRC_FSHADER_RGBSOBEL \
    "varying vec2 texcoord;\n" \
    "uniform sampler2D tex;\n" \
    "uniform vec2 texelsize;\n" \
    "void main(void) {\n" \
    "    vec4 tm1m1 = texture2D(tex,texcoord+vec2(-1,-1)*texelsize);\n" \
    "    vec4 tm10  = texture2D(tex,texcoord+vec2(-1,0)*texelsize);\n" \
    "    vec4 tm1p1 = texture2D(tex,texcoord+vec2(-1,1)*texelsize);\n" \
    "    vec4 tp1m1 = texture2D(tex,texcoord+vec2(1,-1)*texelsize);\n" \
    "    vec4 tp10  = texture2D(tex,texcoord+vec2(1,0)*texelsize);\n" \
    "    vec4 tp1p1 = texture2D(tex,texcoord+vec2(1,1)*texelsize);\n" \
    "    vec4 t0m1  = texture2D(tex,texcoord+vec2(0,-1)*texelsize);\n" \
    "    vec4 t0p1  = texture2D(tex,texcoord+vec2(0,-1)*texelsize);\n" \
    "    vec4 xdiff = -1.0*tm1m1 + -2.0*tm10 + -1.0*tm1p1 + 1.0*tp1m1 + 2.0*tp10 + 1.0*tp1p1;\n" \
    "    vec4 ydiff = -1.0*tm1m1 + -2.0*t0m1 + -1.0*tp1m1 + 1.0*tm1p1 + 2.0*t0p1 + 1.0*tp1p1;\n" \
    "    vec4 tot   = sqrt(xdiff*xdiff+ydiff*ydiff);\n" \
    "    vec4 col   = tot;\n" \
    "    col.a      = 1.0;\n" \
    "    gl_FragColor = clamp(col,vec4(0),vec4(1));\n" \
    "}\n"

    
    //Source: http://www.nexcius.net/2012/11/20/how-to-load-a-glsl-shader-in-opengl-using-c/
    GLuint createShaderProgram(std::string v, std::string f) {        
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
    
    void init(int w, int h) {        
        //if already initialised: return.
        if (FlashCamUtilOpenGL::initialised)
            return;
        
        FlashCamUtilOpenGL::_width  = w;
        FlashCamUtilOpenGL::_height = h;
        
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
            EGL_WIDTH,  (const EGLint) FlashCamUtilOpenGL::_width,
            EGL_HEIGHT, (const EGLint) FlashCamUtilOpenGL::_height,
            EGL_NONE,
        };
        
        static const EGLint contextAttr[] = {
            EGL_CONTEXT_CLIENT_VERSION, 2,
            EGL_NONE
        };
        
        /*************************/
        
        // 1. Get an EGL display connection
        FlashCamUtilOpenGL::_display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
        assert(FlashCamUtilOpenGL::_display != EGL_NO_DISPLAY);
        eglcheck();
        
        // 2. Initialize the EGL display connection
        EGLint major, minor;
        result = eglInitialize(FlashCamUtilOpenGL::_display, &major, &minor);
        reglcheck();
        eglcheck();
        
        // 3. Select an appropriate configuration
        EGLint numConfigs;
        EGLConfig config;
        result = eglChooseConfig(FlashCamUtilOpenGL::_display, configAttr, &config, 1, &numConfigs);
        reglcheck();
        eglcheck();
        //eglDispConfig(config);
        
        // 4. Create a surface
        FlashCamUtilOpenGL::_surface = eglCreatePbufferSurface(FlashCamUtilOpenGL::_display, config, pbufferAttr);
        assert(FlashCamUtilOpenGL::_surface != EGL_NO_SURFACE);
        eglcheck();
        
        // 5. Bind the API
        result = eglBindAPI(EGL_OPENGL_ES_API);
        reglcheck();
        eglcheck();
        
        // 6. Create a context and make it current
        FlashCamUtilOpenGL::_context = eglCreateContext(FlashCamUtilOpenGL::_display, config, EGL_NO_CONTEXT, contextAttr);
        assert(FlashCamUtilOpenGL::_context != EGL_NO_CONTEXT);
        eglcheck();
        
        // 7. Make it current
        result = eglMakeCurrent(FlashCamUtilOpenGL::_display, FlashCamUtilOpenGL::_surface, FlashCamUtilOpenGL::_surface, FlashCamUtilOpenGL::_context);
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
        
        // 8. Set background color and clear buffers
        //glClearColor(1.0f, 0.0f, 0.0f, 1.0f);
        //glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        //eglcheck();
        
        //Load Shaders
        //load program/buffers upon first call
        FlashCamUtilOpenGL::progid_oes2rgb  = createShaderProgram(SRC_VSHADER_2D, SRC_FSHADER_OES2RGB);   
        FlashCamUtilOpenGL::progid_rgbblur  = createShaderProgram(SRC_VSHADER_2D, SRC_FSHADER_RGBBLUR);   
        FlashCamUtilOpenGL::progid_rgbsobel = createShaderProgram(SRC_VSHADER_2D, SRC_FSHADER_RGBSOBEL);   

        //setup buffers
        FlashCamUtilOpenGL::vbufid         = loadVertixBuffer();   
        FlashCamUtilOpenGL::fbufid         = loadFrameBuffer();  
        FlashCamUtilOpenGL::_callback      = NULL;
        
        FlashCamUtilOpenGL::initialised = true;
    }
    
    bool isInitialised() {
        return FlashCamUtilOpenGL::initialised;
    }

    
    void destroyEGLImage(EGLImageKHR *targetimg) {
        if (*targetimg != EGL_NO_IMAGE_KHR) {
            eglDestroyImageKHR(FlashCamUtilOpenGL::_display, *targetimg);
            *targetimg = EGL_NO_IMAGE_KHR;
        }
    }


    void destroy() {
        std::cout << "Clear OpenGL." << std::endl;

        //destroy textures
        for (std::vector<GLuint>::iterator it = FlashCamUtilOpenGL::textures.begin() ; it != FlashCamUtilOpenGL::textures.end(); ++it)
            glDeleteTextures(1, (GLuint*) &*it);
        
        //destroy buffers;
        glDeleteFramebuffers(1, &fbufid);
        glDeleteBuffers(1, &vbufid);
        glDeleteProgram(progid_oes2rgb);
        glDeleteProgram(progid_rgbblur);
        glDeleteProgram(progid_rgbsobel);
        
        if (FlashCamUtilOpenGL::_callback)
            FlashCamUtilOpenGL::_callback();
        
        //detroy openGL
        eglMakeCurrent(FlashCamUtilOpenGL::_display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        eglDestroyContext(FlashCamUtilOpenGL::_display, FlashCamUtilOpenGL::_context);
        eglDestroySurface(FlashCamUtilOpenGL::_display, FlashCamUtilOpenGL::_surface);
        eglTerminate(FlashCamUtilOpenGL::_display);
    }
    
    
    
    
    //transform a MMAL-buffer into a OpenGL texture.
    void mmalbuf2TextureOES(MMAL_BUFFER_HEADER_T *buffer, GLuint targetid, EGLImageKHR *targetimg) {
        glBindFramebuffer(GL_FRAMEBUFFER, FlashCamUtilOpenGL::fbufid); eglcheck();

        //set texture id
        glBindTexture(GL_TEXTURE_EXTERNAL_OES, targetid); eglcheck();
        
        //destroy previous image
        if (*targetimg != EGL_NO_IMAGE_KHR) {
            eglDestroyImageKHR(FlashCamUtilOpenGL::_display, *targetimg);
            *targetimg = EGL_NO_IMAGE_KHR;
        }
        
        //Create new image
        *targetimg = eglCreateImageKHR(FlashCamUtilOpenGL::_display, EGL_NO_CONTEXT, EGL_IMAGE_BRCM_MULTIMEDIA_Y, (EGLClientBuffer) buffer->data, NULL);
        eglcheck();
        
        glEGLImageTargetTexture2DOES(GL_TEXTURE_EXTERNAL_OES, *targetimg);
        eglcheck();
        
        //Scaling: nearest (=no) interpolation for scaling down and up.
        // Also disables mipmapping
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
    }
    

    void texture2texture(GLuint input_texid, GLenum input_target,  GLuint result_texid, GLuint progid) {        
        int width  = FlashCamUtilOpenGL::_width;
        int height = FlashCamUtilOpenGL::_height;
    
        //select proper progam
        glUseProgram(progid); eglcheck();        
        //Set framebuffer and render postitions
        glBindBuffer(GL_ARRAY_BUFFER, FlashCamUtilOpenGL::vbufid); eglcheck();
        glBindFramebuffer(GL_FRAMEBUFFER, FlashCamUtilOpenGL::fbufid); eglcheck();
        
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

    void textureRGBsobel(GLuint input_texid, GLuint result_texid) {        
        texture2texture(input_texid, GL_TEXTURE_2D, result_texid, progid_rgbsobel);    
    }
    
    
    GLuint generateTexture(GLenum type) {
        //create texture
        GLuint id;
        glGenTextures(1, &id);eglcheck();
        glBindTexture(type, id);eglcheck();
        textures.push_back(id);
        return id;        
    }
        
    GLuint createTexture() {
        int w = FlashCamUtilOpenGL::_width;
        int h = FlashCamUtilOpenGL::_height;
        return createTexture(w, h, GL_RGBA); 
    }

    GLuint createTexture(int w, int h, GLenum dataformat) {
        GLuint id = generateTexture(GL_TEXTURE_2D);
        //Scaling: nearest (=no) interpolation for scaling down and up.
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);eglcheck();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);eglcheck();
        //Wrapping: repeat. Only use (s,t) as we are using a 2D texture
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);eglcheck();
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);eglcheck();
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, dataformat, GL_UNSIGNED_BYTE, NULL); eglcheck();
        
        return id;
    }
    
    
    
    FLASHCAM_OPENGL_STATE_T getOpenGLState( FLASHCAM_CALLBACK_OPENGL_DESTROY_T callback) {
        _callback = callback;        
        FLASHCAM_OPENGL_STATE_T s;
        s.default_width     = _width;
        s.default_height    = _height;
        s.display           = _display;
        s.surface           = _surface;
        s.context           = _context;
        return s;
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

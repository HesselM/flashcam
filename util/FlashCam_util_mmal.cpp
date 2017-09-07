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

#include "FlashCam_util_mmal.h"

#include "interface/vcos/vcos.h"
#include "interface/mmal/mmal_logging.h"

namespace FlashCamMMAL {
    
    int mmal_to_int(MMAL_STATUS_T status) {
        if (status == MMAL_SUCCESS) {
            return 0;
        } else {
            switch (status) {
                case MMAL_ENOMEM :   vcos_log_error("Out of memory"); break;
                case MMAL_ENOSPC :   vcos_log_error("Out of resources (other than memory)"); break;
                case MMAL_EINVAL:    vcos_log_error("Argument is invalid"); break;
                case MMAL_ENOSYS :   vcos_log_error("Function not implemented"); break;
                case MMAL_ENOENT :   vcos_log_error("No such file or directory"); break;
                case MMAL_ENXIO :    vcos_log_error("No such device or address"); break;
                case MMAL_EIO :      vcos_log_error("I/O error"); break;
                case MMAL_ESPIPE :   vcos_log_error("Illegal seek"); break;
                case MMAL_ECORRUPT : vcos_log_error("Data is corrupt \attention FIXME: not POSIX"); break;
                case MMAL_ENOTREADY :vcos_log_error("Component is not ready \attention FIXME: not POSIX"); break;
                case MMAL_ECONFIG :  vcos_log_error("Component is not configured \attention FIXME: not POSIX"); break;
                case MMAL_EISCONN :  vcos_log_error("Port is already connected "); break;
                case MMAL_ENOTCONN : vcos_log_error("Port is disconnected"); break;
                case MMAL_EAGAIN :   vcos_log_error("Resource temporarily unavailable. Try again later"); break;
                case MMAL_EFAULT :   vcos_log_error("Bad address"); break;
                default :            vcos_log_error("Unknown status error"); break;
            }
            return 1;
        }
    }
    
}

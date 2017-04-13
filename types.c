//
//  types.c
//  
//
//  Created by Hessel van der Molen on 13/04/17.
//
//

#include <stdio.h>
#include "types.h"

#include "interface/vcos/vcos.h"
#include "interface/mmal/mmal_logging.h"

int mmal_status_to_int(MMAL_STATUS_T status)
{
    if (status == MMAL_SUCCESS)
        return 0;
    else
    {
        switch (status)
        {
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

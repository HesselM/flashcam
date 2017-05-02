//
//  FlashCam_PLL.hpp
//  
//
//  Created by Hessel van der Molen on 02/05/17.
//
//

#ifndef FlashCamPLL_h
#define FlashCamPLL_h

#include "FlashCam.h"

class FlashCamPLL
{
    
private:
    
public:    
    static void update( FLASHCAM_SETTINGS_T *settings, FLASHCAM_PARAMS_T *params, uint64_t buffertime);
    static void start( FLASHCAM_SETTINGS_T *settings, FLASHCAM_PARAMS_T *params );
    static void stop();
};


#endif /* FlashCamPLL_h */

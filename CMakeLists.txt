cmake_minimum_required (VERSION 2.8.11)

project(flashcam)

option(VIDTEST "compile for video testing" OFF)
option(VIDTESTEGL "compile for video testing with OpenGL" OFF)
option(PLLTUNE "compile for PLL tuning" OFF)
option(STEPRESPONSE "compile for steprespoinse" OFF)
option(FRAMECAPTURE "compile to capture, view and store frames by using OpenCV" OFF)
option(FRAMECAPTUREEGL "compile to capture, view and store frames by using OpenCV and OpenGL" OFF)

set(CMAKE_CXX_FLAGS "-fpermissive -std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_C_FLAGS   "-fpermissive -std=c++11 ${CMAKE_C_FLAGS}")

# Main sources for FlashCam-lib
set(FLASHCAM_SOURCES FlashCam.cpp types.cpp)

#include required packages
find_package( Threads REQUIRED )
find_package( PkgConfig REQUIRED )
find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

# MMAL
pkg_search_module( MMAL REQUIRED mmal )
include_directories( ${MMAL_INCLUDE_DIRS} )
link_directories( ${MMAL_LIBRARY_DIRS} )

# BCM_HOST
pkg_search_module( BCMHOST REQUIRED bcm_host )
include_directories( ${BCMHOST_INCLUDE_DIRS} )
link_directories( ${BCMHOST_LIBRARY_DIRS} )

#EGL -> optional
if (FRAMECAPTUREEGL)
    pkg_search_module(EGL REQUIRED egl)
    include_directories(${EGL_INCLUDE_DIRS})
    link_directories(${EGL_LIBRARY_DIRS} )
endif()

# WiringPi -> optional (required for PLL)
pkg_search_module( WIRINGPI wiringpi )
if (WIRINGPI_FOUND) 
    message(">> Found WiringPi: including function in build")
    include_directories( ${WIRINGPI_INCLUDE_DIRS} )
    link_directories( ${WIRINGPI_LIBRARY_DIRS} )
#add PLL sources
    add_definitions( -DBUILD_FLASHCAM_WITH_PLL )
    set(FLASHCAM_SOURCES FlashCamPLL.cpp; ${FLASHCAM_SOURCES})
else()
    message(">> Did not found WiringPi: functions disabled")
endif()

# Userland -> if not set via commandline or toolchain, set default value.
if (NOT USERLAND_DIR)
    message(">> Setting default USERLAND_DIR: /usr/src/userland")
    set(USERLAND_DIR "/usr/src/userland")
endif()
include_directories(${USERLAND_DIR})
include_directories(${USERLAND_DIR}/host_applications/linux/libs/sm)

# Projectdirs
include_directories(${CMAKE_SOURCE_DIR})

# Which test?
if(VIDTESTEGL)
    message(">> Building for video-testing with OpenGL (VIDTESTEGL=ON)")
    set(FLASHCAM_SOURCES FlashCamEGL.cpp; FlashCamEGLUtil.cpp; FlashCam_test_vid_egl.cpp; ${FLASHCAM_SOURCES})
    add_definitions( -DEGL)
elseif (FRAMECAPTUREEGL)
    message(">> Building for OpenGL framecapturing (FRAMECAPTUREEGL=ON)")
    set(FLASHCAM_SOURCES FlashCamEGL.cpp; FlashCamEGLUtil.cpp; FlashCam_framecapture_egl.cpp; ${FLASHCAM_SOURCES})
    add_definitions( -DEGL)
elseif (FRAMECAPTURE)
    message(">> Building for framecapturing (FRAMECAPTURE=ON)")
    set(FLASHCAM_SOURCES FlashCam_framecapture.cpp; ${FLASHCAM_SOURCES})
elseif(PLLTUNE)
    message(">> Building for PLL-tuning (PLLTUNE=ON)")
    add_definitions( -DPLLTUNE)
    if (STEPRESPONSE)
        add_definitions( -DSTEPRESPONSE)
    endif()
    set(FLASHCAM_SOURCES FlashCam_tunepll.cpp; terminal.cpp; ${FLASHCAM_SOURCES})
elseif(VIDTEST)
    message(">> Building for video-testing (VIDTEST=ON)")
    set(FLASHCAM_SOURCES FlashCam_test_vid.cpp; ${FLASHCAM_SOURCES})
else()
    message(">> Building for capture-testing (VIDTEST=OFF)")
    set(FLASHCAM_SOURCES FlashCam_test.cpp; ${FLASHCAM_SOURCES})
endif()
        

# Executables
add_executable(flashcam ${FLASHCAM_SOURCES})

# Libraries
target_link_libraries(flashcam ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(flashcam ${OpenCV_LIBS} )
target_link_libraries(flashcam ${MMAL_LIBRARIES})
target_link_libraries(flashcam ${BCMHOST_LIBRARIES})
target_link_libraries(flashcam m)

if (FRAMECAPTUREEGL)
    target_link_libraries(flashcam ${EGL_LIBRARIES})
endif()

if (WIRINGPI_FOUND) 
    target_link_libraries(flashcam ${WIRINGPI_LIBRARIES})
endif()

# Print output.
set( PRETTY_CMAKE_CXX_FLAGS "\t${CMAKE_CXX_FLAGS}")
string(REPLACE " -" "\n\t-" PRETTY_CMAKE_CXX_FLAGS ${PRETTY_CMAKE_CXX_FLAGS})
set( PRETTY_CMAKE_C_FLAGS "\t${CMAKE_C_FLAGS}")
string(REPLACE " -" "\n\t-" PRETTY_CMAKE_C_FLAGS ${PRETTY_CMAKE_C_FLAGS})

message(">> Used CXX flags: \n ${PRETTY_CMAKE_CXX_FLAGS}")
message(">> Used C flags: \n${PRETTY_CMAKE_C_FLAGS}")
message(">> Source List: \n${FLASHCAM_SOURCES}")


# list(REMOVE_DUPLICATES <list>

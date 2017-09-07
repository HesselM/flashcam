cmake_minimum_required (VERSION 2.8.11)

project(flashcam)

option(TEST_CAP "compile for capture-mode testing" OFF)
option(TEST_VID "compile for video-mode streaming testing" OFF)
option(TEST_VID_OPENGL "compile for video-mode streaming testing with OpenGL rendering" OFF)
option(TEST_VID_FRAMECAPTURE "compile for video-mode testing. Frames are recorded with a keypress." OFF)
option(TEST_VID_OPENGL_FRAMECAPTURE "compile for video-mode streaming testing with OpenGL rendering. Frames are recorded with a keypress." OFF)
option(TEST_PLL_TUNE "compile for PLL tuning" OFF)
option(TEST_PLL_STEPRESPONSE "compile for PLL stepresponse recording" OFF)

set(CMAKE_CXX_FLAGS "-fpermissive -std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_C_FLAGS   "-fpermissive -std=c++11 ${CMAKE_C_FLAGS}")

# Main sources for FlashCam-lib
set(FLASHCAM_SOURCES FlashCam.cpp FlashCam_types.cpp util/FlashCam_util_mmal.cpp)

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


# EGL -> optional (required for OpenGL rendering)
pkg_search_module(EGL REQUIRED egl)
if (EGL_FOUND) 
    include_directories(${EGL_INCLUDE_DIRS})
    link_directories(${EGL_LIBRARY_DIRS} )
#add OpenGL sources
    add_definitions( -DBUILD_FLASHCAM_WITH_OPENGL )
    set(FLASHCAM_SOURCES    opengl/FlashCam_opengl.cpp; 
                            util/FlashCam_util_opengl.cpp;
                            ${FLASHCAM_SOURCES})
    message(">> Found EGL: including OpenGL functions in build")
else()
    message(">> Did not found EGL: OpenGL functions and tests are disabled")
endif()


# WiringPi -> optional (required for PLL)
pkg_search_module( WIRINGPI wiringpi )
if (WIRINGPI_FOUND) 
    include_directories( ${WIRINGPI_INCLUDE_DIRS} )
    link_directories( ${WIRINGPI_LIBRARY_DIRS} )
#add PLL sources
    add_definitions( -DBUILD_FLASHCAM_WITH_PLL )
    set(FLASHCAM_SOURCES    pll/FlashCam_pll.cpp; 
                            ${FLASHCAM_SOURCES})
    message(">> Found WiringPi: including PLL functions in build")
else()
    message(">> Did not found WiringPi: PLL functions and tests are disabled")
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
include_directories(${CMAKE_SOURCE_DIR}/pll)
include_directories(${CMAKE_SOURCE_DIR}/opengl)
include_directories(${CMAKE_SOURCE_DIR}/tests)
include_directories(${CMAKE_SOURCE_DIR}/util)


# Which test?
if (TEST_CAP)
    set(FLASHCAM_SOURCES tests/FlashCam_test_cap.cpp; ${FLASHCAM_SOURCES})
    message(">> Building for capture-mode testing. (TEST_CAP=ON)")

elseif (TEST_VID)
    set(FLASHCAM_SOURCES tests/FlashCam_test_vid.cpp; ${FLASHCAM_SOURCES})
    message(">> Building for video-mode stream testing. (TEST_VID=ON)")

elseif (TEST_VID_FRAMECAPTURE)
    set(FLASHCAM_SOURCES tests/FlashCam_test_vid_framecapture.cpp; ${FLASHCAM_SOURCES})
    message(">> Building for video-mode stream testing. Frames are recorded with a keypress. (TEST_VID_FRAMECAPTURE=ON)")

elseif (TEST_VID_OPENGL AND EGL_FOUND)
    set(FLASHCAM_SOURCES tests/FlashCam_test_vid_opengl.cpp; ${FLASHCAM_SOURCES})
    message(">> Building for video-mode stream testing with OpenGL rendering. (TEST_VID_OPENGL=ON)")

elseif (TEST_VID_OPENGL_FRAMECAPTURE AND EGL_FOUND)
    set(FLASHCAM_SOURCES tests/FlashCam_test_vid_framecapture.cpp; ${FLASHCAM_SOURCES})
    message(">> Building for video-mode stream testing with OpenGL rendering. Frames are recorded with a keypress. (TEST_VID_OPENGL_FRAMECAPTURE=ON)")

elseif (TEST_PLL_TUNE AND WIRINGPI_FOUND)
    add_definitions( -DPLLTUNE )
    set(FLASHCAM_SOURCES tests/FlashCam_test_pll_tune.cpp; util/FlashCam_util_terminal.cpp; ${FLASHCAM_SOURCES})
    message(">> Building for PLL tuning. (TEST_PLL_TUNE=ON)")

elseif (TEST_PLL_STEPRESPONSE AND WIRINGPI_FOUND)
    add_definitions( -DPLLTUNE )
    add_definitions( -DSTEPRESPONSE )
    set(FLASHCAM_SOURCES tests/FlashCam_test_pll_tune.cpp; util/FlashCam_util_terminal.cpp; ${FLASHCAM_SOURCES})
    message(">> Building for PLL stepresponse recording. (TEST_PLL_STEPRESPONSE=ON)")

endif()


# Executables
add_executable(flashcam ${FLASHCAM_SOURCES})

# Libraries
target_link_libraries(flashcam ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(flashcam ${OpenCV_LIBS} )
target_link_libraries(flashcam ${MMAL_LIBRARIES})
target_link_libraries(flashcam ${BCMHOST_LIBRARIES})
target_link_libraries(flashcam m)

if (EGL_FOUND)
    target_link_libraries(flashcam ${EGL_LIBRARIES})
endif()

if (WIRINGPI_FOUND) 
    target_link_libraries(flashcam ${WIRINGPI_LIBRARIES})
endif()

# Print output.
function(removeDuplicateSubstring stringIn stringOut)
    separate_arguments(stringIn)
    list(REMOVE_DUPLICATES stringIn)
    string(REPLACE ";" " " stringIn "${stringIn}")
    set(${stringOut} "${stringIn}" PARENT_SCOPE)
endfunction()

#remove duplicates
string(REPLACE "-isystem /" "-isystem-/" CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
string(REPLACE "-isystem /" "-isystem-/" CMAKE_C_FLAGS   ${CMAKE_C_FLAGS})
removeDuplicateSubstring(${CMAKE_CXX_FLAGS} CMAKE_CXX_FLAGS)
removeDuplicateSubstring(${CMAKE_C_FLAGS}   CMAKE_C_FLAGS)
string(REPLACE "-isystem-/" "-isystem /" CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
string(REPLACE "-isystem-/" "-isystem /" CMAKE_C_FLAGS ${CMAKE_C_FLAGS})

#make it pretty
string(REPLACE " -" "\n\t-" PRETTY_CMAKE_CXX_FLAGS "\t${CMAKE_CXX_FLAGS}")
string(REPLACE " -" "\n\t-" PRETTY_CMAKE_C_FLAGS "\t${CMAKE_C_FLAGS}")

list( SORT FLASHCAM_SOURCES)
string(REPLACE ";"  "\n\t- " PRETTY_FLASHCAM_SOURCES "\t- ${FLASHCAM_SOURCES}")

#show result
message(">> Used CXX flags: \n ${PRETTY_CMAKE_CXX_FLAGS}")
message(">> Used C flags: \n ${PRETTY_CMAKE_C_FLAGS}")
message(">> Source List: \n ${PRETTY_FLASHCAM_SOURCES}")



# list(REMOVE_DUPLICATES <list>

cmake_minimum_required (VERSION 2.8.11)

project(flashcam)

option(VIDTEST "compile for video testing" OFF)

set(CMAKE_CXX_FLAGS "-fpermissive -std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_C_FLAGS   "-fpermissive -std=c++11 ${CMAKE_C_FLAGS}")

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

# WiringPi
#pkg_search_module( WIRINGPI REQUIRED wiringpi )
#include_directories( ${WIRINGPI_INCLUDE_DIRS} )
#link_directories( ${WIRINGPI_LIBRARY_DIRS} )

# Userland -> if not set via commandline or toolchain, set default value.
if(NOT USERLAND_DIR)
    set(USERLAND_DIR "/usr/src/userland")
endif(NOT USERLAND_DIR)

include_directories(${USERLAND_DIR})
include_directories(${USERLAND_DIR}/host_applications/linux/libs/sm)

# Projectdirs
include_directories(${CMAKE_SOURCE_DIR})

# executables
if(VIDTEST)
    message("Building for video-testing (VIDTEST=ON)")
    add_executable(flashcam FlashCam_test_vid.cpp FlashCam.cpp types.c)
else()
    message("Building for capture-testing (VIDTEST=OFF)")
    add_executable(flashcam FlashCam_test.cpp FlashCam.cpp types.c)
endif()

# Libraries
target_link_libraries(flashcam ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(flashcam ${OpenCV_LIBS} )
target_link_libraries(flashcam ${MMAL_LIBRARIES})
target_link_libraries(flashcam ${BCMHOST_LIBRARIES})
#target_link_libraries(flashcam ${WIRINGPI_LIBRARIES})
target_link_libraries(flashcam m)

message("Used CXX flags: ${CMAKE_CXX_FLAGS}")
message("Used C flags: ${CMAKE_C_FLAGS}")

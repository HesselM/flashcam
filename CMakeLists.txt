cmake_minimum_required (VERSION 2.8.11)

project(flashcamyuv)

SET(COMPILE_DEFINITIONS -Werror)

set(CMAKE_CXX_FLAGS " -fpermissive -std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_C_FLAGS " -fpermissive -std=c++11 ${CMAKE_C_FLAGS}")


#include required packages
find_package( Threads REQUIRED )
find_package( PkgConfig REQUIRED )
find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

# MMAL
pkg_search_module(MMAL REQUIRED mmal)
include_directories(${MMAL_INCLUDE_DIRS})
link_directories(${MMAL_LIBRARY_DIRS} )

# BCM_HOST
pkg_search_module(BCMHOST REQUIRED bcm_host)
include_directories(${BCMHOST_INCLUDE_DIRS})
link_directories(${BCMHOST_LIBRARY_DIRS} )

# Userland
include_directories(${USERLAND_DIR})
include_directories(${USERLAND_DIR}/host_applications/linux/libs/sm)

# Projectdirs
include_directories(${CMAKE_SOURCE_DIR})

# executables
add_executable(flashcamyuv RaspiStillYUV.cpp FlashCam.cpp types.c)


# Libraries
target_link_libraries(flashcamyuv ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(flashcamyuv ${OpenCV_LIBS} )
target_link_libraries(flashcamyuv ${MMAL_LIBRARIES})
target_link_libraries(flashcamyuv ${BCMHOST_LIBRARIES})
target_link_libraries(flashcamyuv m)

message("Used CXX flags: ${CMAKE_CXX_FLAGS}")
message("Used C flags: ${CMAKE_C_FLAGS}")

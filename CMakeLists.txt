cmake_minimum_required (VERSION 2.8.11)

project(flashcam)

SET(COMPILE_DEFINITIONS -Werror)

#include required packages
find_package( Threads REQUIRED )
find_package( PkgConfig REQUIRED )

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
add_executable(flashcam RaspiStill.c RaspiCamControl.c)


# Libraries
target_link_libraries(flashcam ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(flashcam ${MMAL_LIBRARIES})
target_link_libraries(flashcam ${BCMHOST_LIBRARIES})
target_link_libraries(flashcam m)
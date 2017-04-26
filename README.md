# FlashCam:
C++ Library for the PiCamera. Can be used for both video capturing and stills. 
With a 320x240 resolution, video mode is able to reach 120fps on a RPi Zero Wireless.

In progress: addition of flash in both Capture (working) and Video (todo) mode.

Based on:
- [picamera](https://github.com/waveform80/picamera/)
- [picam](https://github.com/HesselM/picam)
- [raspicam](https://github.com/cedricve/raspicam)
- [raspicam](https://github.com/raspberrypi/userland)

Requisites
- [Crosscompile Environment](https://github.com/HesselM/rpicross_notes) (but native compilation might be working too)
- [GPIO setup / flash](http://picamera.readthedocs.io/en/latest/recipes2.html?highlight=flash#using-a-flash-with-the-camera)

Optional
- [OpenCV](https://github.com/HesselM/rpicross_notes) (used is test-files)


# How to use:

For crosscompilation with [this setup](https://github.com/HesselM/rpicross_notes), the compilation steps become:

```
XCS~$ mkdir -p ~/rpi/build/flashcam
XCS~$ cd ~/rpi/build/flashcam
XCS~$ cmake \
    -D VIDTEST=ON
    -D CMAKE_TOOLCHAIN_FILE=/home/pi/rpicross_notes/rpi-generic-toolchain.cmake \
    -D CMAKE_INSTALL_PREFIX=/home/pi/rpi/rootfs/usr \
    ~/path/to/repository/flashcam/
XCS~$ make
XCS~$ scp flashcam rpizero-local:~/
XCS~$ ssh -X rpizero-local
RPi~$ ./flashcam
``` 

Use `VIDTEST=ON` or `VIDTEST=OFF` (default=`OFF`) to test either the capture or the video mode.

When compilin native (NOT TESTED!):

```
RPi~$ mkdir -p ~/build/flashcam
RPi~$ cd ~/build/flashcam
RPi~$ cmake -D VIDTEST=ON ~/path/to/repository/flashcam/
RPi~$ make
RPi~$ ./flashcam
```

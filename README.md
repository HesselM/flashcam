# FlashCam:
C++ Library for the PiCamera. Can be used for both video capturing and stills. 
With a 320x240 resolution, video mode is able to reach 120fps on a RPi Zero Wireless.


#September 2017
Library is work in progress, hence this readme is outdated. Currently functional:
- Single frame capturing (capture mode): callback to user defined function with full
- Continous frame capturing (video mode): callback to user defined function per frame.
- Phase Locked Loop (PLL): synchronised Hardware PWM with exposure time of camera. With proper tuning exposure and PWM signal can be synced within 60 microseconds. Only works in video mode. 
- OpenGL rendering: Captured frame is not pushed to CPU domain, but stays in GPU, allowing efficient application of OpenGL shaders.  

Pleas see the `CmakeLists` and `tests` directory for examples and available tests.


# OLD README:
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

Use `PLLTEST=ON` (overrides `VIDTEST`) to test PLL mode of the video stream. This mode uses the hardware PWM (GPIO18) of the Raspberry Pi to generate a PWM pulse with a configurable duty cycle at the set framerate of the camera. Running the PLL-executable requires `sudo` permissions, hence run as `root` or with `sudo` prepended. 

When compilin native (NOT TESTED!):

```
RPi~$ mkdir -p ~/build/flashcam
RPi~$ cd ~/build/flashcam
RPi~$ cmake -D VIDTEST=ON ~/path/to/repository/flashcam/
RPi~$ make
RPi~$ ./flashcam
```

# Test results

```
RPi~$ ./flashcam 
 -- VIDEO-TEST -- 

Rotation     : 270
AWB          : 1
Flash        : 0
Mirror       : 0
Camera Num   : 0
Exposure     : 6
Metering     : 0
Framerate    : 120.000000
Stabilisation: 0
DRC          : 0
Sharpness    : 0
Contrast     : 0
Brightness   : 50
Saturation   : 0
ISO          : 0
Shutterspeed : 0
AWB-red      : 0
AWB-blue     : 0
Denoise      : 1
Width        : 640
Height       : 480
Verbose      : 0
Update       : 0
Camera-Mode  : 1

...

Timing: 0.008139 (avg: 0.008297; frames: 5880; fps:120.522726)
Timing: 0.007685 (avg: 0.008297; frames: 6000; fps:120.524067)
```


tuning:
```
XCS~$ cmake     -D CMAKE_TOOLCHAIN_FILE=/home/pi/rpicross_notes/rpi-generic-toolchain.cmake -DPLLTUNE=ON ~/code/flashcam/
XCS~$ make
XCS~$ scp flashcam rpizero-local:~/
RPi~$ su -pc ./flashcam

```
Stepresponse:
```
XCS~$ cmake     -D CMAKE_TOOLCHAIN_FILE=/home/pi/rpicross_notes/rpi-generic-toolchain.cmake -DPLLTUNE=ON -DSTEPRESPONSE=ON  ~/code/flashcam/
XCS~$ make
XCS~$ scp flashcam rpizero-local:~/
RPi~$ su -pc ./flashcam
```



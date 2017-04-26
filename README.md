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

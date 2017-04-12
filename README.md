# FlashCam:
Simulated video streaming Library for a Raspberry Pi Camera utilising a Flash in C++.

Based on:
- [picamera](https://github.com/waveform80/picamera/)
- [picam](https://github.com/HesselM/picam)
- [raspicam](https://github.com/cedricve/raspicam)
- [raspicam](https://github.com/raspberrypi/userland)

Video streaming with the Raspberry Pi does not allow the use of a flash. This simple library puts the PiCamera in `still`-mode and simulates video streaming: e.g. fast triggering of consecutive snapshots while utilizing the flash. 

Requisites
- [Crosscompile Environment](https://github.com/HesselM/rpicross_notes)
- [GPIO setup / flash](http://picamera.readthedocs.io/en/latest/recipes2.html?highlight=flash#using-a-flash-with-the-camera)

Optional
- [OpenCV](https://github.com/HesselM/rpicross_notes)


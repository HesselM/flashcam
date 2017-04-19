import picamera

c = picamera.PiCamera()
c.flash_mode='on'

print "flash                :" , c.flash_mode
print "analog gain          :" , c.analog_gain
print "annotate background  :" , c.annotate_background
print "annotate foreground  :" , c.annotate_foreground
print "annotate frame num   :" , c.annotate_frame_num
print "annotate text        :" , c.annotate_text
print "annotate textsize    :" , c.annotate_text_size
print "AWB gains            :" , c.awb_gains
print "AWB mode             :" , c.awb_mode
print "Brightness           :" , c.brightness
print "Clock Mode           :" , c.clock_mode
print "Closed               :" , c.closed
print "Color Effects        :" , c.color_effects
print "Contrast             :" , c.contrast
print "Crop                 :" , c.crop
print "Digital Gain         :" , c.digital_gain
print "DRC strength         :" , c.drc_strength
print "EXIF tags            :" , c.exif_tags
#-25 to 25
print "Exposure compensation:" , c.exposure_compensation
print "Exposure mode        :" , c.exposure_mode
print "Exposure speed       :" , c.exposure_speed
print "Flash mode           :" , c.flash_mode
#print "Frame                :" , c.frame
print "Framerate            :" , c.framerate
print "Framerate_delta      :" , c.framerate_delta
print "Framerate_range      :" , c.framerate_range
print "hflip                :" , c.hflip
print "Image denoise        :" , c.image_denoise
print "Image effect         :" , c.image_effect
print "Image effect params  :" , c.image_effect_params
print "iso                  :" , c.iso
#print "led                  :" , c.led
print "meter mode           :" , c.meter_mode
print "overlays             :" , c.overlays
print "preview              :" , c.preview
print "preview_alpha        :" , c.preview_alpha
print "preview_fullscreen   :" , c.preview_fullscreen
print "preview_layer        :" , c.preview_layer
print "preview_window       :" , c.preview_window
print "previewing           :" , c.previewing
print "raw format           :" , c.raw_format
print "Recordinf            :" , c.recording
print "Resolution           :" , c.resolution
print "Revision             :" , c.revision 
print "rotation             :" , c.rotation 
print "saturation           :" , c.saturation
print "sensor_mode          :" , c.sensor_mode
print "sharpness            :" , c.sharpness
print "shutter_speed        :" , c.shutter_speed
print "still_stats          :" , c.still_stats
print "timestamp            :" , c.timestamp
print "vflip                :" , c.vflip
print "video_denoise        :" , c.video_denoise
print "video_stabilization  :" , c.video_stabilization
print "zoom                 :" , c.zoom

c.capture('test.jpg')

print "flash                :" , c.flash_mode
print "analog gain          :" , c.analog_gain
print "annotate background  :" , c.annotate_background
print "annotate foreground  :" , c.annotate_foreground
print "annotate frame num   :" , c.annotate_frame_num
print "annotate text        :" , c.annotate_text
print "annotate textsize    :" , c.annotate_text_size
print "AWB gains            :" , c.awb_gains
print "AWB mode             :" , c.awb_mode
print "Brightness           :" , c.brightness
print "Clock Mode           :" , c.clock_mode
print "Closed               :" , c.closed
print "Color Effects        :" , c.color_effects
print "Contrast             :" , c.contrast
print "Crop                 :" , c.crop
print "Digital Gain         :" , c.digital_gain
print "DRC strength         :" , c.drc_strength
print "EXIF tags            :" , c.exif_tags
#-25 to 25
print "Exposure compensation:" , c.exposure_compensation
print "Exposure mode        :" , c.exposure_mode
print "Exposure speed       :" , c.exposure_speed
print "Flash mode           :" , c.flash_mode
#print "Frame                :" , c.frame
print "Framerate            :" , c.framerate
print "Framerate_delta      :" , c.framerate_delta
print "Framerate_range      :" , c.framerate_range
print "hflip                :" , c.hflip
print "Image denoise        :" , c.image_denoise
print "Image effect         :" , c.image_effect
print "Image effect params  :" , c.image_effect_params
print "iso                  :" , c.iso
#print "led                  :" , c.led
print "meter mode           :" , c.meter_mode
print "overlays             :" , c.overlays
print "preview              :" , c.preview
print "preview_alpha        :" , c.preview_alpha
print "preview_fullscreen   :" , c.preview_fullscreen
print "preview_layer        :" , c.preview_layer
print "preview_window       :" , c.preview_window
print "previewing           :" , c.previewing
print "raw format           :" , c.raw_format
print "Recordinf            :" , c.recording
print "Resolution           :" , c.resolution
print "Revision             :" , c.revision 
print "rotation             :" , c.rotation 
print "saturation           :" , c.saturation
print "sensor_mode          :" , c.sensor_mode
print "sharpness            :" , c.sharpness
print "shutter_speed        :" , c.shutter_speed
print "still_stats          :" , c.still_stats
print "timestamp            :" , c.timestamp
print "vflip                :" , c.vflip
print "video_denoise        :" , c.video_denoise
print "video_stabilization  :" , c.video_stabilization
print "zoom                 :" , c.zoom

'''
    pi@rpizw-hessel:~$ python picamread.py 
    flash                : on
    analog gain          : 0
    annotate background  : None
    annotate foreground  : #ffffff
    annotate frame num   : False
    annotate text        : 
    annotate textsize    : 32
    AWB gains            : (Fraction(0, 1), Fraction(0, 1))
    AWB mode             : auto
    Brightness           : 50
    Clock Mode           : reset
    Closed               : False
    Color Effects        : None
    Contrast             : 0
    Crop                 : (0.0, 0.0, 1.0, 1.0)
    Digital Gain         : 0
    DRC strength         : off
    EXIF tags            : {u'IFD0.Make': u'RaspberryPi', u'IFD0.Model': u'RP_imx219'}
    Exposure compensation: 0
    Exposure mode        : auto
    Exposure speed       : 0
    Flash mode           : on
    Framerate            : 30
    Framerate_delta      : 0
    Framerate_range      : 30..30
    hflip                : False
    Image denoise        : True
    Image effect         : none
    Image effect params  : None
    iso                  : 0
    meter mode           : average
    overlays             : []
    preview              : None
    preview_alpha        : 255
    preview_fullscreen   : True
    preview_layer        : 2
    preview_window       : None
    previewing           : False
    raw format           : yuv
    Recordinf            : False
    Resolution           : 720x480
    Revision             : imx219
    rotation             : 0
    saturation           : 0
    sensor_mode          : 0
    sharpness            : 0
    shutter_speed        : 0
    still_stats          : False
    timestamp            : 4693243912
    vflip                : False
    video_denoise        : True
    video_stabilization  : False
    zoom                 : (0.0, 0.0, 1.0, 1.0)
    
    
    flash                : on
    analog gain          : 1
    annotate background  : None
    annotate foreground  : #ffffff
    annotate frame num   : False
    annotate text        : 
    annotate textsize    : 32
    AWB gains            : (Fraction(221, 256), Fraction(163, 128))
    AWB mode             : auto
    Brightness           : 50
    Clock Mode           : reset
    Closed               : False
    Color Effects        : None
    Contrast             : 0
    Crop                 : (0.0, 0.0, 1.0, 1.0)
    Digital Gain         : 195/256
    DRC strength         : off
    EXIF tags            : {u'IFD0.Make': u'RaspberryPi', u'IFD0.Model': u'RP_imx219'}
    Exposure compensation: 0
    Exposure mode        : auto
    Exposure speed       : 26872
    Flash mode           : on
    Framerate            : 30
    Framerate_delta      : 0
    Framerate_range      : 30..30
    hflip                : False
    Image denoise        : True
    Image effect         : none
    Image effect params  : None
    iso                  : 0
    meter mode           : average
    overlays             : []
    preview              : None
    preview_alpha        : 255
    preview_fullscreen   : True
    preview_layer        : 2
    preview_window       : None
    previewing           : False
    raw format           : yuv
    Recordinf            : False
    Resolution           : 720x480
    Revision             : imx219
    rotation             : 0
    saturation           : 0
    sensor_mode          : 0
    sharpness            : 0
    shutter_speed        : 0
    still_stats          : False
    timestamp            : 4694030315
    vflip                : False
    video_denoise        : True
    video_stabilization  : False
    zoom                 : (0.0, 0.0, 1.0, 1.0)
    '''

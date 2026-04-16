ioctl: VIDIOC_ENUM_FMT
	Index       : 0
	Type        : Video Capture
	Pixel Format: 'GREY'
	Name        : 8-bit Greyscale
		Size: Discrete 1936x1216
			Interval: Continuous 0.029s - 100000.000s (0.000-34.715 fps)

	Index       : 1
	Type        : Video Capture
	Pixel Format: 'RGGB'
	Name        : 8-bit Bayer RGRG/GBGB
		Size: Discrete 1936x1216
			Interval: Continuous 0.029s - 100000.000s (0.000-34.715 fps)

	Index       : 2
	Type        : Video Capture
	Pixel Format: 'Y10 '
	Name        : 10-bit Greyscale
		Size: Discrete 1936x1216
			Interval: Continuous 0.029s - 100000.000s (0.000-34.715 fps)

	Index       : 3
	Type        : Video Capture
	Pixel Format: 'RG10'
	Name        : 10-bit Bayer RGRG/GBGB
		Size: Discrete 1936x1216
			Interval: Continuous 0.029s - 100000.000s (0.000-34.715 fps)

	Index       : 4
	Type        : Video Capture
	Pixel Format: 'Y12 '
	Name        : 12-bit Greyscale
		Size: Discrete 1936x1216
			Interval: Continuous 0.029s - 100000.000s (0.000-34.715 fps)

	Index       : 5
	Type        : Video Capture
	Pixel Format: 'RG12'
	Name        : 12-bit Bayer RGRG/GBGB
		Size: Discrete 1936x1216
			Interval: Continuous 0.029s - 100000.000s (0.000-34.715 fps)

	Index       : 6
	Type        : Video Capture
	Pixel Format: 'BX24'
	Name        : 32-bit XRGB 8-8-8-8
		Size: Discrete 1936x1216
			Interval: Continuous 0.029s - 100000.000s (0.000-34.715 fps)

	Index       : 7
	Type        : Video Capture
	Pixel Format: 'XR24'
	Name        : 32-bit BGRX 8-8-8-8
		Size: Discrete 1936x1216
			Interval: Continuous 0.029s - 100000.000s (0.000-34.715 fps)

	Index       : 8
	Type        : Video Capture
	Pixel Format: 'VYUY'
	Name        : VYUY 4:2:2
		Size: Discrete 1936x1216
			Interval: Continuous 0.029s - 100000.000s (0.000-34.715 fps)

GREY  10-bit Greyscale          V4L2_PIX_FMT_GREY    v4l2_fourcc('G', 'R', 'E', 'Y') /*  8  Greyscale     */
RGGB  8-bit Bayer RGRG/GBGB	V4L2_PIX_FMT_SRGGB8  v4l2_fourcc('R', 'G', 'G', 'B') /*  8  RGRG.. GBGB.. */
Y10   10-bit Greyscale          V4L2_PIX_FMT_Y10     v4l2_fourcc('Y', '1', '0', ' ') /* 10  Greyscale     */
RG10  10-bit Bayer RGRG/GBGB	V4L2_PIX_FMT_SRGGB10 v4l2_fourcc('R', 'G', '1', '0') /* 10  RGRG.. GBGB.. */
Y12   12-bit Greyscale          V4L2_PIX_FMT_Y12     v4l2_fourcc('Y', '1', '2', ' ') /* 12  Greyscale     */
RG12  12-bit Bayer RGRG/GBGB    V4L2_PIX_FMT_SRGGB12 v4l2_fourcc('R', 'G', '1', '2') /* 12  RGRG.. GBGB.. */
BX24  32-bit XRGB 8-8-8-8       V4L2_PIX_FMT_XRGB32  v4l2_fourcc('B', 'X', '2', '4') /* 32  XRGB-8-8-8-8  */
XR24  32-bit BGRX 8-8-8-8	V4L2_PIX_FMT_XBGR32  v4l2_fourcc('X', 'R', '2', '4') /* 32  BGRX-8-8-8-8  */
VYUY  VYUY 4:2:2                V4L2_PIX_FMT_VYUY    v4l2_fourcc('V', 'Y', 'U', 'Y') /* 16  YUV 4:2:2     */

################################################################################################################
$  v4l2-ctl -d /dev/video0 --all

Driver Info (not using libv4l2):
	Driver name   : avt_tegra_csi2
	Card type     : ALVIUM 1800 C-240c 8-3c
	Bus info      : platform:54080000.vi:4
	Driver version: 4.9.253
	Capabilities  : 0x85200001
		Video Capture
		Read/Write
		Streaming
		Extended Pix Format
		Device Capabilities
	Device Caps   : 0x05200001
		Video Capture
		Read/Write
		Streaming
		Extended Pix Format
Priority: 2
Video input : 0 (Camera 4: ok)
Format Video Capture:
	Width/Height      : 1920/1216
	Pixel Format      : 'XR24'
	Field             : None
	Bytes per Line    : 7680
	Size Image        : 9338880
	Colorspace        : sRGB
	Transfer Function : Default (maps to sRGB)
	YCbCr/HSV Encoding: Default (maps to ITU-R 601)
	Quantization      : Default (maps to Full Range)
	Flags             : 
Crop Capability Video Capture:
	Bounds      : Left 0, Top 0, Width 1936, Height 1216
	Default     : Left 0, Top 0, Width 1936, Height 1216
	Pixel Aspect: 1/1
Crop: Left 0, Top 0, Width 1920, Height 1216
Selection: crop, Left 0, Top 0, Width 1920, Height 1216
Selection: crop_default, Left 0, Top 0, Width 1936, Height 1216
Selection: crop_bounds, Left 0, Top 0, Width 1936, Height 1216
Selection: compose, Left 0, Top 0, Width 1920, Height 1216
Selection: compose_default, Left 0, Top 0, Width 1920, Height 1216
Selection: compose_bounds, Left 0, Top 0, Width 1920, Height 1216
Selection: native_size, Left 0, Top 0, Width 1936, Height 1216
Streaming Parameters Video Capture:
	Capabilities     : timeperframe
	Frames per second: invalid (0/1000)
	Read buffers     : 1

User Controls

                     brightness 0x00980900 (int)    : min=0 max=255 step=1 default=0 value=0 flags=slider
                     saturation 0x00980902 (int)    : min=0 max=200 step=1 default=100 value=100 flags=slider
                            hue 0x00980903 (int)    : min=-4000 max=4000 step=1 default=0 value=0 flags=slider
             auto_white_balance 0x0098090c (bool)   : default=0 value=0
                  white_balance 0x0098090d (button) : flags=write-only, execute-on-write
                    red_balance 0x0098090e (int64)  : min=0 max=8000 step=1 default=2354 value=1093 flags=slider
                   blue_balance 0x0098090f (int64)  : min=0 max=8000 step=1 default=2100 value=2356 flags=slider
                          gamma 0x00980910 (int64)  : min=40 max=240 step=5 default=100 value=100 flags=slider
                       exposure 0x00980911 (int64)  : min=168727 max=10000006303 step=1 default=5017697 value=11487192 flags=slider
                      gain_auto 0x00980912 (bool)   : default=0 value=0
                           gain 0x00980913 (int64)  : min=0 max=2400 step=1 default=0 value=1560 flags=slider
                      reverse_x 0x00980914 (bool)   : default=0 value=0
                      reverse_y 0x00980915 (bool)   : default=0 value=0
          frame_timeout_enabled 0x009809c8 (bool)   : default=1 value=1
                  frame_timeout 0x009809c9 (int)    : min=100 max=12000 step=1 default=12000 value=12000
       stride_alignment_enabled 0x009809ca (bool)   : default=1 value=1
         crop_alignment_enabled 0x009809cb (bool)   : default=1 value=1
          value_update_interval 0x009809cc (int)    : min=0 max=60000 step=1 default=1000 value=1000
             force_value_update 0x009809cd (button) : flags=write-only, execute-on-write

Camera Controls

                  exposure_auto 0x009a0901 (menu)   : min=0 max=1 default=1 value=1
              exposure_absolute 0x009a0902 (int)    : min=1 max=100000 step=1 default=50 value=114 flags=slider
              exposure_auto_min 0x009a0928 (int64)  : min=168727 max=10000006303 step=1 default=168727 value=168727 flags=slider
              exposure_auto_max 0x009a0929 (int64)  : min=168727 max=10000006303 step=1 default=10000006303 value=10000006303 flags=slider
                  gain_auto_min 0x009a092a (int64)  : min=0 max=2400 step=1 default=0 value=0 flags=slider
                  gain_auto_max 0x009a092b (int64)  : min=0 max=2400 step=1 default=2400 value=2400 flags=slider
      exposure_active_line_mode 0x009a092c (bool)   : default=0 value=0
  exposure_active_line_selector 0x009a092d (int)    : min=0 max=1 step=1 default=1 value=1 flags=slider
         exposure_active_invert 0x009a092e (bool)   : default=0 value=0
                   trigger_mode 0x009a092f (bool)   : default=0 value=0
             trigger_activation 0x009a0930 (menu)   : min=0 max=4 default=0 value=0
                 trigger_source 0x009a0931 (menu)   : min=0 max=4 default=4 value=4
               trigger_software 0x009a0932 (button) : flags=inactive, write-only, execute-on-write
             device_temperature 0x009a0933 (int)    : min=-1000 max=2000 step=1 default=290 value=640 flags=volatile
                    bypass_mode 0x009a2064 (intmenu): min=0 max=1 default=0 value=0
                override_enable 0x009a2065 (intmenu): min=0 max=1 default=0 value=0
                   height_align 0x009a2066 (int)    : min=1 max=16 step=1 default=1 value=1
                     size_align 0x009a2067 (intmenu): min=0 max=2 default=0 value=0
               write_isp_format 0x009a2068 (bool)   : default=0 value=0
       sensor_signal_properties 0x009a2069 (u32)    : min=0 max=4294967295 step=1 default=0 [30][18] flags=read-only, has-payload
        sensor_image_properties 0x009a206a (u32)    : min=0 max=4294967295 step=1 default=0 [30][16] flags=read-only, has-payload
      sensor_control_properties 0x009a206b (u32)    : min=0 max=4294967295 step=1 default=0 [30][36] flags=read-only, has-payload
              sensor_dv_timings 0x009a206c (u32)    : min=0 max=4294967295 step=1 default=0 [30][16] flags=read-only, has-payload
               low_latency_mode 0x009a206d (bool)   : default=0 value=0
               preferred_stride 0x009a206e (int)    : min=0 max=65535 step=1 default=0 value=0
                   sensor_modes 0x009a2082 (int)    : min=0 max=30 step=1 default=30 value=1 flags=read-only




https://techoverflow.net/2022/11/01/how-to-set-v4l2-exposure-to-manual-mode-in-opencv-python/

How to set V4L2 exposure to manual mode in OpenCV & Python
Using OpenCV on Linux, if you have a video device that interfaces a V4L2 device such as a USB webcam:

set-v4l2-exposure-to-manual-modeopencv-python.py📋 Copy to clipboard⇓ Download
camera = cv2.VideoCapture(0)
you can typically set the automatic exposure mode by setting exposure_auto to 1 (the following output is from v4l2-ctl -d /dev/video0 --all):

set-v4l2-exposure-to-manual-modeopencv-python.txt📋 Copy to clipboard⇓ Download
exposure_auto 0x009a0901 (menu)   : min=0 max=3 default=3 value=1
              1: Manual Mode
              3: Aperture Priority Mode
As you can see in our previous blogpost, exposure_auto (which is named V4L2_CID_EXPOSURE_AUTO in V4L2 in C/C++) is mapped to CAP_PROP_AUTO_EXPOSURE.

Therefore, you can enable manual exposure using

set-v4l2-exposure-to-manual-modeopencv-python.py📋 Copy to clipboard⇓ Download
camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) # Set exposure to manual mode
You should, however, verify these settings using v4l2-ctl --all using your specific camera.

If this post helped you, please consider buying me a coffee or donating via PayPal to support research & publishing of new posts on TechOverflow








How are OpenCV CAP_PROP_… mapped to V4L2 ctrls / parameters?
From both the OpenCV documentation and the V4L2 documentation, it is unclear how all the CAP_PROP_... parameters are mapped to v4l2 controls such as exposure_absolute.

However, you can easily look in the source code (int capPropertyToV4L2(int prop) in cap_v4l.cpp) in order to see how the parameters are mapped internally. Github link to the source code

CAP_PROP_FRAME_COUNT	V4L2_CID_MPEG_VIDEO_B_FRAMES
CAP_PROP_BRIGHTNESS	V4L2_CID_BRIGHTNESS
CAP_PROP_CONTRAST	V4L2_CID_CONTRAST
CAP_PROP_SATURATION	V4L2_CID_SATURATION
CAP_PROP_HUE	V4L2_CID_HUE
cv2.CAP_PROP_GAIN	V4L2_CID_GAIN
cv2.CAP_PROP_EXPOSURE	V4L2_CID_EXPOSURE_ABSOLUTE
CAP_PROP_WHITE_BALANCE_BLUE_U	V4L2_CID_BLUE_BALANCE
CAP_PROP_SHARPNESS	V4L2_CID_SHARPNESS
CAP_PROP_AUTO_EXPOSURE	V4L2_CID_EXPOSURE_AUTO
CAP_PROP_GAMMA	V4L2_CID_GAMMA
CAP_PROP_TEMPERATURE	V4L2_CID_WHITE_BALANCE_TEMPERATURE
CAP_PROP_WHITE_BALANCE_RED_V	V4L2_CID_RED_BALANCE
CAP_PROP_ZOOM	V4L2_CID_ZOOM_ABSOLUTE
CAP_PROP_FOCUS	V4L2_CID_FOCUS_ABSOLUTE
CAP_PROP_ISO_SPEED	V4L2_CID_ISO_SENSITIVITY
CAP_PROP_BACKLIGHT	V4L2_CID_BACKLIGHT_COMPENSATION
CAP_PROP_PAN	V4L2_CID_PAN_ABSOLUTE
CAP_PROP_TILT	V4L2_CID_TILT_ABSOLUTE
CAP_PROP_ROLL	V4L2_CID_ROTATE
CAP_PROP_IRIS	V4L2_CID_IRIS_ABSOLUTE
CAP_PROP_AUTOFOCUS	V4L2_CID_FOCUS_AUTO
CAP_PROP_SAR_NUM	V4L2_CID_MPEG_VIDEO_H264_VUI_EXT_SAR_HEIGHT
CAP_PROP_SAR_DEN	V4L2_CID_MPEG_VIDEO_H264_VUI_EXT_SAR_WIDTH
CAP_PROP_AUTO_WB	V4L2_CID_AUTO_WHITE_BALANCE
CAP_PROP_WB_TEMPERATURE	V4L2_CID_WHITE_BALANCE_TEMPERATURE


video.get(cv2.CAP_PROP_GAIN)
video.get(cv2.CAP_PROP_EXPOSURE)
video.set(cv2.CAP_PROP_EXPOSURE, 200)


enum VideoCaptureProperties
{
    CAP_PROP_POS_MSEC             =0,
    CAP_PROP_POS_FRAMES           =1,
    CAP_PROP_POS_AVI_RATIO        =2,
    CAP_PROP_FRAME_WIDTH          =3,
    CAP_PROP_FRAME_HEIGHT         =4,
    CAP_PROP_FPS                  =5,
    CAP_PROP_FOURCC               =6,
    CAP_PROP_FRAME_COUNT          =7,
    CAP_PROP_FORMAT               =8,
    CAP_PROP_MODE                 =9,
    CAP_PROP_BRIGHTNESS           =10,
    CAP_PROP_CONTRAST             =11,
    CAP_PROP_SATURATION           =12,
    CAP_PROP_HUE                  =13,
    CAP_PROP_GAIN                 =14,
    CAP_PROP_EXPOSURE             =15,
    CAP_PROP_CONVERT_RGB          =16,
    CAP_PROP_WHITE_BALANCE_BLUE_U =17,
    CAP_PROP_RECTIFICATION        =18,
    CAP_PROP_MONOCHROME           =19,
    CAP_PROP_SHARPNESS            =20,
    CAP_PROP_AUTO_EXPOSURE        =21,
    CAP_PROP_GAMMA                =22,
    CAP_PROP_TEMPERATURE          =23,
    CAP_PROP_TRIGGER              =24,
    CAP_PROP_TRIGGER_DELAY        =25,
    CAP_PROP_WHITE_BALANCE_RED_V  =26,
    CAP_PROP_ZOOM                 =27,
    CAP_PROP_FOCUS                =28,
    CAP_PROP_GUID                 =29,
    CAP_PROP_ISO_SPEED            =30,
    CAP_PROP_BACKLIGHT            =32,
    CAP_PROP_PAN                  =33,
    CAP_PROP_TILT                 =34,
    CAP_PROP_ROLL                 =35,
    CAP_PROP_IRIS                 =36,
    CAP_PROP_SETTINGS             =37,
    CAP_PROP_BUFFERSIZE           =38,
    CAP_PROP_AUTOFOCUS            =39,
};


    def set_auto_exposure(self, auto) -> bool:
        if self.__is_on_linux():
            if type(auto) is bool:
                _auto = 3 if auto else 1
            else:
                _auto = auto
            code = self.__v4l2_ctl_command('exposure_auto', _auto)
            if code == 0:
                return True
        if type(auto) is bool:
            return self.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75 if auto else 0.25)
        return self.set(cv2.CAP_PROP_AUTO_EXPOSURE, auto)

    def get_data(self):


# SeeCam


maneesh@ubuntu:~/SeeCam/python$ v4l2-ctl -d /dev/video1 --list-formats-ext
ioctl: VIDIOC_ENUM_FMT
	Type: Video Capture

	[0]: 'UYVY' (UYVY 4:2:2)
		Size: Discrete 1280x720
			Interval: Discrete 0.067s (15.000 fps)
		Size: Discrete 1920x1080
			Interval: Discrete 0.125s (8.000 fps)
		Size: Discrete 1920x1200
			Interval: Discrete 0.167s (6.000 fps)
	[1]: 'MJPG' (Motion-JPEG, compressed)
		Size: Discrete 1280x720
			Interval: Discrete 0.008s (120.000 fps)
			Interval: Discrete 0.017s (60.000 fps)
		Size: Discrete 1920x1080
			Interval: Discrete 0.008s (120.000 fps)
			Interval: Discrete 0.017s (60.000 fps)
			Interval: Discrete 0.033s (30.000 fps)
		Size: Discrete 1920x1200
			Interval: Discrete 0.009s (114.000 fps)
			Interval: Discrete 0.017s (60.000 fps)

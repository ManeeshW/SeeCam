import cv2
import time
import v4l2
import fcntl

#video = cv2.VideoCapture(0)

video = cv2.VideoCapture()

video.open(0, apiPreference=cv2.CAP_V4L2)
video.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
video.set(cv2.CAP_PROP_AUTO_WB, 1)




vd = open('/dev/video0', 'rb+', buffering=0)
qctrl = v4l2.v4l2_queryctrl()
ctrl = v4l2.v4l2_control()
qctrl.id = v4l2.V4L2_CID_EXPOSURE_AUTO
x = v4l2.V4L2_CID_EXPOSURE_AUTO
y = v4l2.V4L2_CID_AUTOGAIN
z = v4l2.V4L2_CID_AUTO_WHITE_BALANCE
fcntl.ioctl(vd, v4l2.VIDIOC_QUERYCTRL, qctrl)
ctrl.id = qctrl.id
fcntl.ioctl(vd, v4l2.VIDIOC_G_CTRL, ctrl)
print(ctrl.id)


fcntl.ioctl(vd, v4l2.VIDIOC_S_CTRL, ctrl)
#b = (ctrl.id, ctrl.value, qctrl.minimum, qctrl.maximum)

#video.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
#video.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
#video.set(cv2.CAP_PROP_FRAME_HEIGHT, 1216)
#video.set(cv2.CAP_PROP_FPS, 30.0)

#now set the camera exposure to -4 ( means 2^-4 = 1/16 = 80 ms)
#video.get(cv2.CAP_PROP_FRAME_WIDTH)
#video.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
#video.set(cv2.CAP_PROP_EXPOSURE, 200)
#video.set(cv2.CAP_PROP_GAIN, -4)
print("CAP_PROP_FOURCC : ",video.get(cv2.CAP_PROP_FOURCC))
print("CAP_PROP_FRAME_WIDTH : ",video.get(cv2.CAP_PROP_FRAME_WIDTH))
print("CAP_PROP_FRAME_HEIGHT : ",video.get(cv2.CAP_PROP_FRAME_HEIGHT))
print("CAP_PROP_FPS : ",video.get(cv2.CAP_PROP_FPS))
print("CAP_PROP_AUTO_EXPOSURE : ",video.get(cv2.CAP_PROP_AUTO_EXPOSURE))
print("CID_EXPOSURE_ABSOLUTE : ",video.get(cv2.CAP_PROP_EXPOSURE))
print("CAP_PROP_GAIN : ",video.get(cv2.CAP_PROP_GAIN))
print("CAP_PROP_BRIGHTNESS : ",video.get(cv2.CAP_PROP_BRIGHTNESS))
print("CAP_PROP_AUTO_WB : ",video.get(cv2.CAP_PROP_AUTO_WB))
print("CAP_PROP_ISO_SPEED : ",video.get(cv2.CAP_PROP_ISO_SPEED))




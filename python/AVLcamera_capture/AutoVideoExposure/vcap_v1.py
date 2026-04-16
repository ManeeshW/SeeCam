import cv2
import time
import v4l2
import fcntl

#video = cv2.VideoCapture(0)

video = cv2.VideoCapture()

video.open(0, apiPreference=cv2.CAP_V4L2)
video.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
video.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
video.set(cv2.CAP_PROP_FRAME_HEIGHT, 1216)
video.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
video.set(cv2.CAP_PROP_AUTO_WB, 1)
video.set(cv2.CAP_PROP_FPS, 24)
#video.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)

#video.get(cv2.CAP_PROP_FRAME_WIDTH)
video.get(cv2.CAP_PROP_FRAME_WIDTH)
video.get(cv2.CAP_PROP_AUTO_EXPOSURE)

print('Starting video-capture test...')

t0 = time.time()
for i in range(10):
    success, image = video.read()
    #ret, jpeg = cv2.imencode('.jpg',image)
    cv2.imwrite("result.png", image)
t1 = time.time()
t = ( t1 - t0 ) / 10.0
fps = 1.0 / t

print('Test finished. ' + str(t) + ' sec. per img.')
print(str( fps ) + ' fps reached')

video.release()

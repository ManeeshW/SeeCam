import cv2
import numpy as np

f = open('/home/fdcl/AVLcamera_capture/sample/content/88.jpg', 'rb')
image_bytes = f.read()  # b'\xff\xd8\xff\xe0\x00\x10...'
buf = np.frombuffer(image_bytes, np.uint8)
print(buf.shape)
decoded = cv2.imdecode(buf, -1)

print(decoded.shape)

import numpy as np
import cv2
  
img = cv2.imread('/home/fdcl/content/wow2.jpg')
print(img.shape)
img_encode = cv2.imencode('.jpg', img)[1]
  
data_encode = np.array(img_encode)
byte_encode = data_encode.tobytes()
  
#print(byte_encode)

import cv2
import numpy as np

f = open('/home/fdcl/content/1212.jpg', 'rb')
image_bytes = f.read()  # b'\xff\xd8\xff\xe0\x00\x10...'
buf = np.frombuffer(image_bytes, np.uint8)
print(buf.shape)
decoded = cv2.imdecode(buf, -1)

print(decoded.shape)

# your Pillow code
#import io
#from PIL import Image
#image = np.array(Image.open(io.BytesIO(image_bytes))) 
#print('PIL:\n', image)

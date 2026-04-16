import cv2
import numpy as np
import os
from datetime import datetime
import time


class Camera(object):
    def __init__(self):
        os.system("v4l2-ctl --list-devices")
        self.video = cv2.VideoCapture()
        self.video.open(2, apiPreference=cv2.CAP_V4L2)

        # === CRITICAL: Set these BEFORE width/height ===
        #self.video.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))  # YUYV = most reliable for OpenCV
        # Alternative good options if YUYV doesn't look perfect:
        self.video.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('X', 'R', '2', '4'))  # XR24 = 32-bit BGRX
        # self.video.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('A', 'B', '2', '4'))  # AB24 = 32-bit RGBA

        self.video.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.video.set(cv2.CAP_PROP_FRAME_HEIGHT, 1216)

        self._folder_name = datetime.now().strftime('capture_%Y%m%d_%H%M%S')
        #creating camera capture directory
        self.parent_dir = "/home/maneesh/AVLcamera_capture/img_data_bundles/"
        self.directory = self._folder_name
        self.path = os.path.join(self.parent_dir, self.directory)
        os.mkdir(self.path)

        # Check if camera opened successfully
        if (self.video.isOpened()== False): 
            print("Error opening video stream or file")
 

        print('Camera 02: initialized')

    def start_capturing(self):
        ret, color_image = self.video.read()

        
        #dim = (640,480)
        #resized = cv2.resize(crop_img, dim, interpolation = cv2.INTER_AREA)
        f = 2.25
        dim = (int(1936/f),int(1216/f))
        color_image = color_image[64:1216,48:1936] #experimental FOV balance by cropping unwanted pixels
        #print(color_image.shape) #(1156, 1870, 3)
        (h, w, ch) = color_image.shape
        
        w_s = int((w - (640*f))//2)
        w_e = int((w + (640*f))//2)
        h_s = int((h - (480*f))//2)
        h_e = int((h + (480*f))//2)
        #print(w_s, w_e,w_s + w_e, "|", h_s, h_e,h_s + h_e)
        crop_img = color_image[h_s:h_e,w_s:w_e]
        #print(crop_img.shape) #(1152, 1536, 3)
        f = 2.25
        dim = (int(crop_img.shape[1]/f),int(crop_img.shape[0]/f))
        resized = cv2.resize(crop_img, dim, interpolation = cv2.INTER_AREA)
        #print(resized.shape)
        return ret, resized

    def liveRecording(self):
        freq = 5.2
        desired_dt = 1.0 / freq
        avg_number = 100
        f = freq
        count = 0
        img_no = 0
        t_pre = time.time()
        # Read until video is completed
        while(self.video.isOpened()):
            #t = time.time()
            # Capture frame-by-frame
            #dt = (t - t_pre)
            #if dt < desired_dt:
            #    continue
            ret, frame = self.start_capturing()
            #freq = (freq * (avg_number - 1) + (1.0 / dt)) / avg_number
            #t_pre = t
            count+=1
            if ret == True:
                
                # Using cv2.putText() method
                org = (320, 240)
                if count < 50:
                    color = (0, 0, 250)
                else:
                    color = (0, 255, 0)
                image = frame.copy()
                #image = cv2.flip(image,1)
                (h,w,ch) = image.shape
                #image = cv2.line(image, (20,0), (20,h), (0, 250, 0), 1)
                image = cv2.line(image, (0,h//2), (w,h//2), (250, 0, 250), 1)
                #image = cv2.line(image, (0,h//3), (w,h//3), (120, 0, 250), 1)
                image = cv2.line(image, (w//2,0), (w//2,h), (56, 250, 0), 1)
                #image = cv2.line(image, (w//2-20,h//2), (w//2+20,h//2), (0, 0, 250), 2)
                #image = cv2.line(image, (w//2,h//2+20), (w//2,h//2-20), (0, 0, 250), 2)
                #image = cv2.circle(image,(320,240),6, color, 3,)
                image = cv2.putText(image, '{}'.format(count), org, cv2.FONT_HERSHEY_SIMPLEX, 2, color, 2, cv2.LINE_AA)
                #image = cv2.putText(image, '{}'.format(freq), org, cv2.FONT_HERSHEY_SIMPLEX, 2, color, 2, cv2.LINE_AA)
                # Display the resulting frame
               
                cv2.imshow('Frame',image)

                if count == 60:
                    count = 0
                    img_no += 1
                    img_name = "/{:d}.jpg".format(img_no)
                    cv2.imwrite(self.path+img_name,frame) 

                # Press Q on keyboard to  exit
                if cv2.waitKey(25) & 0xFF == ord('q'):
                    break
        
            # Break the loop
            else: 
                break
        
        # When everything done, release the video capture object
        self.video.release()
        
        # Closes all the frames
        cv2.destroyAllWindows()

if __name__ == "__main__":
    cam = Camera()
    cam.liveRecording()


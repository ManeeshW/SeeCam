import numpy as np
import cv2
import threading
import os
from datetime import datetime
from common_types import Cam2Data, nano

import time
import os

class AvlCamera(threading.Thread):
    def __init__(self, thread_id, t0, enabled, freq=5.05):
        threading.Thread.__init__(self)
        self.thread_id = thread_id
        self._lock = threading.Lock()

        self._on = True
        self._enabled = enabled
        self._t0 = nano.cam2.t_stamp
        print('Time stamp of the RPI recieved : ', nano.cam2.t_stamp)
        self._t = (datetime.now() - t0).total_seconds()
        self._folder_name = datetime.now().strftime('capture_%Y%m%d_%H%M%S')
        self._freq = freq
        self._desired_dt = 1.0 / freq
        self._logON = 0
        self._idx = 0

        # self.avlcam = Camera("/dev/video0")
        # self.avlcam.cam_get_controls()
        # self.avlcam.cam_set_controls()
        
        os.system("v4l2-ctl --list-devices")
        self.video = cv2.VideoCapture()

        self.video.open(0, apiPreference=cv2.CAP_V4L2)
        #self.video.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.video.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('X', 'R', '2', '4'))  # XR24
        self.video.set(cv2.CAP_PROP_FRAME_WIDTH, 1936)
        self.video.set(cv2.CAP_PROP_FRAME_HEIGHT, 1216)
        # self.video.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
        # self.video.set(cv2.CAP_PROP_AUTO_WB, 1)
        # self.video.set(cv2.CAP_PROP_FPS, 24)

        print('Camera 02: initialized')

    def run(self):
        if not self._enabled:
            print('Camera has been disabled from settings'
                '\n\tExiting thread')
            return

        freq = self._freq
        t = datetime.now()
        t_pre = datetime.now() #nano.cam2.t_stamp
        off = False
        avg_number = 100.0

        
        #creating camera capture directory
        parent_dir = "/home/maneesh/AVLcamera_capture/cam_data2/"
        directory = self._folder_name
        path = os.path.join(parent_dir, directory)
        os.mkdir(path)
        t_pre = time.time()
        t0 = t_pre
        idx = 0

        print("Waiting ....")
        while off:
            off = nano.on
            continue

        print("Image Capturing loop has started")
        while self._on:
            t = time.time()
            dt = (t - t_pre)
            if dt < self._desired_dt:
                continue

            freq = (freq * (avg_number - 1) + (1.0 / dt)) / avg_number
            t_pre = t

            #color_image = cv2.imread('/home/fdcl/AVLcamera_capture/sample/content/88.jpg')
            time.sleep(0.1)
            idx += 1
            img_name = "/{:d}.png".format(idx)
            #print('img ', idx, " | freq : ",float(freq),'Hz  ', img_name)
            success, color_image = self.video.read() #<-----------------------------x
            f = 2.25
            dim = (int(1936/f),int(1216/f))
            color_image = color_image[64:1216,48:1936]
            (h, w, ch) = color_image.shape
            w_s = int((w - (640*f))//2)
            w_e = int((w + (640*f))//2)
            h_s = int((h - (480*f))//2)
            h_e = int((h + (480*f))//2)

            crop_img = color_image[h_s:h_e,w_s:w_e]

            dim = (int(crop_img.shape[1]/f),int(crop_img.shape[0]/f))
            resized = cv2.resize(crop_img, dim, interpolation = cv2.INTER_AREA)
            color_image = resized

            # Write images
            cv2.imwrite(path+img_name,color_image) #<-----------------------------x
            # Show images
            #cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            #cv2.imshow('RealSense', color_image)
            #cv2.waitKey(1)
            with self._lock:
                nano.cam2.t = (t - t0)
                nano.cam2.freq = int(freq)
                nano.cam2.idx = idx
                # self._t = (t - self._t0).total_seconds()
                # self._freq = int(freq)
                # self._idx = idx
                # self.update_data()
                print('img ', idx , " | freq : ",float(freq),'Hz   ', )
            t2 = time.time()
            T = (t2 - t0)
            f = 1/T
            t0 = t2
            #print("--- %s seconds ---" % T )
            #print("--- %s Hz ---" % f )
        print('Avl camera: thread closed')

    # def update_data(self):
    #     data = Cam2Data(
    #         self._t,
    #         self._freq,
    #         self._idx,
    #     )
    #     nano.update_cam2(data)

    def end_thread(self):
        self._on = False
        print('Camera 02 : thread close')



import cv2
from fcntl import ioctl
import mmap
import numpy as np
import os
import struct
import v4l2
import io
from PIL import Image
from logging import getLogger
l = getLogger(__name__)
import ctypes

NUM_BUFFERS = 10


class Camera(object):
    def __init__(self, device_name):
        self.controls = {}
        self.ctrl_id = {}
        self.ctrl_class = {}
        self.ctrl_val = {}

        self.device_name = device_name
        self.open_device()
        self.init_device()
        self._showInfo = True
        l.debug("open")
   
    def query_ctrl(self, qctrl_id):
        qctrl = v4l2.v4l2_queryctrl()
        qctrl.id = qctrl_id
        
        try:
            ioctl(self.fd, v4l2.VIDIOC_QUERYCTRL, qctrl)
            return qctrl
        except:
            l.error("QUERYCTRL failed")  
  
    def get_ctrl(self, ctrl_name):
        qctrl = self.query_ctrl(self.ctrl_id[ctrl_name])
        ext_ctrl = v4l2.v4l2_ext_control()
        ext_ctrl.id = self.ctrl_id[ctrl_name]

        ext_ctrls = v4l2.v4l2_ext_controls()
        ext_ctrls.controls.contents = ext_ctrl
        ext_ctrls.count = 1
        ext_ctrls.ctrl_class = self.ctrl_class[ctrl_name]
        
        try:
            ioctl(self.fd, v4l2.VIDIOC_G_EXT_CTRLS, ext_ctrls)
            self.controls[ctrl_name] = (ext_ctrl.id, ext_ctrl.value, qctrl.minimum, qctrl.maximum)
            if self._showInfo:
                 print("Get AVL Camera",ctrl_name,"value : ({}<=)".format(qctrl.minimum),ext_ctrl.value,"(<={})".format(qctrl.maximum))
        except:
            l.error("G_EXT_CTRL failed")    
 

    def set_ctrl(self, ctrl_name):
        qctrl = self.query_ctrl(self.ctrl_id[ctrl_name])
        ext_ctrl = v4l2.v4l2_ext_control()
        ext_ctrl.id = self.ctrl_id[ctrl_name]
        ext_ctrl.value = self.ctrl_val[ctrl_name]
        ext_ctrls = v4l2.v4l2_ext_controls()
        ext_ctrls.controls.contents = ext_ctrl
        ext_ctrls.count = 1
        ext_ctrls.ctrl_class = self.ctrl_class[ctrl_name]
        
        try:
            ioctl(self.fd, v4l2.VIDIOC_S_EXT_CTRLS, ext_ctrls)
            self.controls[ctrl_name] = (ext_ctrl.id, ext_ctrl.value, qctrl.minimum, qctrl.maximum)
            if self._showInfo:
                 print("Set AVL Camera",ctrl_name,"value to ({}<=)".format(qctrl.minimum),self.ctrl_val[ctrl_name],"(<={})".format(qctrl.maximum))
        except:
            l.error("S_EXT_CTRL failed") 

    def cam_controls(self):
        #Brightness
        self.ctrl_id["Brightness"] = v4l2.V4L2_CID_BRIGHTNESS
        self.ctrl_class["Brightness"] = v4l2.V4L2_CTRL_CLASS_USER 
        self.ctrl_val["Brightness"] = 1

        #Gamma
        self.ctrl_id["Gamma"] = v4l2.V4L2_CID_GAMMA
        self.ctrl_class["Gamma"] = v4l2.V4L2_CTRL_CLASS_USER 
        self.ctrl_val["Gamma"] = 110  

        #Saturation
        self.ctrl_id["Saturation"] = v4l2.V4L2_CID_SATURATION
        self.ctrl_class["Saturation"] = v4l2.V4L2_CTRL_CLASS_USER 
        self.ctrl_val["Saturation"] = 50

        #Hue
        self.ctrl_id["Hue"] = v4l2.V4L2_CID_HUE
        self.ctrl_class["Hue"] = v4l2.V4L2_CTRL_CLASS_USER 
        self.ctrl_val["Hue"] = -75 

        #Manuel Gain
        self.ctrl_id["Manuel Gain"] = v4l2.V4L2_CID_GAIN
        self.ctrl_class["Manuel Gain"] = v4l2.V4L2_CTRL_CLASS_USER 
        self.ctrl_val["Manuel Gain"] = 1500 

        #Manuel Exposure
        self.ctrl_id["Manuel Exposure"] = v4l2.V4L2_CID_EXPOSURE
        self.ctrl_class["Manuel Exposure"] = v4l2.V4L2_CTRL_CLASS_USER 
        self.ctrl_val["Manuel Exposure"] = 18523960 

        #Blue Balance
        self.ctrl_id["Blue Balance"] = v4l2.V4L2_CID_BLUE_BALANCE
        self.ctrl_class["Blue Balance"] = v4l2.V4L2_CTRL_CLASS_USER  
        self.ctrl_val["Blue Balance"] = 1100 

        # #Red Balance
        self.ctrl_id["Red Balance"] = v4l2.V4L2_CID_RED_BALANCE
        self.ctrl_class["Red Balance"] = v4l2.V4L2_CTRL_CLASS_USER 
        self.ctrl_val["Red Balance"] = 1100 

        #Auto White Balance
        self.ctrl_id["Auto White Balance"] = v4l2.V4L2_CID_AUTO_WHITE_BALANCE
        self.ctrl_class["Auto White Balance"] = v4l2.V4L2_CTRL_CLASS_USER 
        self.ctrl_val["Auto White Balance"] = 1  #[0 False Auto off] / [1 True Auto on]

        #Auto Gain
        self.ctrl_id["Auto Gain"] = v4l2.V4L2_CID_AUTOGAIN
        self.ctrl_class["Auto Gain"] = v4l2.V4L2_CTRL_CLASS_USER 
        self.ctrl_val["Auto Gain"] = 1  #[0 False Auto off] / [1 True Auto on] 

        #Absolute Exposure
        self.ctrl_id["Absolute Exposure"] = v4l2.V4L2_CID_EXPOSURE_ABSOLUTE
        self.ctrl_class["Absolute Exposure"] = v4l2.V4L2_CTRL_CLASS_CAMERA 
        self.ctrl_val["Absolute Exposure"] = 50  

        #Auto Exposure
        self.ctrl_id["Auto Exposure"] = v4l2.V4L2_CID_EXPOSURE_AUTO
        self.ctrl_class["Auto Exposure"] = v4l2.V4L2_CTRL_CLASS_CAMERA 
        self.ctrl_val["Auto Exposure"] = 1  #[1 False Auto off] / [0 True Auto on]


    def cam_get_controls(self): 
       # self.set_ctrl("Brightness")
       #self.set_ctrl("Gamma")   
       # self.set_ctrl("Saturation")
       #self.set_ctrl("Hue") 
       # self.set_ctrl("Manuel Gain")
       #self.set_ctrl("Manuel Exposure") 
       # self.set_ctrl("Blue Balance")
       #self.set_ctrl("Red Balance") 
        self.set_ctrl("Brightness")
        self.set_ctrl("Auto White Balance")
        #self.set_ctrl("Auto Gain")
        #self.set_ctrl("Absolute Exposure")
        self.set_ctrl("Auto Exposure")

    def cam_set_controls(self): 
        self.get_ctrl("Brightness")
        self.get_ctrl("Gamma")   
        self.get_ctrl("Saturation")
        self.get_ctrl("Hue") 
        self.get_ctrl("Manuel Gain")
        self.get_ctrl("Manuel Exposure") 
        self.get_ctrl("Blue Balance")
        self.get_ctrl("Red Balance") 
        self.get_ctrl("Brightness")
        self.get_ctrl("Auto White Balance")
        self.get_ctrl("Auto Gain")
        self.get_ctrl("Absolute Exposure")
        self.get_ctrl("Auto Exposure")

    def open_device(self):
        self.fd = os.open(self.device_name, os.O_RDWR, 0)

    def init_device(self):
        cap = v4l2.v4l2_capability()
        fmt = v4l2.v4l2_format()
              
        ioctl(self.fd, v4l2.VIDIOC_QUERYCAP, cap)
        
        if not (cap.capabilities & v4l2.V4L2_CAP_VIDEO_CAPTURE):
            raise Exception("{} is not a video capture device".format(self.device_name))
        
        fmt.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        ioctl(self.fd, v4l2.VIDIOC_G_FMT, fmt)

        fmtdesc = v4l2.v4l2_fmtdesc()
        fmtdesc.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        fmtdesc.index = 1
        ioctl(self.fd, v4l2.VIDIOC_ENUM_FMT, fmtdesc)

        self.init_mmap()
        self.cam_controls()
    
    def init_mmap(self):
        req = v4l2.v4l2_requestbuffers()
        
        req.count = NUM_BUFFERS
        req.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        req.memory = v4l2.V4L2_MEMORY_MMAP
        
        try:
            ioctl(self.fd, v4l2.VIDIOC_REQBUFS, req)
        except Exception:
            raise Exception("video buffer request failed")
        
        if req.count < 2:
            raise Exception("Insufficient buffer memory on {}".format(self.device_name))

        self.buffers = []
        for x in range(req.count):
            buf = v4l2.v4l2_buffer()
            buf.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
            buf.memory = v4l2.V4L2_MEMORY_MMAP
            buf.index = x
            
            ioctl(self.fd, v4l2.VIDIOC_QUERYBUF, buf)

            buf.buffer =  mmap.mmap(self.fd, buf.length, mmap.PROT_READ, mmap.MAP_SHARED, offset=buf.m.offset)
            self.buffers.append(buf)

    def start_capturing(self):
        for buf in self.buffers:
            ioctl(self.fd, v4l2.VIDIOC_QBUF, buf)
        video_type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        ioctl(self.fd, v4l2.VIDIOC_STREAMON, struct.pack('I', video_type))
        self.main_loop()

    def get_capability(self, vd):
        cp = v4l2.v4l2_capability()
        fcntl.ioctl(vd, v4l2.VIDIOC_QUERYCAP, cp)
        return cp

    def show_capability(self, cp):
        print("Driver:", cp.driver)
        print("Camera:", cp.card)
        print("video capture device?\t", bool(cp.capabilities & v4l2.V4L2_CAP_VIDEO_CAPTURE))
        print("Supports read() call?\t", bool(cp.capabilities &  v4l2.V4L2_CAP_READWRITE))
        print("Supports streaming?\t", bool(cp.capabilities & v4l2.V4L2_CAP_STREAMING))

    def show_format(self, vd):
        fmt = v4l2.v4l2_format()
        pix_format = v4l2.v4l2_pix_format()
        # print("pix_format.width:", pix_format.width)
        # print("pix_format.height:", pix_format.height)
        # print("pix_format.pixelformat:", pix_format.pixelformat)
    
    def process_image(self, buf):
        video_buffer = self.buffers[buf.index].buffer
        data = video_buffer.read(buf.bytesused)
        try:
            #x = np.fromstring(data, dtype=np.uint8)
            #z = Image.open(io.BytesIO(data))
            y = np.frombuffer(data, np.uint8)
            image = cv2.imdecode(y,  cv2.IMREAD_COLOR)
            print(image)
            #image = cv2.imdecode(np.fromstring(data, dtype=np.uint8),  cv2.IMREAD_COLOR)
            #cv2.imshow(self.device_name, image)
            #cv2.write(decoded)
            cv2.waitKey(1)
            video_buffer.seek(0)
        except Exception as e:
            # Not entirely sure what Exceptions I'm looking for here, potentially a bad read?
            print(e)


if __name__ == "__main__":
    cam = Camera("/dev/video0")
    cam.cam_get_controls()
    cam.cam_set_controls()


    # #set Auto Exposure
    # ctrl_name = "Auto Exposure"
    # ctrl_id = v4l2.V4L2_CID_EXPOSURE_AUTO
    # ctrl_class = v4l2.V4L2_CTRL_CLASS_CAMERA
    # #get Auto Exposure
    # cam.get_ctrl(ctrl_name, ctrl_id, ctrl_class)
    # #set Auto Exposure   [1 False Auto off] / [0 True Auto on]
    # ctrl_val = 0   
    # cam.set_ctrl(ctrl_name, ctrl_val, ctrl_id, ctrl_class)

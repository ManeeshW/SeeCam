#V4L2_CID_AUTO_WHITE_BALANCE

#VIDIOC_QUERYCTRL
#qctrl.id, qctrl.name, qctrl.flags, qctrl.minimum, qctrl.maximum, qctrl.step, qctrl.type, qctrl.default_value)

#VIDIOC_QUERYMENU
#id=%d,index=%d,name=%s,value=%d", qctrlmenu.id, qctrlmenu.index, qctrlmenu.name, qctrlmenu.value);

#VIDIOC_QUERY_EXT_CTRL
#qctrl_ext.id, qctrl_ext.name, qctrl_ext.flags, qctrl_ext.minimum, qctrl_ext.maximum, qctrl_ext.step, qctrl_ext.type, qctrl_ext.default_value, qctrl_ext.elem_size, qctrl_ext.elems, qctrl_ext.nr_of_dims, qctrl_ext.dims[0], qctrl_ext.dims[1], qctrl_ext.dims[2], qctrl_ext.dims[3]);qctrl_ext.id |= next_fl;

#ReadExtControl(value, V4L2_CID_AUTOGAIN, "ReadAutoGain", "V4L2_CID_AUTOGAIN", V4L2_CTRL_CLASS_USER);
#SetExtControl(value, V4L2_CID_AUTOGAIN, "SetAutoGain", "V4L2_CID_AUTOGAIN", V4L2_CTRL_CLASS_USER);
#SetExtControl(value, V4L2_CID_EXPOSURE_AUTO, "SetAutoExposure", "V4L2_CID_EXPOSURE_AUTO", V4L2_CTRL_CLASS_CAMERA)


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
        self.device_name = device_name
        self.open_device()
        self.init_device()
        l.debug("open")

        # qctrl = v4l2.v4l2_queryctrl()
        # ctrl = v4l2.v4l2_control()
        # ctrl.id = v4l2.V4L2_CID_BRIGHTNESS

        # ioctl(self.fd, v4l2.VIDIOC_G_CTRL, ctrl)
        # self.controls["brightness"] = (ctrl.id, ctrl.value, qctrl.minimum, qctrl.maximum)
        # print(self.controls)

        # ctrl.value = 7
        # ioctl(self.fd, v4l2.VIDIOC_S_CTRL, ctrl)
        # self.controls["brightness"] = (ctrl.id, ctrl.value, qctrl.minimum, qctrl.maximum)
        # print(self.controls)

        #queryctrl/g_ctrl
        qctrl = v4l2.v4l2_queryctrl()
        #brightness
        qctrl.id = v4l2.V4L2_CID_AUTO_WHITE_BALANCE #v4l2.V4L2_CID_EXPOSURE_AUTO # V4L2_CID_EXPOSURE_ABSOLUTE #V4L2_CID_AUTOGAIN
        ioctl(self.fd, v4l2.VIDIOC_QUERYCTRL, qctrl)

        ext_ctrl = v4l2.v4l2_ext_control()
        ext_ctrls = v4l2.v4l2_ext_controls()
        ext_ctrl.id = v4l2.V4L2_CID_AUTO_WHITE_BALANCE
        ext_ctrls.controls.contents = ext_ctrl
        ext_ctrls.count = 1
        ext_ctrls.ctrl_class = v4l2.V4L2_CTRL_CLASS_USER
        ioctl(self.fd, v4l2.VIDIOC_G_EXT_CTRLS, ext_ctrls)

        self.controls["AutoWB"] = (ext_ctrl.id, ext_ctrl.value, qctrl.minimum, qctrl.maximum)
        print(self.controls)

        val = 300 #1 Auto on
        
        ext_ctrl = v4l2.v4l2_ext_control()
        ext_ctrl.id = v4l2.V4L2_CID_AUTO_WHITE_BALANCE
        ext_ctrl.value = val

        ext_ctrls = v4l2.v4l2_ext_controls()

        ext_ctrls.controls.contents = ext_ctrl
        ext_ctrls.count = 1
        ext_ctrls.ctrl_class = v4l2.V4L2_CTRL_CLASS_USER
        
        ioctl(self.fd, v4l2.VIDIOC_S_EXT_CTRLS, ext_ctrls)
        self.controls["AutoWB"] = (ext_ctrl.id, ext_ctrl.value, qctrl.minimum, qctrl.maximum)
        print(self.controls)
        # extCtrl = v4l2.v4l2_ext_control()
        # extCtrls = v4l2.v4l2_ext_controls()
        
	

    def get_ctrl(self, ctrl_name):
        (idx, v, mn, mx) = self.controls[ctrl_name]
        ctrl = v4l2.v4l2_control()
        ctrl.id = idx
        try:
            fcntl.ioctl(self.vd, v4l2.VIDIOC_G_CTRL, ctrl)
            self.controls[ctrl_name] = (idx, ctrl.value, mn, mx)
            return ctrl.value
        except:
            l.error("G_CTRL failed")        
        
        
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
        # print("width:", fmt.fmt.pix.width)
        # print("height", fmt.fmt.pix.height)
        # print("pixelformat", fmt.fmt.pix.pixelformat)

        fmtdesc = v4l2.v4l2_fmtdesc()
        fmtdesc.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        fmtdesc.index = 1
        ioctl(self.fd, v4l2.VIDIOC_ENUM_FMT, fmtdesc)
        # print("fmtdesc.description:", fmtdesc.description)
        # print("fmtdesc.pixelformat:", fmtdesc.pixelformat)
        
        self.init_mmap()
    
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


    def main_loop(self):
        for x in range(1):
            print("grabbing frame {}".format(x))
            buf = self.buffers[x % NUM_BUFFERS]
            ioctl(self.fd, v4l2.VIDIOC_DQBUF, buf)
            self.process_image(buf)
            ioctl(self.fd, v4l2.VIDIOC_QBUF, buf)


if __name__ == "__main__":
    cam = Camera("/dev/video0")
    cam.start_capturing()

import numpy as np
import threading
from datetime import datetime

class Time:
    def __init__(self, t=0.0):
        self._t0 = t
        self.t = (datetime.now()- self._t0).total_seconds()

class Cam2Data:
    def __init__(self, t_stamp=0.1, t=0.0, freq=0, idx=0):
        self.t_stamp = t_stamp
        self.t = t
        self.freq = freq
        self.idx = idx

class Data:
    def __init__(self, on = True):
        self._lock = threading.Lock()
        self.on = on
        self.cam2 = Cam2Data()

    def update_cam2(self, data):
        with self._lock:
            self.cam2 = data

nano = Data()


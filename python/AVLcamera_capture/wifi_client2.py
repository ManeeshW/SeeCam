import numpy as np
import socket
import threading
import time

from datetime import datetime

import proto.time_pb2
from common_types import nano

HOST = '192.168.8.1'
PORT = 5000

#HOST = '192.168.10.4'
#PORT = 9999


class Client2(threading.Thread):
    def __init__(self, thread_id, freq=100):
        threading.Thread.__init__(self)
        self.thread_id = thread_id
        self._lock = threading.Lock()

        self._on = True
        self._system_on = True
        self._enabled = True

        self._desired_dt = 1.0 / freq
        self._freq = 0

        self._client_data = proto.time_pb2.Data()
        self._server_data = proto.time_pb2.Data()

        self._wifi_started = True

        print('WIFI: initialized')


    def run(self):
        print('WIFI: starting thread')

        freq = 0.0
        avg_number = 100

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((HOST, PORT))

            if self._wifi_started:
                data_received = s.recv(1024)
                self.parse_received_data(data_received)
                print('Jetson system time updated')
                print("seconds",nano.cam2.t_stamp_seconds)
                print("seconds",nano.cam2.t_stamp_nanos)

            t0 = nano.cam2.t_stamp #datetime.now()
            t_pre = datetime.now()

            while self._on:
                t = datetime.now()
                dt = (t - t_pre).total_seconds()
                if dt < self._desired_dt:
                    continue
                
                # dt_millis = (t - t0)
                # t_millis = int(dt_millis.seconds * 1.0e3
                #             + dt_millis.microseconds / 1.0e3)
                if self._wifi_started:
                    self._wifi_started = False
                else:
                    try:
                        data_received = s.recv(1024)
                        if not data_received:
                            break
                    except ConnectionResetError:
                        print('WIFI2: server closed')
                        break
                    except:
                        print('WIFI2: error reading receiving from server')
                #print(f"Received {data_received!r}")
                #data = b"Hello, pi"
                #s.sendall(data)
                self.parse_received_data(data_received)
                

                data_to_send = self.get_data_to_send()
                s.sendall(data_to_send)

            print('WIFI: sending turn off cammand to PI')
            for i in range(2):
                self._client_data.system_on = False
                data_to_send = self.get_data_to_send()

                try:
                    s.sendall(data_to_send)
                except BrokenPipeError:
                    break
                except:
                    print('WIFI: error reading sending data to server')
                time.sleep(0.1)
        self._on = False
        self._system_on = False
        print('WIFI: thread closed')
        

    
    def get_data_to_send(self):
        with self._lock:
            self._client_data.system_on = self._on
        
        self.update_data()
        return self._client_data.SerializeToString()

    
    def parse_received_data(self, data):
        self._server_data.ParseFromString(data)
        nano.on = self._server_data.system_on
        nano.cam2.t_stamp_seconds = self._server_data.t_stamp_seconds
        nano.cam2.t_stamp_nanos = self._server_data.t_stamp_nanos
        
        
    def is_system_on(self):
        return self._system_on

    def update_data(self):
        self._client_data.t_cam2 = nano.cam2.t
        self._client_data.freq_cam2 = nano.cam2.freq
        self._client_data.idx_cam2 = nano.cam2.idx
        

    def end_thread(self):
        self._client_data.system_on = False
        time.sleep(2)
        self._on = False

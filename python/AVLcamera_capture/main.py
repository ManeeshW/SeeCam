import threading
import time

from datetime import datetime
from common_types import nano
from wifi_client2 import Client2
from avlcamera import AvlCamera

CAM2_ON = True

def main():
    print('MAIN: Starting rover')

    t0 = datetime.now()
    
    thread_wifi = Client2(1)
    thread_cam2 = AvlCamera(2, t0, CAM2_ON)
    
    threads = []
    threads.append(thread_wifi)
    threads.append(thread_cam2)

    for thread in threads:
        thread.start()
    
    exit_flag = False

    while not exit_flag:
        try:
            time.sleep(0.01)

            if not thread_wifi.is_system_on(): #*************************************************
            #if not thread_wifi.is_system_on():
                exit_flag = True

        except KeyboardInterrupt:
            exit_flag = True
            print('\nMAIN: Stopping rover')

    
    print('MAIN: Sending close signal to threads')
    for thread in threads:
        if thread._enabled:
            thread.end_thread()
            thread.join()

    print('MAIN: Rover closed!')


if __name__ == '__main__':
    main()

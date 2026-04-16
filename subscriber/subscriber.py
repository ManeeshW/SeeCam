import zenoh
import base64
import json
import numpy as np
import cv2
import struct

class ZenohSubscriber:
    def __init__(self, data_buffer, config):
        self.data_buffer = data_buffer
        self.config = config
        self.session = None

    def start(self):
        """Initialize Zenoh session and subscriber."""
        try:
            zenoh_config = zenoh.Config()
            zenoh_config.insert_json5("connect/endpoints", f'["udp/0.0.0.0:{self.config.get("zenoh_port", 4556)}"]')
            zenoh_config.insert_json5("scouting/multicast/enabled", "true")
            self.session = zenoh.open(zenoh_config)
            print("[INFO] Zenoh session established, scouting for publisher")
            self.sub = self.session.declare_subscriber("seecam/image", self.callback)
        except Exception as e:
            print(f"[ERROR] Failed to establish Zenoh session: {e}")
            raise

    def callback(self, sample):
        """Process incoming Zenoh sample and add to buffer."""
        try:
            payload = sample.payload.to_bytes()

            # Unpack header: sequence (4 bytes int), timestamp (8 bytes float), image_len (4 bytes int)
            header_size = struct.calcsize("!IdI")
            sequence_number, timestamp, image_len = struct.unpack("!IdI", payload[:header_size])
            image_data = payload[header_size:header_size + image_len]

            nparr = np.frombuffer(image_data, np.uint8)
            image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if image is None:
                print("[ERROR] Failed to decode image")
                return
            self.data_buffer.add_frame(image, sequence_number)
            print(f"[INFO] Image #{sequence_number} added to buffer at timestamp {timestamp}")
        except Exception as e:
            print(f"[ERROR] Callback error: {e}")

    def stop(self):
        """Close Zenoh session."""
        if self.session:
            self.session.close()
            print("[INFO] Zenoh session closed")
import cv2
import zenoh
import time
import os
import socket
import psutil
import struct
import json

SHOW_FRAME = False  # Set True to visualize frame on Jetson

def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception as e:
        print(f"[ERROR] Could not get local IP: {e}")
        return "Unknown"

def encode_image(image, quality=50):
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
    success, buffer = cv2.imencode(".jpg", image, encode_param)
    if not success:
        print("[ERROR] Failed to encode image to JPEG")
        return None
    return buffer.tobytes()

def set_jetson_performance():
    try:
        os.system('sudo nvpmodel -m 0')
        os.system('sudo jetson_clocks')
        print("Jetson performance mode enabled.")
    except Exception as e:
        print(f"Could not set Jetson performance mode: {e}")

def init_camera(width=640, height=480):
    USE_GSTREAMER = True  # Enable for CSI camera lower latency
    cap = None

    if USE_GSTREAMER:
        gst_pipeline = (
            f"v4l2src device=/dev/video0 ! video/x-raw, width={width}, height={height}, framerate=30/1 "
            "! videoconvert ! appsink"
        )
        cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        if cap.isOpened():
            print("[INFO] Camera opened with GStreamer.")
            return cap
        else:
            print("[WARN] Failed to open camera with GStreamer. Trying default method...")

    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if cap.isOpened():
        print("[INFO] Camera opened with default OpenCV (V4L2 backend).")
    else:
        print("[ERROR] Could not open camera.")
    return cap

def main():
    os.environ["RUST_LOG"] = "debug"
    set_jetson_performance()

    # Load configuration
    try:
        with open("config.json", "r") as f:
            config = json.load(f)
        resolution = config.get("image_resolution", {"width": 640, "height": 480})
        width = resolution.get("width", 640)
        height = resolution.get("height", 480)
        compression_quality = config.get("compression_quality", 100)  # Lowered for speed
        zenoh_port = config.get("zenoh_port", 4556)
        print(f"[INFO] Loaded config: resolution {width}x{height}, compression quality {compression_quality}, zenoh port {zenoh_port}")
    except Exception as e:
        print(f"[ERROR] Failed to load config.json, using default resolution 640x480, compression quality 50, and port 4556: {e}")
        width, height, compression_quality, zenoh_port = 640, 480, 50, 4556

    cap = init_camera(width, height)
    if cap is None or not cap.isOpened():
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, 60)  # Increased to 30 FPS

    time.sleep(1.0)

    config = zenoh.Config()
    config.insert_json5("listen/endpoints", f'["udp/0.0.0.0:{zenoh_port}"]')
    config.insert_json5("scouting/multicast/enabled", "true")
    try:
        session = zenoh.open(config)
        print("[INFO] Zenoh session established")
    except Exception as e:
        print(f"[ERROR] Failed to establish Zenoh session: {e}")
        return

    publisher_ip = get_local_ip()
    print(f"[INFO] Publisher running on IP: {publisher_ip}:{zenoh_port}")

    pub = session.declare_publisher("seecam/image", priority=zenoh.Priority.DATA)

    sequence_number = 0
    process = psutil.Process()
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[ERROR] Failed to capture frame.")
                continue

            if frame.shape[1] != width or frame.shape[0] != height:
                frame = cv2.resize(frame, (width, height))
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                print(f"[INFO] Captured image size: {frame.shape[1]}x{frame.shape[0]}")

            if SHOW_FRAME:
                cv2.imshow("Captured Frame", frame)
                if cv2.getWindowProperty("Captured Frame", cv2.WND_PROP_VISIBLE) < 1:
                    print("Window closed. Exiting...")
                    break
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Pressed 'q'. Exiting...")
                    break

            encoded_image = encode_image(frame, compression_quality)
            if encoded_image is None:
                continue

            timestamp = time.time()

            # Pack header: sequence (4 bytes int), timestamp (8 bytes float), image_len (4 bytes int)
            header = struct.pack("!IdI", sequence_number, timestamp, len(encoded_image))
            message = header + encoded_image
            sequence_number += 1

            pub.put(message)
            print(f"[INFO] Published image #{sequence_number} at {timestamp}, Memory: {process.memory_info().rss / 1024**2:.2f} MB")

    except KeyboardInterrupt:
        print("Keyboard interrupt. Stopping...")
    finally:
        try:
            cap.release()
            cv2.destroyAllWindows()
            session.close()
        except Exception as e:
            print(f"[WARN] Cleanup issue: {e}")

if __name__ == "__main__":
    main()
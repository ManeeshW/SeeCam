import sys
import json
import signal
import time
import threading
from subscriber import ZenohSubscriber
from databuffer import DataBuffer
from image_saver import ImageSaver
import os

def signal_handler(sig, frame, app=None, saver=None):
    """Handle SIGINT (Ctrl+C) to gracefully exit."""
    print("Keyboard interrupt. Stopping...")
    if saver is not None:
        saver.stop_saving()  # Stop any running timers
    if app is not None:
        from PyQt6.QtWidgets import QApplication
        QApplication.quit()
    else:
        sys.exit(0)

def main():
    # Load configuration
    try:
        with open("config.json", "r") as f:
            config = json.load(f)
    except Exception as e:
        print(f"[ERROR] Failed to load config.json, using default settings: {e}")
        config = {"image_resolution": {"width": 640, "height": 480}}

    # Initialize data buffer
    data_buffer = DataBuffer(max_size=10)

    # Initialize subscriber
    subscriber = ZenohSubscriber(data_buffer, config)
    subscriber.start()

    # Check if Qt application is needed
    visualization_enabled = config.get("visualization_enabled", False)
    saving_enabled = config.get("image_saving_enabled", False)

    # Initialize image saver
    saver = ImageSaver(data_buffer, config)

    if not (visualization_enabled or saving_enabled):
        print("[INFO] Visualization and saving disabled. Running subscriber only. Press Ctrl+C to stop.")
        try:
            signal.signal(signal.SIGINT, lambda sig, frame: signal_handler(sig, frame, saver=saver))
            while True:
                time.sleep(0.1)  # Prevent busy loop
        except KeyboardInterrupt:
            print("Keyboard interrupt. Stopping...")
        finally:
            subscriber.stop()
            saver.close()
            return

    if visualization_enabled:
        # Import PyQt6 only if visualization is enabled
        from PyQt6 import QtWidgets, QtGui, QtCore
        from qtviz import ImageViewer

    # Initialize Qt application if visualization is enabled
    app = None
    if visualization_enabled:
        app = QtWidgets.QApplication(sys.argv)
        os.environ["RUST_LOG"] = "debug"

    # Set up signal handler for Ctrl+C
    signal.signal(signal.SIGINT, lambda sig, frame: signal_handler(sig, frame, app, saver))

    # Initialize image viewer if enabled
    viewer = None
    if visualization_enabled:
        viewer = ImageViewer(data_buffer, config)
        viewer.show()

    # Start periodic saving if enabled
    if saving_enabled:
        saver.start_saving()

    if visualization_enabled:
        try:
            sys.exit(app.exec())
        except KeyboardInterrupt:
            print("Keyboard interrupt. Stopping...")
        finally:
            subscriber.stop()
            saver.close()
    else:
        try:
            while True:
                time.sleep(0.1)  # Keep program running
        except KeyboardInterrupt:
            print("Keyboard interrupt. Stopping...")
        finally:
            subscriber.stop()
            saver.close()

if __name__ == "__main__":
    main()
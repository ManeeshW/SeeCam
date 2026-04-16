import h5py
import numpy as np
import threading
import os
from datetime import datetime

class ImageSaver:
    def __init__(self, data_buffer, config):
        self.data_buffer = data_buffer
        self.config = config
        self.hdf5_file = None
        self.frame_count = 0
        self.timer = None
        if self.config.get("image_saving_enabled", False):
            self.initialize_hdf5()

    def initialize_hdf5(self):
        """Initialize HDF5 file for storing images and metadata."""
        try:
            # Create directory if it doesn't exist
            hdf5_dir = self.config.get("hdf5_dir_path", "image_hdf5")
            os.makedirs(hdf5_dir, exist_ok=True)
            
            # Generate filename with timestamp and event name
            event_name = self.config.get("event_name", "default_event")
            timestamp = datetime.now().strftime("%Y.%m.%d-%H.%M.%S")
            file_name = f"{timestamp}_{event_name}.h5"
            file_path = os.path.join(hdf5_dir, file_name)
            
            self.hdf5_file = h5py.File(file_path, "a")  # Append mode
            print(f"[INFO] HDF5 file opened at {file_path}")
        except Exception as e:
            print(f"[ERROR] Failed to initialize HDF5 file: {e}")
            raise

    def is_saving_enabled(self):
        """Check if saving is enabled."""
        return self.config.get("image_saving_enabled", False) and self.hdf5_file is not None

    def save_frame(self):
        """Save the latest frame from the buffer to HDF5."""
        if not self.is_saving_enabled():
            return
        frame = self.data_buffer.get_latest_frame()
        if not frame:
            return
        try:
            group_name = f"frame_{self.frame_count:06d}"
            group = self.hdf5_file.create_group(group_name)
            group.create_dataset("image", data=frame["image"], compression="gzip", compression_opts=4)
            group.attrs["receive_time"] = frame["receive_time"]
            group.attrs["utc_time"] = frame["utc_time"]
            group.attrs["sequence_number"] = frame["sequence_number"]
            self.frame_count += 1
            print(f"[INFO] Saved frame #{self.frame_count} to HDF5")
        except Exception as e:
            print(f"[ERROR] Failed to save frame: {e}")

    def start_saving(self):
        """Start periodic saving of frames."""
        if not self.is_saving_enabled():
            return
        def save_frame_periodically():
            self.save_frame()
            if self.is_saving_enabled():
                self.timer = threading.Timer(0.1, save_frame_periodically)
                self.timer.daemon = True
                self.timer.start()

        self.timer = threading.Timer(0.1, save_frame_periodically)
        self.timer.daemon = True
        self.timer.start()

    def stop_saving(self):
        """Stop the saving timer."""
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
            print("[INFO] Saving timer stopped")

    def close(self):
        """Close the HDF5 file and stop the timer."""
        self.stop_saving()
        if self.hdf5_file:
            self.hdf5_file.close()
            print("[INFO] HDF5 file closed")
            self.hdf5_file = None
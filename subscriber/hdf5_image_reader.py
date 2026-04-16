import h5py
import numpy as np
import cv2
import os
import json

class ImageReader:
    def __init__(self, file_path):
        self.file_path = file_path
        self.hdf5_file = None

    def open(self):
        """Open the HDF5 file for reading."""
        try:
            self.hdf5_file = h5py.File(self.file_path, "r")
            print(f"[INFO] Opened HDF5 file for reading: {self.file_path}")
        except Exception as e:
            print(f"[ERROR] Failed to open HDF5 file: {e}")
            raise

    def get_frame(self, frame_id):
        """Retrieve a specific frame by ID."""
        try:
            group_name = f"frame_{frame_id:06d}"
            if group_name not in self.hdf5_file:
                print(f"[ERROR] Frame {frame_id} not found")
                return None
            group = self.hdf5_file[group_name]
            return {
                "image": np.array(group["image"]),
                "receive_time": group.attrs["receive_time"],
                "utc_time": group.attrs["utc_time"],
                "sequence_number": group.attrs["sequence_number"]
            }
        except Exception as e:
            print(f"[ERROR] Failed to read frame {frame_id}: {e}")
            return None

    def get_all_frame_ids(self):
        """Return a list of all frame IDs in the HDF5 file."""
        try:
            return [int(name.split("_")[1]) for name in self.hdf5_file.keys() if name.startswith("frame_")]
        except Exception as e:
            print(f"[ERROR] Failed to list frames: {e}")
            return []

    def extract_all_frames(self, output_dir="Data"):
        """Extract all frames and metadata, saving images to output_dir/images and metadata to output_dir/metadata.json."""
        try:
            # Create output directories
            images_dir = os.path.join(output_dir, "images")
            os.makedirs(images_dir, exist_ok=True)
            metadata_file = os.path.join(output_dir, "metadata.json")

            # Initialize metadata dictionary
            metadata = {}

            # Iterate through all frame IDs
            frame_ids = self.get_all_frame_ids()
            for frame_id in frame_ids:
                frame = self.get_frame(frame_id)
                if frame is None:
                    continue

                # Save image
                image_path = os.path.join(images_dir, f"frame_{frame_id:06d}.png")
                image_rgb = frame["image"]
                image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)  # Convert back to BGR for OpenCV
                cv2.imwrite(image_path, image_bgr)
                print(f"[INFO] Saved frame {frame_id} to {image_path}")

                # Store metadata, converting sequence_number to Python int
                metadata[f"frame_{frame_id:06d}"] = {
                    "receive_time": float(frame["receive_time"]),  # Ensure float for JSON
                    "utc_time": frame["utc_time"],
                    "sequence_number": int(frame["sequence_number"]),  # Convert int64 to int
                    "image_path": image_path
                }

            # Save metadata to JSON
            with open(metadata_file, "w") as f:
                json.dump(metadata, f, indent=4)
            print(f"[INFO] Saved metadata to {metadata_file}")

        except Exception as e:
            print(f"[ERROR] Failed to extract frames: {e}")

    def close(self):
        """Close the HDF5 file."""
        if self.hdf5_file:
            self.hdf5_file.close()
            print("[INFO] HDF5 file closed")
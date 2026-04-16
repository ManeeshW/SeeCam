import json
from hdf5_image_reader import ImageReader
import os

def main():
    # Load configuration
    try:
        with open("config.json", "r") as f:
            config = json.load(f)
    except Exception as e:
        print(f"[ERROR] Failed to load config.json: {e}")
        return

    # Get HDF5 directory path from config
    hdf5_dir = config.get("hdf5_dir_path", "image_hdf5")
    
    # Process all HDF5 files in the directory
    try:
        for file_name in os.listdir(hdf5_dir):
            if file_name.endswith(".h5"):
                hdf5_file_path = os.path.join(hdf5_dir, file_name)
                reader = ImageReader(hdf5_file_path)
                try:
                    reader.open()
                    # Create output directory based on HDF5 filename
                    base_name = os.path.splitext(file_name)[0]
                    output_dir = os.path.join(config.get("data_dir_path", "Data"), base_name)
                    reader.extract_all_frames(output_dir=output_dir)
                except Exception as e:
                    print(f"[ERROR] Failed to process HDF5 file {file_name}: {e}")
                finally:
                    reader.close()
    except Exception as e:
        print(f"[ERROR] Failed to access HDF5 directory: {e}")

if __name__ == "__main__":
    main()
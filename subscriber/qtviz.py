from PyQt6 import QtWidgets, QtGui, QtCore
import numpy as np
import cv2
import time
import psutil
import gc

class ImageViewer(QtWidgets.QWidget):
    updateImageSignal = QtCore.pyqtSignal(dict)

    def __init__(self, data_buffer, config):
        super().__init__()
        self.data_buffer = data_buffer
        self.config = config
        # Get resolution from config, default to 640x480 for initial setup
        resolution = self.config.get("image_resolution", {"width": 640, "height": 480})
        self.target_width = resolution.get("width", 640)
        self.target_height = resolution.get("height", 480)
        self.setWindowTitle("SeeCam Image Viewer")
        self.setMinimumSize(self.target_width + 20, self.target_height + 40)
        self.layout = QtWidgets.QVBoxLayout()
        self.image_label = QtWidgets.QLabel(self)
        self.image_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.image_label.setFixedSize(self.target_width, self.target_height)
        self.stats_label = QtWidgets.QLabel(self)
        self.stats_label.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.layout.addWidget(self.image_label)
        self.layout.addWidget(self.stats_label)
        self.setLayout(self.layout)
        self.last_receive_time = None
        self.update_count = 0
        self.process = psutil.Process()

        # Connect signal
        self.updateImageSignal.connect(self.update_image)

        # Timer to check for new frames
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.check_for_new_frame)
        self.timer.start(10)  # Check every 10ms

    def closeEvent(self, event):
        self.timer.stop()
        super().closeEvent(event)

    def check_for_new_frame(self):
        """Check for new frame in buffer and emit signal."""
        if not self.config.get("visualization_enabled", False):
            return
        frame = self.data_buffer.get_latest_frame()
        if frame:
            self.updateImageSignal.emit(frame)

    @QtCore.pyqtSlot(dict)
    def update_image(self, frame):
        try:
            image = frame["image"]
            receive_time = frame["receive_time"]
            sequence_number = frame["sequence_number"]

            # Get image dimensions
            height, width, channel = image.shape
            print(f"[INFO] Received image size: {width}x{height}")

            # Adjust QLabel and window to match image size
            self.image_label.setFixedSize(width, height)
            self.setMinimumSize(width + 20, height + 40)  # Add padding for window borders and stats

            # Display image
            image_copy = np.copy(image)
            bytes_per_line = 3 * image_copy.shape[1]
            qimage = QtGui.QImage(image_copy.data, image_copy.shape[1], image_copy.shape[0], bytes_per_line, QtGui.QImage.Format.Format_RGB888)
            pixmap = QtGui.QPixmap.fromImage(qimage)
            self.image_label.setPixmap(pixmap)

            # Calculate latency and frequency
            display_time = time.time()
            latency = (display_time - receive_time) * 1000
            frequency = 0.0
            if self.last_receive_time is not None:
                time_diff = receive_time - self.last_receive_time
                if time_diff > 0:
                    frequency = 1.0 / time_diff
            self.last_receive_time = receive_time

            # Update stats
            self.stats_label.setText(f"Frequency: {frequency:.2f} Hz | Latency: {latency:.2f} ms | Seq: {sequence_number}")
            print(f"[INFO] Image #{sequence_number} displayed, Display size: {pixmap.width()}x{pixmap.height()}")

            # Clean up
            del qimage
            del pixmap
            self.update_count += 1
            if self.update_count % 50 == 0:
                gc.collect()
                print(f"[INFO] Memory usage after {self.update_count} images: {self.process.memory_info().rss / 1024**2:.2f} MB")

        except Exception as e:
            print(f"[ERROR] Failed to display image: {e}")
import numpy as np
from collections import deque
import time
from datetime import datetime

class DataBuffer:
    def __init__(self, max_size=10):
        self.buffer = deque(maxlen=max_size)

    def add_frame(self, image, sequence_number):
        """Add an image frame with timestamp and UTC time."""
        receive_time = time.time()
        utc_time = datetime.utcnow().isoformat()
        self.buffer.append({
            "image": np.copy(image),  # Copy to prevent memory issues
            "receive_time": receive_time,
            "utc_time": utc_time,
            "sequence_number": sequence_number
        })

    def get_latest_frame(self):
        """Retrieve the most recent frame, or None if buffer is empty."""
        return self.buffer[-1] if self.buffer else None

    def get_all_frames(self):
        """Return all frames in the buffer."""
        return list(self.buffer)

    def clear(self):
        """Clear the buffer."""
        self.buffer.clear()
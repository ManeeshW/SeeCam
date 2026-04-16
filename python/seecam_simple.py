import cv2

# ================== SIMPLE SETTINGS ==================
DEVICE = 1                    # Camera device number (/dev/video1)
CROP_TOP_BOTTOM = 80          # pixels removed from top and bottom (same as C++)
ENABLE_CROP = True            # Set False to disable cropping
# ====================================================

print("Starting SeeCam Viewer...")


import cv2

# ================== SIMPLE SETTINGS ==================
DEVICE = 1                    # Camera device number (/dev/video1)
CROP_TOP_BOTTOM = 80          # pixels removed from top and bottom (same as C++)
ENABLE_CROP = True            # Set False to disable cropping
# ====================================================

print("Starting SeeCam Viewer...")

# Open camera with real/native resolution (same as before)
gst_pipeline = f"v4l2src device=/dev/video{DEVICE} ! video/x-raw ! videoconvert ! appsink"
cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("⚠️ GStreamer failed → trying normal OpenCV...")
    cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)

if not cap.isOpened():
    print("❌ Error: Could not open camera!")
    exit()

print("✅ Camera opened successfully")
ret, frame = cap.read()
if ret:
    orig_h, orig_w = frame.shape[:2]
    if ENABLE_CROP and frame.shape[0] >= (CROP_TOP_BOTTOM * 2):
        frame = frame[CROP_TOP_BOTTOM : -CROP_TOP_BOTTOM, :, :]
        crop_h, crop_w = frame.shape[:2]
        print(f"✂️  After crop: {crop_w} × {crop_h}")

print("GUI window forced to 640×480")
print("Press 'q' to quit")

if ENABLE_CROP and frame.shape[0] >= (CROP_TOP_BOTTOM * 2):
        frame = frame[CROP_TOP_BOTTOM : -CROP_TOP_BOTTOM, :, :]
        crop_h, crop_w = frame.shape[:2]
        print(f"âï¸  After crop: {crop_w} Ã {crop_h}")

print("GUI window forced to 640Ã480")
print("Press 'q' to quit")
ret, frame = cap.read()
if ret:
    orig_h, orig_w = frame.shape[:2]


while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Failed to grab frame")
        break

    # === Exact same crop as your C++ code ===
    if ENABLE_CROP and frame.shape[0] >= (CROP_TOP_BOTTOM * 2):
        frame = frame[CROP_TOP_BOTTOM : -CROP_TOP_BOTTOM, :, :]

    # === Draw center cross lines ===
    height, width = frame.shape[:2]
    center_x = width // 2
    center_y = height // 2
    cv2.line(frame, (center_x, 0), (center_x, height), (0, 255, 0), 2)   # vertical
    cv2.line(frame, (0, center_y), (width, center_y), (0, 255, 0), 2)   # horizontal

    # === FORCE display to exactly 640x480 (this makes the GUI window small) ===
    frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)

    cv2.imshow("SeeCam Viewer - 640x480 (Press Q to quit)", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
        
cap.release()
cv2.destroyAllWindows()

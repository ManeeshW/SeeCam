import cv2
import json
import os

# ================== SETTINGS ==================
DEVICE = 0
ORIGINAL_WIDTH = 1920
ORIGINAL_HEIGHT = 1200
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480

# Crop margins (pixels removed from each side)
left_crop = 320
right_crop = 320
top_crop = 180
bottom_crop = 180

CROP_STEP = 1                  # pixels changed per key press
MIN_CROP_MARGIN = 0             # can go down to zero
CONFIG_FILE = "seecam_config.json"
# =============================================

print("Starting SeeCam Viewer - Adjustable Individual Crop to 640x480")

# Load previous crop margins from config if it exists
if os.path.exists(CONFIG_FILE):
    try:
        with open(CONFIG_FILE, 'r') as f:
            config = json.load(f)
            left_crop = config.get("left_crop", 164)
            right_crop = config.get("right_crop", 74)
            top_crop = config.get("top_crop", 0)
            bottom_crop = config.get("bottom_crop", 0)
        print(f"Loaded crop margins - Left:{left_crop} Right:{right_crop} Top:{top_crop} Bottom:{bottom_crop}")
    except:
        print("Could not load config, using defaults")

# Try GStreamer pipeline first
gst_pipeline = f"v4l2src device=/dev/video{DEVICE} ! image/jpeg,width={ORIGINAL_WIDTH},height={ORIGINAL_HEIGHT},framerate=60/1 ! jpegdec ! videoconvert ! appsink"
cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("GStreamer failed -> trying normal OpenCV V4L2...")
    cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)

# Force high resolution and MJPG
cap.set(cv2.CAP_PROP_FRAME_WIDTH, ORIGINAL_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, ORIGINAL_HEIGHT)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(cv2.CAP_PROP_FPS, 60)

print(f"Camera set to {ORIGINAL_WIDTH}x{ORIGINAL_HEIGHT}")
if not cap.isOpened():
    print("Error: Could not open camera!")
    exit()

print("Camera opened successfully")
print("Controls:")
print("  A          : Increase left crop")
print("  SHIFT + A  : Decrease left crop")
print("  D          : Increase right crop")
print("  SHIFT + D  : Decrease right crop")
print("  W          : Increase top crop")
print("  SHIFT + W  : Decrease top crop")
print("  S          : Increase bottom crop")
print("  SHIFT + S  : Decrease bottom crop")
print("  Q          : Quit and save crop margins to config")
print("Crop info will be printed in this terminal window (outside the frame)")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Calculate current crop size
    crop_w = ORIGINAL_WIDTH - left_crop - right_crop
    crop_h = ORIGINAL_HEIGHT - top_crop - bottom_crop

    # Safety check
    crop_w = max(100, crop_w)
    crop_h = max(100, crop_h)

    # Crop the region
    x1 = left_crop
    y1 = top_crop
    cropped = frame[y1:y1 + crop_h, x1:x1 + crop_w]

    # Resize to exactly 640x480
    display_frame = cv2.resize(cropped, (DISPLAY_WIDTH, DISPLAY_HEIGHT), interpolation=cv2.INTER_LINEAR)

    # Draw center crosshair
    cv2.line(display_frame, (320, 0), (320, 480), (0, 255, 0), 2)
    cv2.line(display_frame, (0, 240), (640, 240), (0, 255, 0), 2)

    # === NO cv2.putText anymore - info is printed to terminal instead ===
    cv2.imshow("SeeCam Viewer - 640x480 (A/D/W/S + SHIFT=decrease | Q=save&quit)", display_frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        # Save crop margins to config
        config = {
            "original_width": ORIGINAL_WIDTH,
            "original_height": ORIGINAL_HEIGHT,
            "left_crop": int(left_crop),
            "right_crop": int(right_crop),
            "top_crop": int(top_crop),
            "bottom_crop": int(bottom_crop)
        }
        with open(CONFIG_FILE, "w") as f:
            json.dump(config, f, indent=2)
        print(f"Saved crop margins to {CONFIG_FILE}")
        print(f"  Left: {left_crop}, Right: {right_crop}, Top: {top_crop}, Bottom: {bottom_crop}")
        break

    # ====================== INDIVIDUAL CROP CONTROLS ======================
    # Normal key = increase, SHIFT + key = decrease

    changed = False

    if key == ord('a'):           # increase left
        left_crop += CROP_STEP
        changed = True
    elif key == ord('A'):         # SHIFT + a → decrease left
        left_crop = max(MIN_CROP_MARGIN, left_crop - CROP_STEP)
        changed = True

    elif key == ord('d'):         # increase right
        right_crop += CROP_STEP
        changed = True
    elif key == ord('D'):         # SHIFT + d → decrease right
        right_crop = max(MIN_CROP_MARGIN, right_crop - CROP_STEP)
        changed = True

    elif key == ord('w'):         # increase top
        top_crop += CROP_STEP
        changed = True
    elif key == ord('W'):         # SHIFT + w → decrease top
        top_crop = max(MIN_CROP_MARGIN, top_crop - CROP_STEP)
        changed = True

    elif key == ord('s'):         # increase bottom
        bottom_crop += CROP_STEP
        changed = True
    elif key == ord('S'):         # SHIFT + s → decrease bottom
        bottom_crop = max(MIN_CROP_MARGIN, bottom_crop - CROP_STEP)
        changed = True

    # ====================== SAFETY CLAMP ======================
    # Prevent crop area from becoming too small
    left_crop = max(MIN_CROP_MARGIN, min(left_crop, ORIGINAL_WIDTH - 100 - right_crop))
    right_crop = max(MIN_CROP_MARGIN, min(right_crop, ORIGINAL_WIDTH - 100 - left_crop))
    top_crop = max(MIN_CROP_MARGIN, min(top_crop, ORIGINAL_HEIGHT - 100 - bottom_crop))
    bottom_crop = max(MIN_CROP_MARGIN, min(bottom_crop, ORIGINAL_HEIGHT - 100 - top_crop))

    # ====================== PRINT CROP INFO TO TERMINAL ======================
    if changed:
        crop_w = ORIGINAL_WIDTH - left_crop - right_crop
        crop_h = ORIGINAL_HEIGHT - top_crop - bottom_crop
        crop_w = max(100, crop_w)
        crop_h = max(100, crop_h)
        print(f"Crop: {crop_w}x{crop_h} | L:{left_crop} R:{right_crop} T:{top_crop} B:{bottom_crop}")

cap.release()
cv2.destroyAllWindows()

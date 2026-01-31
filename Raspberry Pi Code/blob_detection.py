from picamera2 import Picamera2
import cv2
import numpy as np
import time


# =========================
# Settings
# =========================
WIDTH, HEIGHT = 320, 240
PRINT_HZ = 5
SHOW_DEBUG = True             # shows the ROI binary mask
from picamera2 import Picamera2
import cv2
import numpy as np
import time
import socket

HOST = "192.168.10.1"
PORT = 5000

#  ROI
ROI_W0, ROI_H0 = 140, 100      # start size
ROI_MIN_W, ROI_MIN_H = 60, 60
ROI_MAX_W, ROI_MAX_H = 140, HEIGHT
SHRINK_FACTOR = 0.92          # when found
GROW_FACTOR = 1.35            # when lost
LOST_REACQUIRE_N = 8          # after N misses -> full-frame reacquire

# Blob limits
MIN_AREA = 1          # adjust
# MAX_AREA will be ROI area dynamically

# Dynamic threshold (robust)
TOP_DROP_FRAC = 0.05          # drop top 2% brightest pixels when estimating background
K_MAD = 6.0                   # threshold = median + K_MAD * MAD
MAD_EPS = 2.0                 # avoid zero MAD

# Optional blur to reduce sensor noise
BLUR = True

# =========================
# Mouse click -> read exact pixel intensity
# =========================
clicked = {"x": None, "y": None}
def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked["x"], clicked["y"] = x, y

# =========================
# Helpers
# =========================
def largest_blob_from_binary(th_img, min_area, max_area):
    """
    Returns (x, y, w, h, area, cx, cy) in ROI coords for the largest component, else None.
    """
    n, labels, stats, cents = cv2.connectedComponentsWithStats(th_img, connectivity=8)
    if n <= 1:
        return None
    areas = stats[1:, cv2.CC_STAT_AREA]
    i = 1 + int(np.argmax(areas))
    x, y, w, h, area = stats[i]
    if area < min_area or area > max_area:
        return None
    cx, cy = cents[i]
    return (int(x), int(y), int(w), int(h), int(area), float(cx), float(cy))

def robust_threshold_from_roi(roi_u8, drop_top_frac=0.02, k_mad=8.0):
    """
    Dynamic threshold based on ROI background stats:
      1) flatten ROI
      2) drop brightest top X% (assumed possible laser pixels)
      3) TH = median(bg) + k * MAD(bg)
    Returns (TH, median, mad).
    
    K_MAD SHOULD BE MULTIPLE OF 1.482 
    
    """
    v = roi_u8.reshape(-1).astype(np.float32)
    if v.size < 50:
        med = float(np.median(v))
        mad = float(np.median(np.abs(v - med)))
        th = med + k_mad * max(mad, MAD_EPS)
        return th, med, mad

    v.sort()
    cut = int((1.0 - drop_top_frac) * v.size)
    cut = max(1, min(cut, v.size))
    bg = v[:cut]

    med = float(np.median(bg))
    mad = float(np.median(np.abs(bg - med)))
    th = med + k_mad * max(mad, MAD_EPS)
    return th, med, mad

# =========================
# Camera setup
# =========================
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"format": "BGR888", "size": (WIDTH, HEIGHT)}
)
picam2.configure(config)

target_fps = 120
frame_us = int(1_000_000 / target_fps)

picam2.set_controls({
    "AeEnable": False,
    "AwbEnable": False,

    # lock fps timing
    "FrameDurationLimits": (frame_us, frame_us),

    # lock exposure/gain
    "ExposureTime": 3000,      # must be <= frame_us
    "AnalogueGain": 5.0,

    # optional: lock WB gains (prevents color drift even with AWB off on some setups)
    # "ColourGains": (1.6, 1.6),
})

picam2.start()

cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)
cv2.setMouseCallback("Camera Feed", on_mouse)
if SHOW_DEBUG:
    cv2.namedWindow("ROI Binary (Dynamic TH)", cv2.WINDOW_NORMAL)

print("Press 'q' to quit. Click on the image to print that pixel's intensity.")

# =========================
# Dynamic ROI state
# =========================
roi_w, roi_h = ROI_W0, ROI_H0
roi_center = (140,100)             # (cx, cy) in full-frame coords
lost_count = 0

last_print = 0.0

try:
    while True:
        frame = picam2.capture_array()                 # BGR
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # 8-bit


        cx, cy = roi_center
        x0 = int(cx - roi_w // 2)
        y0 = int(cy - roi_h // 2)
        x0 = max(0, min(x0, WIDTH - roi_w))
        y0 = max(0, min(y0, HEIGHT - roi_h))
        x1 = min(WIDTH,  x0 + roi_w)
        y1 = min(HEIGHT, y0 + roi_h)
        roi = gray[y0:y1, x0:x1]
        
        if BLUR:
            roi_proc = cv2.GaussianBlur(roi, (5, 5), 0)
        else:
            roi_proc = roi

        # ---- Dynamic threshold from ROI stats 
        TH, bg_med, bg_mad = robust_threshold_from_roi(
            roi_proc, drop_top_frac=TOP_DROP_FRAC, k_mad=K_MAD
        )

        # ---- Binary mask: bright spot ----  (filtering part)
        th_img = (roi_proc.astype(np.float32) > TH).astype(np.uint8) * 255

        if SHOW_DEBUG:
            cv2.imshow("ROI Binary (Dynamic TH)", th_img)

        # ---- Find blob ----
        max_area = th_img.shape[0] * th_img.shape[1]
        blob = largest_blob_from_binary(th_img, MIN_AREA, max_area)

        # ---- Draw ROI box ----
        
        #cx2,cy2 = 139, 126
        
        cv2.rectangle(frame, (x0, y0), (x1 - 1, y1 - 1), (0, 255, 0), 1)
        cv2.rectangle(frame, (108, 107), (169, 167), (0, 255, 0), 1)


        # ---- Update ROI tracking ----
        if blob:
            # Blob centroid in full-frame coords
            bx, by, bw, bh, area, bc_x, bc_y = blob
            spot_x = x0 + bc_x
            spot_y = y0 + bc_y

            roi_center = (spot_x, spot_y)
          
            # shrink ROI (down to minimum)
            roi_w = int(max(ROI_MIN_W, roi_w * SHRINK_FACTOR))
            roi_h = int(max(ROI_MIN_H, roi_h * SHRINK_FACTOR))

            # Draw bbox + centroid
            cv2.rectangle(frame, (x0 + bx, y0 + by), (x0 + bx + bw, y0 + by + bh), (0, 0, 255), 1)
            cv2.circle(frame, (int(spot_x), int(spot_y)), 3, (0, 0, 255), -1)
            cv2.putText(frame, f"A={area}", (x0 + bx, y0 + max(0, by - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
            
        else:

            # expand ROI (up to max) while trying to reacquire
            roi_w = int(min(ROI_MAX_W, max(ROI_MIN_W, roi_w * GROW_FACTOR)))
            roi_h = int(min(ROI_MAX_H, max(ROI_MIN_H, roi_h * GROW_FACTOR)))


        cv2.imshow("Camera Feed", frame)

        # ---- Terminal status ----
        now = time.time()
        if now - last_print >= 1.0 / PRINT_HZ:
            roi_min, roi_max, roi_mean = int(roi.min()), int(roi.max()), float(roi.mean())
            if blob:
                _, _, _, _, area, bc_x, bc_y = blob
                print(f"ROI[{x0}:{x1},{y0}:{y1}] min/max/mean={roi_min}/{roi_max}/{roi_mean:.1f}  "
                      f"TH={TH:.1f} med={bg_med:.1f} mad={bg_mad:.1f}  "
                      f"blob area={area} roi_c=({bc_x:.1f},{bc_y:.1f}) full_c=({spot_x:.1f},{spot_y:.1f} lost={lost_count}")
                    
            last_print = now

        # ---- Click intensity ----
        if clicked["x"] is not None:
            x, y = clicked["x"], clicked["y"]
            val = int(gray[y, x])
            print(f"Clicked pixel -> (x={x}, y={y}) intensity={val}")
            clicked["x"] = clicked["y"] = None

        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            break

finally:
    cv2.destroyAllWindows()
    picam2.stop()

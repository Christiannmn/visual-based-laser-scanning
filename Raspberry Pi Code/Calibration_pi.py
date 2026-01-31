from picamera2 import Picamera2
import cv2
import numpy as np
import time
import socket

# =========================
# Settings
# =========================
WIDTH, HEIGHT = 320, 240
SHOW_DEBUG = True

HOST = "192.168.10.1"   # PC IP
PORT = 5000

MIN_AREA = 5
TOP_DROP_FRAC = 0.01
K_MAD = 6.0
MAD_EPS = 2.0
BLUR = True

# =========================
# Helpers
# =========================

def largest_blob_from_binary(th_img, min_area, max_area):
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
# Connect socket
# =========================
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setblocking(True)
sock.connect((HOST, PORT))
sock.sendall(b"Start\n")

# =========================
# Camera setup
# =========================
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"format": "BGR888", "size": (WIDTH, HEIGHT)}
)
picam2.configure(config)
picam2.start()

cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)
if SHOW_DEBUG:
    cv2.namedWindow("Binary Mask", cv2.WINDOW_NORMAL)

print("Calibration mode: FULL FRAME ONLY")

try:
    while True:
        frame = picam2.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ===== FULL FRAME ROI (FIXED) =====
        roi = gray  # <-- this guarantees full frame

        roi_proc = cv2.GaussianBlur(roi, (5, 5), 0) if BLUR else roi

        TH, bg_med, bg_mad = robust_threshold_from_roi(
            roi_proc, drop_top_frac=TOP_DROP_FRAC, k_mad=K_MAD
        )

        th_img = (roi_proc.astype(np.float32) > TH).astype(np.uint8) * 255
        if SHOW_DEBUG:
            cv2.imshow("Binary Mask", th_img)

        max_area = WIDTH * HEIGHT
        blob = largest_blob_from_binary(th_img, MIN_AREA, max_area)

        # Draw full-frame border
        cv2.rectangle(frame, (0, 0), (WIDTH - 1, HEIGHT - 1), (0, 255, 0), 1)

        if blob:
            bx, by, bw, bh, area, cx, cy = blob

            # FULL-FRAME COORDS
            spot_x = cx
            spot_y = cy

            cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (0, 0, 255), 1)
            cv2.circle(frame, (int(spot_x), int(spot_y)), 3, (0, 0, 255), -1)

            # Always send true full-frame centroid
            sock.sendall(f"{spot_x:.1f},{spot_y:.1f}\n".encode())

        cv2.imshow("Camera Feed", frame)

        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            break

finally:
    try:
        sock.close()
    except:
        pass
    cv2.destroyAllWindows()
    picam2.stop()

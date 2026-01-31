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
BLUR = True

# -------------------------
# Socket 
# -------------------------
HOST = "192.168.10.1"  # PC receiver IP
PORT = 5000


# -------------------------
# ROI 
# -------------------------
SEARCH_ROI = (107, 107,137,137)
TRACK_W, TRACK_H = 30, 30
LOST_REACQUIRE_N = 10            
BLOB_AREA = 43 # Measured blob area at z distance. If area is too low --> use 5%


# -------------------------
#Threshold  intensity params
# -------------------------
TOP_DROP_FRAC = BLOB_AREA / (TRACK_W * TRACK_H) # Should be ca. 1-5% 
K_MAD = 2.7
MAD_EPS = 1.4826


# -------------------------
# Peak params
# -------------------------
MIN_POS_PIXELS = 5   # Min size for intensity 
PEAK_FLOOR = 80 
P_MIN = 4.5
R_MIN = 0.12

TOPK_FRAC = BLOB_AREA / (TRACK_W * TRACK_H)


#If roi changed min distance
ROI_SEND_MIN_CHANGE = 0


def robust_threshold_from_roi(roi_u8, drop_top_frac=0.05, k_mad=8.0):
    
    
    #Drop top % of pixels --> We do not want to use that as background 
    v = roi_u8.reshape(-1).astype(np.float32)
    v.sort()
    cut = int((1.0 - drop_top_frac) * v.size)
    cut = max(1, min(cut, v.size))
    bg = v[:cut]

    #Median absolute deviation 
    med = float(np.median(bg))
    mad = float(np.median(np.abs(bg - med)))
    th = med + k_mad * max(mad, MAD_EPS)
    return th


def detect_PR_and_peak_xy(H):
    v = H[H > 0]
    #Threshold for size of measured pixels
    if v.size < MIN_POS_PIXELS:
        return False, 0.0, 0.0, 0.0, None

    #Find the highest intensity
    peak = float(v.max())
    #Find the mean intensity
    mean_pos = float(v.mean())
    #Find the total intensity
    total = float(v.sum())
    
    #How big is the highest peak vs rest
    P = peak / (mean_pos + 1e-6)

    #Find top brightest pixels
    K = max(1, int(TOPK_FRAC * v.size))
    kth = np.partition(v, -K)[-K]
    top_sum = float(v[v >= kth].sum())
    
    #How many pixels are the brightest pixels compared to background
    R = top_sum / (total + 1e-6)

    detected = (peak > PEAK_FLOOR) and (P > P_MIN) and (R > R_MIN)
    
    if not detected:
        return False, peak, P, R, None

    py, px = np.unravel_index(np.argmax(H), H.shape)
    return True, peak, P, R, (int(px), int(py))


def clamp_roi(x0, y0, x1, y1, W, H):
    if x1 < x0: x0, x1 = x1, x0
    if y1 < y0: y0, y1 = y1, y0

    x0 = max(0, min(int(x0), W - 1))
    y0 = max(0, min(int(y0), H - 1))
    x1 = max(1, min(int(x1), W))
    y1 = max(1, min(int(y1), H))

    if x1 <= x0: x1 = min(W, x0 + 1)
    if y1 <= y0: y1 = min(H, y0 + 1)
    return x0, y0, x1, y1


def roi_centered_on(px, py, w, h, W, H):
    half_w = w // 2
    half_h = h // 2
    x0 = px - half_w
    y0 = py - half_h
    x1 = x0 + w
    y1 = y0 + h
    return clamp_roi(x0, y0, x1, y1, W, H)



# Socket connect 
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock_ok = False
if s.connect_ex((HOST, PORT)) == 0:
    print("Connected to server.")
    sock_ok = True
    s.sendall(b"Start\n")
    time.sleep(0.5)
else:
    print("Failed to connect to server.")
    time.sleep(1)

if sock_ok:
    s.setblocking(False)

recv_buf = ""




def send_roi(roi):
    global sock_ok
    if not sock_ok:
        return
    try:
        x0, y0, x1, y1 = roi
        s.sendall(f"{x0},{y0},{x1},{y1}\n".encode())
    except OSError:
        sock_ok = False



def poll_done_messages():
    """
    Drains socket and returns the LAST DONE message parsed:
      returns (got_done: bool, ok: bool)
    """
    global recv_buf, sock_ok
    if not sock_ok:
        return False, False

    latest_done = None

    while True:
        try:
            data = s.recv(4096)
            if not data:
                sock_ok = False
                return False, False
            recv_buf += data.decode(errors="replace")
        except BlockingIOError:
            break
        except OSError:
            sock_ok = False
            return False, False

    if "\n" in recv_buf:
        lines = recv_buf.split("\n")
        recv_buf = lines[-1]
        for line in lines[:-1]:
            line = line.strip()
            if line.startswith("DONE"):
                latest_done = line
                print("Done received \n")

    if latest_done is None:
        return False, False

    parts = latest_done.split(",")
    ok = (len(parts) >= 2 and parts[1].strip() == "1")
    return True, ok



# =========================
# Camera Init
# =========================
picam2 = Picamera2()

config = picam2.create_preview_configuration(
    main={"format": "BGR888", "size": (WIDTH, HEIGHT)}
)



picam2.configure(config)


picam2.start()

#cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)

print("Pi: ROI-only accumulate until PC says DONE. Press 'q' to quit.")

# =========================
# State Init
# =========================
current_roi = SEARCH_ROI
last_sent_roi = None
# ROI accumulator 
H_roi = None
last_detected_xy_full = None
lost_scans = 0


# send start ROI
send_roi(current_roi)
last_sent_roi = current_roi

try:
    while True:
       
        frame = picam2.capture_array()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.rectangle(frame, (107, 107),(137,137), (0, 255, 0), 1)

        # Clamp ROI 
        x0, y0, x1, y1 = clamp_roi(*current_roi, WIDTH, HEIGHT)
        current_roi = (x0, y0, x1, y1)

        # Allocate/reset accumulator when ROI size changes
        h = y1 - y0
        w = x1 - x0
        if H_roi is None or H_roi.shape != (h, w):
            H_roi = np.zeros((h, w), dtype=np.float32)

        # ---- ROI crop  ----
        roi = gray[y0:y1, x0:x1]
        roi_proc = cv2.GaussianBlur(roi, (5, 5), 0) if BLUR else roi

        # Finding background intensity in ROI
        I_ref = robust_threshold_from_roi(roi_proc, TOP_DROP_FRAC, K_MAD)

        # Removing background intensity within ROI
        I_val = roi_proc.astype(np.float32) - float(I_ref)
        
         #Set negative values to 0.
        I_val[I_val < 0] = 0.0

        # Accumulate within ROI
        H_roi += I_val

        # Check for DONE
        got_done, ok = poll_done_messages()

        if got_done:
            detected, peak, P, R, peak_xy = detect_PR_and_peak_xy(H_roi)
            print(f"peak:{peak}, mean_peak:{P}, R: {R}")
            
            if ok and detected and peak_xy is not None:
                px_roi, py_roi = peak_xy

                # Convert to full-frame coords
                px = x0 + px_roi
                py = y0 + py_roi
                last_detected_xy_full = (px, py)

                
                print(f"{px},{py}")

                # Next ROI centered on detection (full-frame coords)
                current_roi = roi_centered_on(px, py, TRACK_W, TRACK_H, WIDTH, HEIGHT)
                lost_scans = 0
            else:
                last_detected_xy_full = None
                lost_scans += 1
                if lost_scans >= LOST_REACQUIRE_N:
                    current_roi = SEARCH_ROI
                    lost_scans = 0

            send_roi(current_roi)
            last_sent_roi = current_roi

            # Reset accumulator for next scan
            H_roi.fill(0.0)
        
        # ---- Visualization ----
        # Current commanded ROI (yellow)
        x0, y0, x1, y1 = current_roi
    
        cv2.rectangle(frame, (x0, y0), (x1 - 1, y1 - 1), (0, 255, 255), 1)


        # Last detected box (green) updates only on DONE
        if last_detected_xy_full is not None:
            px, py = last_detected_xy_full
            cv2.circle(frame, (px, py), 3, (0, 255, 0), -1)

        cv2.imshow("Camera Feed", frame)

        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            break

finally:
    cv2.destroyAllWindows()
    picam2.stop()
    try:
        s.close()
    except Exception:
        pass

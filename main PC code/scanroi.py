import numpy as np
import time
from galvo_controller import GalvoController
import pandas as pd
import socket


#This is the interface that interpolates coordinates based on the calibration data from "calibration_test_computer.py". This MUST 
#be used at the same time as the intensity_detection.py script on the Raspberry Pi to function. In this script, the gavlo
# is controlled to scan a square region of interest (ROI) --> (cx0, cy0, cx1, cy1) specified by the Raspberry Pi over Ethernet.


# -------------------------
# 1) Fit affine: (cx,cy) -> (gx,gy)
# -------------------------
def fit_cxcy_2_gxgy(df, roi=None):
    """
    Fits:
      gx = a0 + a1*cx + a2*cy
      gy = b0 + b1*cx + b2*cy

    roi: optional (cx0,cx1,cy0,cy1) to do a LOCAL affine fit for  SPEED.
    Returns a, b (each length-3 numpy arrays).
    """
    d = df
    if roi is not None:
        cx0, cx1, cy0, cy1 = roi
        cxmin, cxmax = sorted((cx0, cx1))
        cymin, cymax = sorted((cy0, cy1))
        # margin helps stability (adds extra points around edges)
        margin = 10
        d = d[(d["cx"] >= cxmin - margin) & (d["cx"] <= cxmax + margin) &
              (d["cy"] >= cymin - margin) & (d["cy"] <= cymax + margin)]
        if len(d) < 6:
            raise ValueError(f"Not enough calibration points in ROI for affine fit (got {len(d)}).")

    cx = d["cx"].to_numpy(dtype=float)
    cy = d["cy"].to_numpy(dtype=float)
    gx = d["gx"].to_numpy(dtype=float)
    gy = d["gy"].to_numpy(dtype=float)
    
    #Design matrix
    X = np.column_stack([np.ones_like(cx), cx, cy])  # [1, cx, cy]

    a, *_ = np.linalg.lstsq(X, gx, rcond=None)
    b, *_ = np.linalg.lstsq(X, gy, rcond=None)
    return a, b


# -------------------------
# 2) Prediction
# -------------------------
def affine_predict(a, b, cx, cy):
    cx = np.asarray(cx, dtype=float)
    cy = np.asarray(cy, dtype=float)
    #We fit the model to a first order affine
    gx = a[0] + a[1]*cx + a[2]*cy
    gy = b[0] + b[1]*cx + b[2]*cy
    return gx, gy


# -------------------------
# 3) Scan ROI in pixel space using affine
# -------------------------
def scan_roi_pixels_affine(
    galvo, a, b,
    cx0, cx1, cy0, cy1,
    *,
    step_cx=3,
    step_cy=3,
    delay_s=0.0,
    snake=True,
    clamp=True,
    print_status=False,
):
    cxmin, cxmax = sorted((cx0, cx1))
    cymin, cymax = sorted((cy0, cy1))

    #Galvo scan grid in pixel space 
    xs = np.arange(cxmin, cxmax + 1e-9, step_cx, dtype=float)
    ys = np.arange(cymin, cymax + 1e-9, step_cy, dtype=float)

    for i, cy in enumerate(ys):
        row_xs = xs if (not snake or i % 2 == 0) else xs[::-1]

        gx_row, gy_row = affine_predict(a, b, row_xs, cy)

        for cx, gx, gy in zip(row_xs, gx_row, gy_row):
            gx = float(gx); gy = float(gy)
            if clamp:
                gx = float(np.clip(gx, -1.0, 1.0))
                gy = float(np.clip(gy, -1.0, 1.0))

            if print_status:
                print(f"cx={cx:.1f} cy={cy:.1f} -> gx={gx:.6f} gy={gy:.6f}")
                
            galvo.set_position(gx, gy)

            if delay_s > 0:
                time.sleep(delay_s)
    return True



#Example use of scanning same window repeatedly 30 times...

#df = pd.read_csv("grid_capture2.csv")
#a, b = fit_affine_cxcy_to_gxgy(df, roi=ROI)

#n = 0
#while n < 30:
#    scan_roi_pixels_affine(
#    galvo, a, b,
#    cx0=108, cx1=169,
#    cy0=107, cy1=167,
#    step_cx=3,
#    step_cy=3,
#    delay_s=0.0001,    
#    snake=True,
#    print_status=False
#   )
#    n += 1
#    time.sleep(0.1)




HOST = "0.0.0.0"
PORT = 5000

galvo = GalvoController(device="Dev1/ao0:1", vmin=-9.0, vmax=9.0)
df = pd.read_csv("grid_capture2.csv")

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((HOST, PORT))
s.listen(1)

print("Listening on", PORT)
conn, addr = s.accept()
print("Connected from", addr)

conn.setblocking(False) 
buf = ""

last_roi = None
a = b = None

def parse_roi(line: str):
    parts = line.strip().split(",")
    if len(parts) != 4:
        raise ValueError("expected x0,y0,x1,y1")
    x0 = int(float(parts[0])); y0 = int(float(parts[1]))
    x1 = int(float(parts[2])); y1 = int(float(parts[3]))
    return x0, y0, x1, y1

def send_done(ok: bool, roi=None):
    """
    Send a DONE line back to the Pi.
    Format (simple text line):
      DONE,1\n  or  DONE,0\n
    Optionally include ROI for debugging:
      DONE,1,x0,y0,x1,y1\n
    """
    if roi is None:
        conn.sendall(f"DONE,{1 if ok else 0}\n".encode())
        #Repetitive?????
    else:
        x0, y0, x1, y1 = roi
        conn.sendall(f"DONE,{1 if ok else 0},{x0},{y0},{x1},{y1}\n".encode())
        print("DONE SENT")


try:
    while True:
        latest_line = None
        while True:
            try:
                data = conn.recv(4096)
                if not data:
                    raise ConnectionError("Client disconnected")
                buf += data.decode(errors="replace")
            except BlockingIOError:
                break  

        # ---- Keep only the last complete line ----
        if "\n" in buf:
            lines = buf.split("\n")
            buf = lines[-1]  
            for line in lines[:-1]:
                line = line.strip()
                if not line or line.lower().startswith("start"):
                    continue
                latest_line = line  

        if latest_line is None:
            continue

        # ---- Use latest ROI ----
        x0, y0, x1, y1 = parse_roi(latest_line)
        ROI = (x0, x1, y0, y1) 

        print(f"New ROI: x0={x0} y0={y0} x1={x1} y1={y1}")

        # only refit if ROI changed
        if ROI != last_roi:
            a, b = fit_cxcy_2_gxgy(df, roi=ROI)
            last_roi = ROI
       
        print("SCAN START")
        ok = scan_roi_pixels_affine(
            galvo, a, b,
            cx0=x0, cx1=x1,
            cy0=y0, cy1=y1,
            step_cx=5,
            step_cy=5,
            delay_s=0.001,
            snake=True,
            print_status=False
        )
        
        print("SCAN END ok=", ok)
        
        #  Tell Raspberry Pi that scan is finished
        send_done(ok, roi=(x0, y0, x1, y1))

except Exception as e:
    print("Stopped:", e)
finally:
    try:
        conn.close()
    except Exception:
        pass
    s.close()

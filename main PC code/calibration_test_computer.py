import socket
import time
import numpy as np
import pandas as pd
from galvo_controller import GalvoController


# This is the interface with the calibration (cx, cy) --> (gx,gy).
# This script MUST be run simulataneously with the calibration script on the Raspberry Pi.
# In this script, the galvo scans over a grid of points while the Raspberry Pi sends back
# the detected (cx,cy) coordinates over Ethernet (local IP adress). The collected data is saved to CSV.

HOST = "0.0.0.0"
PORT = 5000

GX_MIN, GX_MAX = -0.47, 0.47
GY_TOP, GY_BOTTOM = 0.7, -0.7

GX_STEP = 0.05
GY_STEP = 0.02

XS = np.arange(GX_MIN, GX_MAX + 1e-9, GX_STEP)
YS = np.arange(GY_TOP, GY_BOTTOM - 1e-9, -GY_STEP)

DWELL_S = 0.15

OUT_CSV = "grid_capture2.csv"

# ---- server ----
srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind((HOST, PORT))
srv.listen(1)

print("Listening on port", PORT)
conn, addr = srv.accept()
print("Connected from", addr)

buf = ""

# ---- wait for Start ----
print("Waiting for Start...")
while True:
    data = conn.recv(1024)
    if not data:
        raise RuntimeError("Disconnected before Start")
    buf += data.decode(errors="replace")
    if "Start\n" in buf:
        buf = buf.split("Start\n", 1)[1]  # drop everything before Start
        print("Starting grid scan...")
        break

# ---- galvo ----
galvo = GalvoController(device="Dev1/ao0:1", vmin=-9.0, vmax=9.0)
galvo.set_position(0.0, 0.20)
time.sleep(0.5)

rows = []

try:
    for gy in YS:
        for gx in XS:
            # move galvo to this grid point
            galvo.set_position(float(gx), float(gy))

            # collect coords during dwell, keep the latest x,y
            t_end = time.time() + DWELL_S
            last_cx = None
            last_cy = None

            conn.setblocking(False)
            while time.time() < t_end:
                try:
                    data = conn.recv(4096)
                    if data:
                        buf += data.decode(errors="replace")
                except BlockingIOError:
                    pass

                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()

                    if not line or line.lower().startswith("start") or "," not in line:
                        continue

                    try:
                        x_str, y_str = line.split(",", 1)
                        last_cx = float(x_str)
                        last_cy = float(y_str)
                    except ValueError:
                        pass

                time.sleep(0.01)

            conn.setblocking(True)

            if last_cx is not None:
                print(f"gx={gx:.3f} gy={gy:.3f} -> cx={last_cx:.1f} cy={last_cy:.1f}")
                rows.append({
                    "gx": float(gx),
                    "gy": float(gy),
                    "cx": float(last_cx),
                    "cy": float(last_cy),
                })
            else:
                print(f"gx={gx:.3f} gy={gy:.3f} -> NO COORDS FOUND")

finally:
    
    galvo.set_position(0.0, 0.0)
    galvo.close()
    conn.close()
    srv.close()

    df = pd.DataFrame(rows)
    df.to_csv(OUT_CSV, index=False)
    print(f"Saved {len(df)} rows to {OUT_CSV}")

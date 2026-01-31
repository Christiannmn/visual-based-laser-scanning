import time
import msvcrt
from galvo_controller import GalvoController

#This interface was used to test the galvo with joystick-like controls using the keyboard. This was used primarily for testing 
#camera feedback as a faster alternative to "test_galvo.py".

def clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, v))

def main():
    galvo = GalvoController(device="Dev1/ao0:1", vmin=-9.0, vmax=9.0)

    # ---- absolute command sent to galvo ----
    x_abs, y_abs = 0.0, 0.0

    # ---- user-defined center (offset) ----
    cx, cy = 0.0, 0.0

    # ---- position relative to center ----
    x_rel, y_rel = 0.0, 0.0

    step_x = 0.05
    step_y = 0.02

    send_period = 0.01
    last_send = 0.0

    def update_abs_from_rel():
        nonlocal x_abs, y_abs
        x_abs = clamp(cx + x_rel)
        y_abs = clamp(cy + y_rel)

    update_abs_from_rel()
    galvo.set_position(x_abs, y_abs)

    print("Arrow-key joystick control (dynamic center)")
    print("Controls:")
    print("  Arrows : move relative to center")
    print("  c      : set NEW center at current position (makes current = 0,0 rel)")
    print("  SPACE  : go to current center")
    print("  x      : reset center to absolute (0,0)")
    print("  q      : quit (goes to absolute 0,0)")
    print("  [ / ]  : decrease/increase step_x")
    print("  - / =  : decrease/increase step_y")
    print(f"Initial: step_x={step_x}, step_y={step_y}")
    print(f"Center: ({cx:.3f}, {cy:.3f})  Rel: ({x_rel:.3f}, {y_rel:.3f})  Abs: ({x_abs:.3f}, {y_abs:.3f})")

    try:
        while True:
            moved = False

            while msvcrt.kbhit():
                ch = msvcrt.getch()

                # Extended keys: arrows, etc.
                if ch in (b"\x00", b"\xe0"):
                    k = msvcrt.getch()

                    if k == b"H":      # Up
                        y_rel += step_y
                        moved = True
                    elif k == b"P":    # Down
                        y_rel -= step_y
                        moved = True
                    elif k == b"M":    # Left
                        x_rel -= step_x
                        moved = True
                    elif k == b"K":    # Right
                        x_rel += step_x
                        moved = True

                    # Clamp REL so ABS stays in [-1,1]
                    # (prevents center+rel exceeding limits)
                    x_rel = clamp(x_rel, -1.0 - cx, 1.0 - cx)
                    y_rel = clamp(y_rel, -1.0 - cy, 1.0 - cy)

                    update_abs_from_rel()
                    print(f"Center=({cx:.3f},{cy:.3f})  Rel=({x_rel:.3f},{y_rel:.3f})  Abs=({x_abs:.3f},{y_abs:.3f})")

                else:
                    key = ch.decode(errors="ignore")

                    if key == "q":
                        galvo.set_position(0.0, 0.0)
                        return

                    if key == " ":
                        # Go to current center => rel becomes 0,0
                        x_rel, y_rel = 0.0, 0.0
                        update_abs_from_rel()
                        moved = True
                        print(f"Go center -> Center=({cx:.3f},{cy:.3f})  Abs=({x_abs:.3f},{y_abs:.3f})")

                    if key == "c":
                        # Set NEW center at current absolute position:
                        # Make current abs become new center, and rel reset to 0,0
                        cx, cy = x_abs, y_abs
                        x_rel, y_rel = 0.0, 0.0
                        update_abs_from_rel()
                        moved = True
                        print(f"NEW center set -> Center=({cx:.3f},{cy:.3f})")

                    if key == "x":
                        # Reset center to absolute origin
                        cx, cy = 0.0, 0.0
                        x_rel, y_rel = 0.0, 0.0
                        update_abs_from_rel()
                        moved = True
                        print("Center reset -> Center=(0.000,0.000)")

                    # step adjust
                    if key == "[":
                        step_x = max(0.001, step_x * 0.8)
                        print(f"step_x={step_x:.4f}")
                    elif key == "]":
                        step_x = min(1.0, step_x * 1.25)
                        print(f"step_x={step_x:.4f}")
                    elif key == "-":
                        step_y = max(0.001, step_y * 0.8)
                        print(f"step_y={step_y:.4f}")
                    elif key == "=":
                        step_y = min(1.0, step_y * 1.25)
                        print(f"step_y={step_y:.4f}")

            now = time.time()
            if moved or (now - last_send) >= send_period:
                galvo.set_position(float(x_abs), float(y_abs))
                last_send = now

            time.sleep(0.001)

    finally:
        try:
            galvo.set_position(0.0, 0.0)
            time.sleep(0.05)
        except Exception:
            pass
        galvo.close()
        print("Closed galvo.")

if __name__ == "__main__":
    main()

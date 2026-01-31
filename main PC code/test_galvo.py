import time
from galvo_controller import GalvoController
import numpy as np
import msvcrt


# This is the interface with testing galvo for positions in different modes. This was primarily used for testing the scanning window
# to be used during calibration and other tests for camera feedback.

def main():
    galvo = GalvoController(device="Dev1/ao0:1", vmin=-10, vmax=10)
    galvo.set_position(0.0, 0.0)

    print("Keys: q=quit/center, d=go(0.1,0.2), s=slow scan, f=fast scan")

    try:
        while True:
            if msvcrt.kbhit():
                key = msvcrt.getch().decode(errors="ignore")

                # ---- quit ----
                if key == "q":
                    galvo.set_position(0.0, 0.0)
                    break

                # ---- go to fixed point ----
                if key == "d":
                    galvo.set_position(-0.04259590457034106,0.20284893699214873)
 
                # ---- slow scan ----
                if key == "s":
                    for x in np.arange(-0.47, 0.47, 0.1):
                        for y in np.arange(-0.7, 0.7, 0.1):
                            print(float(x), float(y))
                            galvo.set_position(float(x), float(y))

                        # dwell 1.5s
                        t0 = time.time()
                        while (time.time() - t0) < 1.5:
                            time.sleep(0.01)
                            if msvcrt.kbhit():
                                k = msvcrt.getch().decode(errors="ignore")
                                if k == "q":
                                    galvo.set_position(0.0, 0.0)
                                    raise KeyboardInterrupt
                                if k == "d":
                                    galvo.set_position(0.2, 0.2)

                # ---- fast scan ----
                if key == "f":
                    for x in np.arange(-0.45, 0.45, 0.01):
                        galvo.set_position(float(x), 0.2)

                        # dwell 0.05s
                        t0 = time.time()
                        while (time.time() - t0) < 0.05:
                            time.sleep(0.01)
                            
                            if msvcrt.kbhit():
                                k = msvcrt.getch().decode(errors="ignore")
                                if k == "q":
                                    galvo.set_position(0.0, 0.0)
                                    raise KeyboardInterrupt


            time.sleep(0.01)  

    except KeyboardInterrupt:
        print("Loop terminated.")

    finally:
        galvo.close()
        print("Closed galvo.")
        
if __name__ == "__main__":
    main()

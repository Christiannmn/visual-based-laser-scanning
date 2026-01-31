
# This is the interface with the Ni-DAQ-board. Written by Adam.

import nidaqmx

class GalvoController:
    def __init__(self, device="Dev1/ao0:1",
                 vmin=-10.0, vmax=10.0):
        self.device = device
        self.vmin, self.vmax = vmin, vmax
        self.task = nidaqmx.Task()
        self.task.ao_channels.add_ao_voltage_chan(
            self.device, min_val=vmin, max_val=vmax
        )
        self.task.start()  

    # Set normalized position in [-1,1]
    def set_position(self, x_norm, y_norm):
        """Map x,y ∈ [-1,1] to [vmin,vmax] and write."""
        span = self.vmax - self.vmin
        vx = (x_norm + 1)/2 * span + self.vmin
        vy = (y_norm + 1)/2 * span + self.vmin
        self.task.write([vx, vy], auto_start=False)

    def close(self):
        self.task.close()


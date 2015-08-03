import time
import sys
import signal
from math import pi, sin
from pi_blaster_servos import PiBlasterServos
from servo import Servo


if __name__ == "__main__":
    s1 = Servo(0.15, 0.055, 0.245)
    pbs = PiBlasterServos()
    pbs.add(s1, 23)

    pbs[0] = 0.0

    def on_terminate(signal, frame):
        pbs.turn_off()
        sys.exit()

    signal.signal(signal.SIGINT, on_terminate)

    i = 0
    fs = 20.

    while(1):
        pbs[0] = pi/2.0 * sin(2.*pi*1.*i/fs)
        i = i + 1

        time.sleep(1./fs)

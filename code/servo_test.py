import time
import sys
import signal
from math import pi, sin
from pi_blaster_servos import PiBlasterServos
from servo import Servo


if __name__ == "__main__":
    # SG92R Servo(0.15, 0.055, 0.245)
    s1 = Servo(0.15, 0.065, 0.225)  # HS-311
    s2 = Servo(0.15, 0.065, 0.225)
    s3 = Servo(0.15, 0.065, 0.225)
    pbs = PiBlasterServos()
    pbs.add(s1, 23)
    pbs.add(s2, 24)
    pbs.add(s3, 18)

    pbs[0] = 0.0
    pbs[1] = 0.0
    pbs[2] = 0.0

    def on_terminate(signal, frame):
        pbs.turn_off()
        sys.exit()

    signal.signal(signal.SIGINT, on_terminate)

    i = 0.
    fs = 100.

    while(1):
        pbs[0] = pi/2.0 * sin(2.*pi*0.5*i/fs)
        pbs[1] = pi/2.0 * sin(2.*pi*0.5*i/fs)
        pbs[2] = pi/2.0 * sin(2.*pi*0.5*i/fs)
        i = i + 1

        time.sleep(1./fs)

import time
import sys
import signal
from math import pi, sin
from pi_blaster_servos import PiBlasterServos
from servo import Servo


if __name__ == "__main__":
    s0 = Servo(0.15, 0.065, 0.225)  # HS-311
    s1 = Servo(0.15, 0.065, 0.225)
    s2 = Servo(0.15, 0.065, 0.225)
    s3 = Servo(0.15, 0.055, 0.245)  # SG92R
    pbs = PiBlasterServos()
    pbs.add(s0, 18)
    pbs.add(s1, 23)
    pbs.add(s2, 24)
    pbs.add(s3, 25)

    pbs[0] = 0.0
    pbs[1] = 0.0
    pbs[2] = 0.0
    pbs[3] = 0.0

    def on_terminate(signal, frame):
        pbs.turn_off()
        sys.exit()

    signal.signal(signal.SIGINT, on_terminate)

    i = 0.
    fs = 100.

    while(1):
        text = raw_input("> ")

        if "cycle" in text:
            i = 0
            rate = 1
            while(i < (fs / rate)):
                pbs[0] = 0.7 * pi/2.0 * sin(2.*pi*rate*i/fs)
                pbs[1] = 0.7 * pi/2.0 * sin(2.*pi*rate*i/fs)
                pbs[2] = 0.7 * pi/2.0 * sin(2.*pi*rate*i/fs)

                i = i + 1
                time.sleep(1./fs)
        elif "bounce" in text:
            i = 0
            rate = 2
            while(i < (fs / rate)):
                pbs[0] = 0.1 * pi/2.0 * sin(2.*pi*rate*i/fs)
                pbs[1] = 0.1 * pi/2.0 * sin(2.*pi*rate*i/fs)
                pbs[2] = 0.1 * pi/2.0 * sin(2.*pi*rate*i/fs)

                i = i + 1
                time.sleep(1./fs)
        elif "nod" in text:
            i = 0
            rate = 3
            while(i < (fs / rate * 2)):
                pbs[0] = 0.1 * pi/2.0 * sin(2.*pi*rate*i/fs)
                pbs[1] = -0.1 * pi/2.0 * sin(2.*pi*rate*i/fs)
                pbs[2] = -0.1 * pi/2.0 * sin(2.*pi*rate*i/fs)

                i = i + 1
                time.sleep(1./fs)

        elif "shake" in text:
            i = 0
            rate = 3
            while(i < (fs / rate * 2)):
                pbs[3] = 0.2 * pi/2.0 * sin(2.*pi*rate*i/fs)
                i = i + 1
                time.sleep(1./fs)
        elif "tilt" in text:
            i = 0
            rate = 2
            while(i < (fs / rate * 2)):
                pbs[0] = 0
                pbs[1] = 0.2 * pi/2.0 * sin(2.*pi*rate*i/fs)
                pbs[2] = -0.2 * pi/2.0 * sin(2.*pi*rate*i/fs)

                i = i + 1
                time.sleep(1./fs)
        elif "lol" in text:
            i = 0
            rate = 1
            while(i < (fs / rate * 4)):
                pbs[0] = 0.2 * pi/2.0 * sin(2.*pi*rate*i/fs)
                pbs[1] = 0.2 * pi/2.0 * sin(2.*pi*rate*i/fs + pi / 3.)
                pbs[2] = 0.2 * pi/2.0 * sin(2.*pi*rate*i/fs + 2. * pi / 3.)
                pbs[3] = -0.4 * pi/2.0 * sin(2.*pi*rate*i/fs)

                i = i + 1
                time.sleep(1./fs)

            pbs[0] = 0
            pbs[1] = 0
            pbs[2] = 0
        elif "reset" in text:
            pbs[0] = 0
            pbs[1] = 0
            pbs[2] = 0
        elif "sleep" in text:
            pbs[0] = -90 * pi / 180.
            pbs[1] = -90 * pi / 180.
            pbs[2] = -90 * pi / 180.
        else:
            commands = text.split(",")
            for command in commands:
                tokens = command.split("=")

                if len(tokens) == 2:
                    motor = int(tokens[0])
                    angle = float(tokens[1]) * pi / 180.0
                    pbs[motor] = angle

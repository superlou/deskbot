import cProfile as profile
from servo_fast import ServoFast as Servo
from pi_blaster_servos import PiBlasterServos
import pyximport
pyximport.install()
from pi_blaster_servos_fast import PiBlasterServosFast

def set(pbs):
    pbs[0] = 0
    pbs[1] = 0
    pbs[2] = 0
    pbs[3] = 0

if __name__ == "__main__":
    s0 = Servo(0.15, 0.065, 0.225)  # HS-311
    s1 = Servo(0.15, 0.065, 0.225)
    s2 = Servo(0.15, 0.065, 0.225)
    s3 = Servo(0.15, 0.245, 0.055)  # SG92R

    pbs = PiBlasterServos()
    pbs.add(s0, 18)
    pbs.add(s1, 23)
    pbs.add(s2, 24)
    pbs.add(s3, 25)

    profile.run("for i in range(10): set(pbs)")
    pbs.close()

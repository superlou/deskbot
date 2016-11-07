import cProfile as profile
from servo import Servo
import pyximport
pyximport.install()
from servo_fast import ServoFast


if __name__ == "__main__":
    s0 = Servo(0.15, 0.065, 0.225)
    profile.run("for i in range(100): s0.period_for_angle(0.123)")
    print(s0.period_for_angle(-1.))
    print(s0.period_for_angle(1.))

    s1 = ServoFast(0.15, 0.065, 0.225)
    profile.run("for i in range(100): s1.period_for_angle(0.123)")
    print(s1.period_for_angle(-1.))
    print(s1.period_for_angle(1.))

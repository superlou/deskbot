import threading
import time
from numpy.random import gamma
from Adafruit_LED_Backpack import Matrix8x8


class Eye(object):
    def __init__(self):
        self.frames = [
            [60, 126, 231, 195, 195, 231, 126, 60],
            [0, 0, 126, 255, 195, 231, 126, 60],
            [0, 0, 0, 195, 255, 231, 126, 60],
            [0, 0, 0, 0, 195, 231, 126, 60]
        ]

        self.display = Matrix8x8.Matrix8x8()
        self.display.begin()
        self.draw_frame(0)

        t = threading.Thread(target=self.task)
        t.daemon = True
        t.start()

    def task(self):
        while 1:
            delay = gamma(20, 0.25)
            time.sleep(delay)
            self.blink()

    def blink(self):
        self.draw_frame(1)
        time.sleep(0.01)
        self.draw_frame(2)
        time.sleep(0.01)
        self.draw_frame(3)
        time.sleep(0.01)
        self.draw_frame(2)
        time.sleep(0.01)
        self.draw_frame(1)
        time.sleep(0.01)
        self.draw_frame(0)

    def draw_frame(self, index):
        self.display.clear()

        for y, row in enumerate(self.frames[index]):
            for x in range(8):
                if (row & (2**x)):
                    self.display.set_pixel(x, y, 1)

        self.display.write_display()

    def turn_off(self):
        self.display.clear()
        self.display.write_display()

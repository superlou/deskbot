import os


class PiBlasterServos(object):
    def __init__(self, device="/dev/pi-blaster"):
        self.servos = []
        self.angles = []
        self.pins = []

        self.device = open(device, 'w', 0)

    def add(self, servo, pin):
        self.servos.append(servo)
        self.angles.append(None)
        self.pins.append(pin)

    def __getitem__(self, key):
        return self.angles[key]

    def __setitem__(self, key, value):
        self.angles[key] = value
        period = self.servos[key].period_for_angle(value)

        command = "{0}={1}\n".format(self.pins[key], period)
        self.device.write(command)

    def turn_off(self):
        pass

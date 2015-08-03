# deskbot

The boring desktop robot.

# Installation

Install pi-blaster.

    sudo apt-get install python-smbus i2c-tools
    sudo raspi-config

In the config menu, advanced options, enable automatic loading of the I2C kernel module.  Still [had to add](https://forums.adafruit.com/viewtopic.php?f=47&t=74517&sid=16e9d784a04c41fe58b5d3ec6f3fdb2d) the following to /etc/modules:

    i2c-bcm2708
    i2c-dev

May need to do some of [manual configuration steps](https://learn.adafruit.com/adafruits-raspberry-pi-lesson-4-gpio-setup/configuring-i2c).

Test with `sudo i2cdetect -y 0`.

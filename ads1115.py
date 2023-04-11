from machine import I2C



class ADS1115:
    def __init__(self, i2c, addr):
        self._i2c = i2c
        self._addr = addr
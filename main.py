from machine import Pin, I2C
from ads1115 import ADS1115
import ads1115
import time

ads = ADS1115(I2C(0, scl=Pin(17), sda=Pin(16), freq=100000), 0x48)

ads.comparator_latch_ack()
#print(f'{var:016b}')
# ads.conversion_mode(ads1115.SINGLE_CONV)
# ads.amplifier_range(ads1115.FSR_4V)
# ads.multiplexer_config(ads1115.MUX_AIN0_GND)
# ads.trig_single_conv()
# time.sleep(1.0)
# print(ads.conversion_read())
# print(ads.conversion_read_raw())
# 
# raw = ads._read_register(0x00)
# print(f'{raw:016b}')
import time
import board
from math import atan2, pi
from typing import Tuple
import adafruit_lsm303dlh_mag

# DataSheet: https://cdn-shop.adafruit.com/datasheets/LSM303DLHC.PDF
# Convert MicroTesla to Compass Head: https://learn.adafruit.com/lsm303-accelerometer-slash-compass-breakout/coding

STRAIGHT_DEGREE = 0
RIGHT_ANGLE_DEGREE = 90
HALF_CIRCLE_DEGREE = RIGHT_ANGLE_DEGREE * 2
FULL_CIRCLE_DEGREE = HALF_CIRCLE_DEGREE * 2

# A tilt-compensated electronic compass (eCompass)
class MagnetometerDriver():
    def __init__(self):
        self.i2c = board.I2C()  # uses board.SCL and board.SDA
        self.compass_sensor = adafruit_lsm303dlh_mag.LSM303DLH_Mag(self.i2c)

    # returns 0-360 deg
    def get_compass_reading(self) -> float:
        x,y,_ = self.compass_sensor.magnetic
        degrees = (atan2(y,x) * HALF_CIRCLE_DEGREE) / pi
        if degrees < 0:
            degrees += FULL_CIRCLE_DEGREE
        return degrees

def main():
    mag = MagnetometerDriver()
    while True:
        print('Compass Heading (degrees): ' + str(mag.get_compass_reading()))
        print('')
        time.sleep(1.0)

if __name__ == '__main__':
    main()
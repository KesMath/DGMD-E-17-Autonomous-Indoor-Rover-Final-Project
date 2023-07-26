import time
import board
import math
from typing import Tuple
import adafruit_lsm303dlh_mag

# DataSheet: https://cdn-shop.adafruit.com/datasheets/LSM303DLHC.PDF
# Convert MicroTesla to Compass Head: https://learn.adafruit.com/lsm303-accelerometer-slash-compass-breakout/coding

STRAIGHT_DEGREE = 0
RIGHT_ANGLE_DEGREE = 90

# A tilt-compensated electronic compass (eCompass)
class MagnetometerDriver():
    def __init__(self):
        self.i2c = board.I2C()  # uses board.SCL and board.SDA
        self.compass_sensor = adafruit_lsm303dlh_mag.LSM303DLH_Mag(self.i2c)

    # returns three-axis magnetic field as x,y,z vectors
    def poll_sensor(self) -> Tuple[float, float, float]:
        return self.compass_sensor.magnetic

    # returns 0-360 deg
    def get_compass_reading(self) -> float:
        x,y,_ = self.poll_sensor()
        degrees = math.atan(x / y) * 180 / math.pi
        if degrees < 0:
            degrees += RIGHT_ANGLE_DEGREE * 4
        
        return degrees

def main():
    mag = MagnetometerDriver()
    while True:
        print('Magnetometer (gauss): ' + str(mag.get_compass_reading()))
        print('')
        time.sleep(1.0)

if __name__ == '__main__':
    main()
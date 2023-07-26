import time
import board
from typing import Tuple
import adafruit_lsm303dlh_mag

# DataSheet: https://cdn-shop.adafruit.com/datasheets/LSM303DLHC.PDF

STRAIGHT_DEGREE = 0
RIGHT_ANGLE_DEGREE = 90

# A tilt-compensated electronic compass (eCompass)
class MagnetometerDriver():
    def __init__(self):
        self.i2c = board.I2C()  # uses board.SCL and board.SDA
        self.compass_sensor = adafruit_lsm303dlh_mag.LSM303DLH_Mag(self.i2c)

    # returns three-axis magnetic field as x,y,z vectors
    def poll_sensor(self) -> Tuple[float, float, float]:
        return (self.compass_sensor.magnetic)

def main():
    mag = MagnetometerDriver()
    while True:
        print('Magnetometer (gauss):' + str(mag.poll_sensor))
        print('')
        time.sleep(1.0)

if __name__ == '__main__':
    main()
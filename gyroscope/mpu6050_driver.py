import time
import board
import numpy as np
import adafruit_mpu6050

# PURCHASE MAGNOMETER!!

# Tutorial: https://learn.adafruit.com/lsm303-accelerometer-slash-compass-breakout/python-circuitpython
# Lib: https://github.com/adafruit/Adafruit_CircuitPython_LSM303DLH_Mag

# Calculate Compass Heading (i.e. Magnometer Vector to Degrees)
# https://learn.adafruit.com/lsm303-accelerometer-slash-compass-breakout/coding


STRAIGHT_DEGREE = 0

class GyroscopeDriver():
    def __init__(self):
        self.i2c = board.I2C()  # uses board.SCL and board.SDA
        self.mpu = adafruit_mpu6050.MPU6050(self.i2c)

    def __poll_sensor(self, index: int):
        return (self.mpu.gyro[index])

    # orientation in X-axis
    def read_roll(self):
        return np.rad2deg(self.__poll_sensor(0))
    
    # orientation in Y-Axis
    def read_pitch(self):
        return np.rad2deg(self.__poll_sensor(1))

    # orientation in Z-Axis
    def read_yaw(self):
        return np.rad2deg(self.__poll_sensor(2))


def main():
    g_driver = GyroscopeDriver()
    tolerance = 0.5 # use exernal compass to determine what sensor reading is at 90deg orientation to determine acceptable tolerance factor 
    while True:
        yaw = g_driver.read_yaw()
        if (STRAIGHT_DEGREE <= yaw and yaw <= tolerance): 
            print("Z-Axis: " + str(yaw))
            print(" ")
            time.sleep(0.1)
        else:    
            print("Z-Axis Spin Out of Tolerance: " + str(yaw))
            time.sleep(0.1)

if __name__ == '__main__':
    main()
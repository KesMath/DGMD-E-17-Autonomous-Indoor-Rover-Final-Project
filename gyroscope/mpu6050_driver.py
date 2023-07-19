import time
import board
import numpy as np
import adafruit_mpu6050

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
        if (STRAIGHT_DEGREE >= g_driver.read_yaw() <= tolerance): 
            print("Z-Axis Spin In of Tolerance: " + str(g_driver.read_yaw()))
            print(" ")
            time.sleep(1)
        print("Z-Axis Spin Out of Tolerance: " + str(g_driver.read_yaw()))

if __name__ == '__main__':
    main()

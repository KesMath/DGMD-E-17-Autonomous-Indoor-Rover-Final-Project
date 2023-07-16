import time
import board
import numpy as np
import adafruit_mpu6050

TURN_DEGREE = 90
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
    def read_yaw(self):
        return np.rad2deg(self.__poll_sensor(1))

    # orientation in Z-Axis
    def read_pitch(self):
        return np.rad2deg(self.__poll_sensor(2))


def main():
    # g_driver = GyroscopeDriver()
    # while True:
    #     print("X: " + str(g_driver.read_roll()))
    #     print("Y: " + str(g_driver.read_yaw()))
    #     print("Z: " + str(g_driver.read_pitch()))
    #     print(" ")
    #     time.sleep(1)
    
    i2c = board.I2C()  # uses board.SCL and board.SDA
    mpu = adafruit_mpu6050.MPU6050(i2c)

    while True:
        print("Gyro X:%.2f, Y: %.2f, Z: %.2f deg/s" % np.rad2deg(mpu.gyro[0]), np.rad2deg(mpu.gyro[1]), np.rad2deg(mpu.gyro[2]))
        print("")
        time.sleep(1)

if __name__ == '__main__':
    main()

import time
import board
import numpy as np
import adafruit_mpu6050

STRAIGHT_DEGREE = 0
RIGHT_THRESHOLD = 89.99999999999999
LEFT_THRESHOLD = -89.99999999999999

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
    while True:
        yaw = g_driver.read_yaw()
        if (yaw > RIGHT_THRESHOLD): 
            print("Right turn in proximity of " + str(RIGHT_THRESHOLD) + ": "  + str(yaw) + "\n")
            time.sleep(0.01)

        elif (yaw < LEFT_THRESHOLD): 
            print("Left turn in proximity of " + str(LEFT_THRESHOLD) + ": "  + str(yaw) + "\n")
            time.sleep(0.01)

if __name__ == '__main__':
    main()
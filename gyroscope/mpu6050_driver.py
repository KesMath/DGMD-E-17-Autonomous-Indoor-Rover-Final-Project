import time
import board
import numpy as np
import adafruit_mpu6050

STRAIGHT_DEGREE = 0
THRESHOLDING_VALUE = (-89, 89)

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


    def is_sensor_exceeding_right_threshold(self):
        while True:
            yaw = self.read_yaw()
            if (yaw > THRESHOLDING_VALUE[1]): 
                print("Right turn in proximity of " + str(THRESHOLDING_VALUE[1]) + ": "  + str(yaw) + "\n")
                time.sleep(0.01)
                break

    def is_sensor_exceeding_left_threshold(self):
        while True:
            yaw = self.read_yaw()
            if (yaw < THRESHOLDING_VALUE[0]): 
                print("Left turn in proximity of " + str(THRESHOLDING_VALUE[0]) + ": "  + str(yaw) + "\n")
                time.sleep(0.01)
                break

def main():
    g_driver = GyroscopeDriver()
    g_driver.is_sensor_exceeding_left_threshold()
    g_driver.is_sensor_exceeding_right_threshold()

if __name__ == '__main__':
    main()
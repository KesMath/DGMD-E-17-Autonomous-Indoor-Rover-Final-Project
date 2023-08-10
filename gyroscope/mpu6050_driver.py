import time
import board
import numpy as np
import adafruit_mpu6050

THRESHOLDING_VALUE = (-89, 89)
SLEEP_DELAY = 1.00e-02

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


    # determine if orientation is -90deg
    def is_sensor_orthogonally_left(self):
        while True:
            yaw = self.read_yaw()
            if (yaw < THRESHOLDING_VALUE[0]): 
                print("Orthogonally-Left turn in proximity of " + str(THRESHOLDING_VALUE[0]) + ": "  + str(yaw) + "\n")
                time.sleep(SLEEP_DELAY)
                break

    # determine if orientation is +90deg
    def is_sensor_orthogonally_right(self):
        while True:
            yaw = self.read_yaw()
            if (yaw > THRESHOLDING_VALUE[1]): 
                print("Orthogonally-Right in proximity of " + str(THRESHOLDING_VALUE[1]) + ": "  + str(yaw) + "\n")
                time.sleep(SLEEP_DELAY)
                break

def main():
    g_driver = GyroscopeDriver()
    g_driver.is_sensor_orthogonally_left()
    g_driver.is_sensor_orthogonally_right()

if __name__ == '__main__':
    main()
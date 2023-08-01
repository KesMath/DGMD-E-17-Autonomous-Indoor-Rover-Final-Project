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
    threshold = 89 
    while True:
        yaw = g_driver.read_yaw()
        if (yaw > threshold): 
            print("Greater than " + str(threshold) + ": "  + str(yaw) + "\n")
            time.sleep(0.01)
        #else:  
            #print("Degree: " + str(yaw))

if __name__ == '__main__':
    main()
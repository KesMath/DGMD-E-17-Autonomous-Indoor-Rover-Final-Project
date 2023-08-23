import board
import asyncio
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

    # determine if sensor's orientation is -90deg
    async def poll_sensor_until_90_clockwise(self, roverBase):
        print("polling sensor...")
        while True:
            yaw = self.read_yaw()
            if (yaw < THRESHOLDING_VALUE[0]): 
                print("Clockwise 90: " + str(THRESHOLDING_VALUE[0]) + ": "  + str(yaw) + "\n")
                print("stopping rover...")
                await roverBase.stop()
                print("rover stopped!")
                return True
            else:
                print("YAW:" + str(yaw))
    
    # determine if sensor's orientation is -90deg
    def poll_sensor_until_90_clockwise2(self, roverBase):
        print("polling sensor...")
        while True:
            yaw = self.read_yaw()
            if (yaw < THRESHOLDING_VALUE[0]): 
                print("Clockwise 90: " + str(THRESHOLDING_VALUE[0]) + ": "  + str(yaw) + "\n")
                return True
            else:
                print("YAW:" + str(yaw))

    # determine if sensor's orientation is +90deg
    async def poll_sensor_until_90_counter_clockwise(self, roverBase):
        while True:
            yaw = self.read_yaw()
            if (yaw > THRESHOLDING_VALUE[1]): 
                print("Counter-Clockwise 90: " + str(THRESHOLDING_VALUE[1]) + ": "  + str(yaw) + "\n")
                print("stopping rover...")
                await roverBase.stop()
                print("rover stopped!")
                return True
            else:
                print("YAW:" + str(yaw))
    
    # trick to call async function using multiporcessing
    # https://stackoverflow.com/questions/71678575/how-do-i-call-an-async-function-in-a-new-process-using-multiprocessing
    def poll_for_90_clockwise(self, roverBase):
        asyncio.run(self.poll_sensor_until_90_clockwise(roverBase))

    def poll_for_90_counter_clockwise(self, roverBase):
        asyncio.run(self.poll_sensor_until_90_counter_clockwise(roverBase))
        
def main():
    g_driver = GyroscopeDriver()
    g_driver.poll_sensor_until_orthogonally_left()

if __name__ == '__main__':
    main()
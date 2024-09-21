from mpu6050 import mpu6050
import time
from utils import printProgressBar

class Rotation:
    # first u need to calibrate the readings before use
    
    # mpu6050's i2c address 
    _mpu = mpu6050(0x68)

    def __init__(self):

        
        self._stop = False # for stop the program

        self._ratecalibrationYaw = 0 
        self._angleYaw = 0
        # self.calibration()
        pass

    def calibration(self, num_of_points = 1000):
        for num in range(num_of_points):
            progress = (num + 1) / num_of_points * 100
            printProgressBar(progress)
            
            # braking logic
            if self._stop:
                break
            
            # take data from the sensor
            gyro_data = self._mpu.get_gyro_data()
            self._ratecalibrationYaw += round(gyro_data['z'])
        
        # calculate the average value
        self._ratecalibrationYaw /= num_of_points
        print(f'The Calibration value of yaw is : {self._ratecalibrationYaw}')
        self.calibrated = True

    def calculate_rotation(self, average = 5):
        while True:
            # breaking logic
            if self._stop:
                break

            # inital values
            rateYaw = 0
            start = time.time()
            
            for _ in range(average):
                gyro_data = self._mpu.get_gyro_data()
                rateYaw += round(gyro_data['z'])

            # calculation 
            dt = time.time() - start 
            rateYaw /= average
            rateYaw -= self._ratecalibrationYaw
            self._angleYaw += rateYaw * dt

            # print(f'algle is : {round(self._angleYaw)}')
        pass

    def reset(self):
        self._angleYaw = 0
        print("rotation reseted...")

    def stop_game(self):
        self._stop = True

if __name__ == '__main__':
    rot= Rotation()
    rot.calibration()
    try:
        while True:
            rot.calculate_rotation()

    except:
        print('rotation calculation stopped...')
        pass
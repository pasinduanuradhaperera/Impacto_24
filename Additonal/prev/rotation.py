from mpu6050 import mpu6050
import time
from utils import printProgressBar

class Accelerations:
    mpu = mpu6050(0x68)
    def __init__(self):
        
        self.true = True
        
        self.RatecalibrationYaw = 0
        self.AngleYaw = 0

        # run calibration process 
        self.calibration()

    def calibration(self,points = 1000):
        for num in range(points):
            precent = (num + 1)/points * 100            
            printProgressBar(precent)        

            # Angle    
            gyro_data = self.mpu.get_gyro_data()
            self.RatecalibrationYaw += round(gyro_data['z'])   

        self.RatecalibrationYaw /= points
        self.RatecalibrationYaw = self.RatecalibrationYaw
        print(f"the calibration values of Yaw: {self.RatecalibrationYaw}")

    def calculate_rotation(self,average = 5, true = True):
        while true:
            RateYaw = 0
            start = time.time()
            for _ in range(average):                                
                gyro_data = self.mpu.get_gyro_data()
                RateYaw += round(gyro_data['z'])
                #time.sleep(0.001)

            dt = time.time() -start            
            RateYaw /= average   
            RateYaw -= self.RatecalibrationYaw
            self.AngleYaw += RateYaw * dt 
            
            print("_________________________________")    
            print(f"Angle is : {round(self.AngleYaw, 2)}")
            print("_________________________________")  

    def reset(self):
        self.AngleYaw = 0
        print('all Reseted....')
    
    

    def mark(self,value):
        if value >= 0:
            mark = '+'
        else:
            mark = ''
        return mark
    
if __name__ == '__main__':
    acc = Accelerations()
    acc.calculate_rotation()

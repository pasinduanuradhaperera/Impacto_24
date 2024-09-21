import threading
from time import time, sleep
from detection import Detection 
from rotation import Accelerations
from motor_run import MotorRun  


class Game:
    def __init__(self, rotation: Accelerations):
        
        self.rot = rotation
        self.mr = MotorRun(rotation)
        self.detect = Detection()

        self.done = 0
        # wait to start 
        sleep(0.5)        
        
        print('started!')
        self.start()

    def det(self):
        marker_thread = threading.Thread(target=self.detect.detect)
        marker_thread.daemon = True
        marker_thread.start()

    def start(self):    
        # movement starts 
        self.det()
        sleep(2)
        while True:
            sleep(1)
            self.start_motor()
            print(self.detect.data)
            if self.done == 1:                                                    
                    break            
        print('Game Done!')
        


    def start_motor(self): 
            
        # vehical moving method
        def move(id):
            resetdata()
            print("hehe")
            sleep(1)
            self.align()
            resetdata()
            print("hehe")
            sleep(1)
            print('aligned')
            self.mr.forward_m(self.detect.data[1])
            print("on the top of the marker")
            self.mr.rotation(id, self.detect.data[2])
            print("take the dirrection")
            if id != 0:
                self.mr.forward_m(2)
            
            
        # data resetting 
        def resetdata():
            self.detect.data[0] = 0
            self.detect.data[1] = 0
            self.detect.data[2] = 0
            self.detect.data[3] = -99
        
    # values
        marker = self.detect.data[3] # marker id
        self.rot.AngleYaw = 0

    # 0 - 8 markder need to do some tasks 
        if marker in range(0, 9):     

            if marker == 0:
                self.done = 1
                            
            print(self.detect.data)
            move(marker)
            resetdata()
            print(self.detect.data)
            
            
    def align(self):
        i = 0 
        self.mr.pA.ChangeDutyCycle(40)
        self.mr.pB.ChangeDutyCycle(40)
        while True:
            
            sleep(0.5)
            if i < 20:
                print("i value is : ", i)
                if abs(self.detect.data[0]) > 2:
                    if self.detect.data[0] > 0 :
                        self.mr.right()
                    else:
                        self.mr.left()
                else:
                    break 
            else:
                break
            i += 1

                    


if __name__ == '__main__':
    a = Accelerations()
    game = Game(a)

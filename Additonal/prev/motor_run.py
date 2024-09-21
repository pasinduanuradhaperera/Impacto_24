import RPi.GPIO as GPIO
import threading
from time import sleep
from rotation import Accelerations
from distance import Distance

class MotorRun:

    def __init__(self,rotaions: Accelerations):
        
        self.rotations = rotaions
        self.distance = Distance()
        # motor A
        self.in1 = 23
        self.in2 = 24
        self.enA = 25

        # motor B
        self.in3 = 22
        self.in4 =16
        self.enB = 17
        
        angle_thread = threading.Thread(target = self.rotations.calculate_rotation)
        angle_thread.daemon = True
        angle_thread.start()

        distance_thread = threading.Thread(target = self.distance.calculate_distance)
        distance_thread.daemon = True
        distance_thread.start()
        sleep(0.2)

        GPIO.setmode(GPIO.BCM)

        # motor A pin setting
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.enA, GPIO.OUT)

        # motor B pin setting
        GPIO.setup(self.in3, GPIO.OUT)
        GPIO.setup(self.in4, GPIO.OUT)
        GPIO.setup(self.enB, GPIO.OUT)

        # set all the pins to LOW 
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        GPIO.output(self.in3, GPIO.LOW)
        GPIO.output(self.in4, GPIO.LOW)

        # set frequency 
        self.pA = GPIO.PWM(self.enA, 1000)
        self.pB = GPIO.PWM(self.enB, 1000)

        # starts in 65% load 
        self.pA.start(40)
        self.pB.start(40)
    

    def rotation(self, id, rot):

        def rotate(angle):
                
            if angle > 0:
                    while True:
                            self.left()
                            sleep(0.01)
                            if self.rotations.AngleYaw > angle:
                                self.right(0.05,0.004)
                                sleep(0.01)
                                print(f'angle is (1){self.rotations.AngleYaw}')
                                break
                            elif self.rotations.AngleYaw == angle:
                                self.stop()
                                print(f'angle is (1){self.rotations.AngleYaw}')
                                break
            elif angle < 0 :
                    while True:
                            self.right()
                            sleep(0.01)
                            if self.rotations.AngleYaw < angle:
                                self.left(0.05,0.004)
                                sleep(0.01)
                                print(f'angle is (2){self.rotations.AngleYaw}')
                                break
                            elif self.rotations.AngleYaw == angle:
                                self.stop()
                                print(f'angle is (2){self.rotations.AngleYaw}')
                                break
            else:
                    pass
                
        def calculate_turn_angle(current_rotation, marker_id):

            marker_angles = {
                1: 90,         # East
                2: 135,        # South-East
                3: 180,        # South
                4: -135,       # South-West
                5: -90,        # West
                6: -45,        # North-West
                7: 0,          # North
                8: 45          # North-East
            }

            marker_angle = marker_angles.get(marker_id, None)
            marker_angle -= current_rotation

            marker_angle = -((marker_angle + 180) % 360 - 180)

            if marker_angle > 0:
                return marker_angle
            elif marker_angle < 0:
                return marker_angle
            else:
                return 0
             
        if id != 0:
            angle = calculate_turn_angle(rot, id)
            print(f'the angle is {angle}')
        else: 
                angle = calculate_turn_angle(rot, 7)
                print(f'the angle is {angle}')
                
        self.rotations.AngleYaw = 0       
        
        rotate(angle)


    def forward_m(self, dis): 
            i = 0
            while True:  
                    sleep(0.1) 
                    self.pA.ChangeDutyCycle(40)
                    self.pB.ChangeDutyCycle(40)            
                    distance = self.distance.distance # traveled distance
                    if distance < dis:
                        distance = self.distance.distance
                        if (dis - distance) > 0.5 and i < 3:
                            print("distance before 1 is :", distance)
                            self.forward()
                            self.stop()
                            print("distance after 1 is :",distance)                          
                        else:                            
                             self.stop()
                             print('Stop by me 1')
                             sleep(0.1)
                             print("distance ",distance)
                             break
                    elif distance > dis:
                        
                        sleep(0.1)
                        distance = self.distance.distance
                        if (distance - dis) > 0.5 and i < 3:
                            print("i value is :", i)
                            print("distance before 2 is :", distance)
                            self.backward()
                            self.stop()
                            print("distance after 2 is :", distance)
                        else:
                            self.stop()
                            print('Stop by me 2')
                            sleep(0.1)
                            print(distance)
                            break
                    else:
                        self.stop()
                        print('Stop by me 3')
                        sleep(0.1)
                        print(distance)
                        break
                    i += 1 
            self.distance.total_rotation = 0

    #GPIO access
    def f_s(self):
        # Motor A
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)

        # Motor B
        GPIO.output(self.in3, GPIO.HIGH)
        GPIO.output(self.in4, GPIO.LOW)

    def b_s(self):

        print("backward")

        # Motor A
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.HIGH)

        # Motor B
        GPIO.output(self.in3, GPIO.LOW)
        GPIO.output(self.in4, GPIO.HIGH)

    def stop(self):

        print("stop")

        # Motor A
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)

        # Motor B
        GPIO.output(self.in3, GPIO.LOW)
        GPIO.output(self.in4, GPIO.LOW)

    def l_s(self):
        # Motor A
        GPIO.output(self.in1, GPIO.HIGH)
        GPIO.output(self.in2, GPIO.LOW)

        # Motor B
        GPIO.output(self.in3, GPIO.LOW)
        GPIO.output(self.in4, GPIO.HIGH)

    def r_s(self):
        # Motor A
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.HIGH)

        # Motor B
        GPIO.output(self.in3, GPIO.HIGH)
        GPIO.output(self.in4, GPIO.LOW)

    # MOTOR CONTROLS
    def forward(self, f = 0.17, r = 0.008):
        print("forward")
        self.f_s()
        sleep(f)
        self.stop()        
        sleep(r)
        self.b_s()

    def backward(self, f = 0.17, r = 0.008):
        print("backward")
        self.b_s()
        sleep(f)
        self.stop()
        sleep(r)        
        self.f_s()

    def right(self, f = 0.05, r = 0.006):
        print("right")
        self.r_s()
        sleep(f)
        self.l_s()
        sleep(r)
        self.stop()

    def left(self, f = 0.05, r = 0.006):
        print("left")
        self.l_s()
        sleep(f)
        self.r_s()
        sleep(r)
        self.stop()

if __name__ == '__main__':

    a = Accelerations()
    motor = MotorRun(a)
    sleep(1)
    # motor.rotation(1, 130)
    

    # for i in range(1):
    #     motor.left()
    
    #for i in range(100):        
        #motor.right()
    
    motor.forward_m(40)

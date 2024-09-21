import RPi.GPIO as GPIO
import threading
import time 

from encoder import Encoder
from rotation import Rotation
from detection import Detection

# motor 1, encoder 1 -> near buckconverter
# motor 2, encoder 2 -> near Other side

class Motor:
    def __init__(self, detection: Detection):
        self.detection = detection # make a common detection object for the Motor calss and Game class

        # Set up GPIO numbering mode
        GPIO.setmode(GPIO.BCM)        

        # --- physical connections ---
        # Define motor pin numbers 
        self.motor_1_a = 22
        self.motor_1_b = 16

        self.motor_2_a = 23
        self.motor_2_b = 24

        # Define motor pin numbers
        self.ENA_1 = 17
        self.ENA_2 = 6

        # indicator pin
        self.LED_pin = 19

        self.p1 = None
        self.P2 = None
        # --- physical connections ---

        # rotation calculation thread
        self.Rotation_thread = None

        # defining nessory objects
        self.encoder = Encoder()
        self.rotation = Rotation()        
        
        self.rotation_thread() 

        self.DISTANCE_PER_REVOLUTION = 10.35 # [cm]
        
        # setting up gpio pins 
        self.gpio_setup()

    def calibration(self, num_of_points = 1000):
        self.rotation.calibration(num_of_points)

    def rotation_thread(self):
        self.rotation.reset()
        self.Rotation_thread = threading.Thread(target=self.rotation.calculate_rotation)
        self.Rotation_thread.daemon = True
        self.Rotation_thread.start()
        return True

    def gpio_setup(self):
        # Set up the encoder input pins with pull-up resistors
        # motor 1
        GPIO.setup(self.motor_1_a, GPIO.OUT)
        GPIO.setup(self.motor_1_b, GPIO.OUT)
        
        # motor 2
        GPIO.setup(self.motor_2_a, GPIO.OUT)
        GPIO.setup(self.motor_2_b, GPIO.OUT)

        # ENA pins
        GPIO.setup(self.ENA_1, GPIO.OUT)
        GPIO.setup(self.ENA_2, GPIO.OUT)

        # indicator
        GPIO.setup(self.LED_pin, GPIO.OUT)

        # set frequency 
        self.p1 = GPIO.PWM(self.ENA_1, 1000)
        self.p2 = GPIO.PWM(self.ENA_2, 1000)

        self.p1.start(100)
        self.p2.start(100)

        self.stop_motor(self.motor_1_a, self.motor_1_b)
        self.stop_motor(self.motor_2_a, self.motor_2_b)    
    
    # convert distance in to pulses for avoid the calculation iteration
    def calculate_pulses(self, distance): 
        motor_1_pulses = distance * 321.8 / self.DISTANCE_PER_REVOLUTION
        motor_2_pulses = distance * 321.8 / self.DISTANCE_PER_REVOLUTION
        return  motor_1_pulses, motor_2_pulses
    
    def move_motor_forward(self, motor_pin1, motor_pin2):
        GPIO.output(motor_pin1, GPIO.HIGH)
        GPIO.output(motor_pin2, GPIO.LOW)

    def move_left(self):
        self.move_motor_forward(self.motor_1_a, self.motor_1_b)
        self.move_motor_forward(self.motor_2_b, self.motor_2_a) # revers the inputs
        time.sleep(0.025)
        
        self.move_motor_forward(self.motor_1_b, self.motor_1_a)
        self.move_motor_forward(self.motor_2_a, self.motor_2_b)
        self.stop_motor(self.motor_1_a, self.motor_1_b)
        self.stop_motor(self.motor_2_a, self.motor_2_b)

    def move_right(self):
        self.move_motor_forward(self.motor_1_b, self.motor_1_a)
        self.move_motor_forward(self.motor_2_a, self.motor_2_b) # revers the inputs
        time.sleep(0.025)
        
        self.move_motor_forward(self.motor_1_a, self.motor_1_b)
        self.move_motor_forward(self.motor_2_b, self.motor_2_a)
        self.stop_motor(self.motor_1_a, self.motor_1_b)
        self.stop_motor(self.motor_2_a, self.motor_2_b)
        
    def stop_motor(self, motor_pin1, motor_pin2):
        GPIO.output(motor_pin1, GPIO.LOW)
        GPIO.output(motor_pin2, GPIO.LOW)
    
    def power_controller(self, target_value, current_value, rpm_differ=0, min_power=25, max_power=100):
        # print('power controll function triggered...')
    
        def adjust_speed(speed_1, speed_2, rpm_differ):
            if rpm_differ > 1:
                speed_2 *= 3
            elif rpm_differ < -1:
                speed_1 *= 3            
            return speed_1, speed_2

        def set_speed(speed_1, speed_2):
            self.p1.start(speed_1)
            self.p2.start(speed_2)

        # Cached initial and target values for faster execution
        idv1 = target_value[0]  # initial difference of values
        idv2 = target_value[1]

        # Avoid division by zero and optimize for constant values
        idv1 = idv1 if idv1 != 0 else 1
        idv2 = idv2 if idv2 != 0 else 1

        # Current differences
        cdv1 = target_value[0] - current_value[0]
        cdv2 = target_value[1] - current_value[1]

        # Simplified power calculation
        speed_1 = round(min_power + ((max_power -min_power) * cdv1 / idv1),2)
        speed_2 = round(min_power + ((max_power -min_power) * cdv2 / idv2),2)

        # Adjust and constrain speeds based on RPM difference
        speed_1, speed_2 = adjust_speed(speed_1, speed_2, rpm_differ)
        speed_1 = max(min(speed_1, max_power), min_power)
        speed_2 = max(min(speed_2, max_power), min_power)

        # print(f'The motor 1 speed is :{speed_1}')
        # print(f'The motor 2 speed is :{speed_2}')          
        # print(f'The rpm differnt is : {rpm_differ}')
        # Set the motor speeds with optimized method calls
        set_speed(speed_1, speed_2)

    def set_power(self, power = 100):
        self.p1.start(power)
        self.p1.start(power)
        self.rotation.reset()
        self.encoder.reset_encoders()

    def led_on(self):        
        GPIO.output(self.LED_pin, GPIO.HIGH )    
    def led_off(self):        
        GPIO.output(self.LED_pin, GPIO.LOW )    

    def motor_move(self):
        print('motor move method triggered...')
        self.align()
        time.sleep(0.2)
        self.move_into(self.detection.data[1])
        
        id  = self.detection.data[3]         
        self.rotate(id, self.detection.data[2]) # call rotation function
        time.sleep(0.2)
        if id in [2,4,6,8]:
            self.move_into(35)
        else:
            self.move_into(20)
        time.sleep(0.2)
        print('exiting the move_into function marker\'s iteration done...')
        
        return True
    
    def align(self):
        print('alignment method triggerd...')
        self.set_power()
        if self.detection.data[1] > 17:
            dt = 1
        elif self.detection.data[1] > 15:
            dt = 2
        elif self.detection.data[1] > 12:
            dt = 3
        else:
            dt = 3
            
        while True:
            time.sleep(0.3)
                
            if self.detection.data[0] > dt:
                print(self.detection.data[0])
                self.move_left()
            elif self.detection.data[0] < -dt:
                print(self.detection.data[0])
                self.move_right()
            else:
                print(self.detection.data[0])
                self.stop_motor(self.motor_1_a, self.motor_1_b)
                self.stop_motor(self.motor_2_a, self.motor_2_b)
                break # ignore the alignment
        print('alignment done...')
    
    def move_into(self, distance, reverse = False):
        print(distance)
        print('move_into method triggered...')
        self.set_power()
        # Calculate pulses only once
        m1p, m2p = self.calculate_pulses(distance)
        
        # Reset encoders before starting movement
        if self.encoder.reset_encoders():
            state1 = False
            state2 = False

            target_value = [m1p, m2p]

            # Minimize variable recalculation in loop
            while not (state1 and state2):
                # Read encoder counts (current positions)
                m1e, m2e = self.encoder.read_encoder_count()
                current_value = [m1e, m2e]

                # Calculate RPM difference outside motor logic
                rpm_differ = self.encoder.calculate_rpm_differ()
                # Use power controller to adjust motor speed
                self.power_controller(target_value, current_value,rpm_differ)
                # print(f'state1 is : {state1}, {m1p}, {m1e}')
                # print(f'state2 is : {state2}, {m2p}, {m2e}')

                # Motor 1 control
                if not state1:
                    if m1p > m1e:
                        if not reverse:
                            self.move_motor_forward(self.motor_1_a, self.motor_1_b)
                        else:    
                            self.move_motor_forward(self.motor_1_b, self.motor_1_a)
                    else:
                        self.stop_motor(self.motor_1_a, self.motor_1_b)
                        state1 = True  # Mark motor 1 as stopped

                # Motor 2 control
                if not state2:
                    if m2p > m2e:
                        if not reverse:
                            self.move_motor_forward(self.motor_2_a, self.motor_2_b)
                        else:    
                            self.move_motor_forward(self.motor_2_b, self.motor_2_a)
                    else:
                        self.stop_motor(self.motor_2_a, self.motor_2_b)
                        state2 = True  # Mark motor 2 as stopped
                # Small delay to reduce CPU load without affecting operational speed
                time.sleep(0.01)
        # print(f'motor moves {self.encoder.read_encoder_count()}')

    def rotate_motor_search(self,target_angle):
        print(f'Target angle is: {target_angle}, starting rotation...')
        
        target_value = [abs(target_angle), abs(target_angle)]
        
        while True:
            current_angle = self.rotation._angleYaw  # Read current yaw angle
            current_value = [abs(current_angle), abs(current_angle)]
            
            # Call the power controller to adjust motor speed
            rpm_differ = self.encoder.calculate_rpm_differ()  # Replace with actual rpm diff if available
            self.power_controller(target_value, current_value, rpm_differ)
            
            # Rotation logic
            if current_angle < target_angle - 4:
                self.move_right()  # Motor control for rotating right
            elif current_angle > target_angle + 4:
                self.move_left()   # Motor control for rotating left
            else:
                self.stop_motor(self.motor_1_a, self.motor_1_b)
                self.stop_motor(self.motor_2_a, self.motor_2_b)  # Stop motors when within range

                break  # Exit loop once the angle is within the target range

            # Adding a small delay to avoid busy looping and reduce CPU load
            time.sleep(0.01)
            
            print(f'Target angle is: {target_angle}, starting rotation...')
        
        target_value = [abs(target_angle), abs(target_angle)]
        
        while True:
            current_angle = self.rotation._angleYaw  # Read current yaw angle
            current_value = [abs(current_angle), abs(current_angle)]
            
            # Call the power controller to adjust motor speed
            rpm_differ = self.encoder.calculate_rpm_differ()  # Replace with actual rpm diff if available
            self.power_controller(target_value, current_value, rpm_differ)
            
            # Rotation logic
            if current_angle < target_angle - 4:
                self.move_right()  # Motor control for rotating right
            elif current_angle > target_angle + 4:
                self.move_left()   # Motor control for rotating left
            else:
                self.stop_motor(self.motor_1_a, self.motor_1_b)
                self.stop_motor(self.motor_2_a, self.motor_2_b)  # Stop motors when within range

                break  # Exit loop once the angle is within the target range

            # Adding a small delay to avoid busy looping and reduce CPU load
            time.sleep(0.01)

    def rotate(self, marker_id, rotation):
        print('rotate function is triggered...')
        self.set_power()
        self.rotation.reset()
        # Start the rotation thread (assumed necessary for sensor updates)

        # Power controller will dynamically adjust motor speeds
        def rotate_motor(target_angle):
            print(f'Target angle is: {target_angle}, starting rotation...')
            
            target_value = [abs(target_angle), abs(target_angle)]
            
            while True:
                current_angle = self.rotation._angleYaw  # Read current yaw angle
                current_value = [abs(current_angle), abs(current_angle)]
                
                # Call the power controller to adjust motor speed
                rpm_differ = self.encoder.calculate_rpm_differ()  # Replace with actual rpm diff if available
                self.power_controller(target_value, current_value, rpm_differ)
                
                # Rotation logic
                if current_angle < target_angle - 4:
                    self.move_right()  # Motor control for rotating right
                elif current_angle > target_angle + 4:
                    self.move_left()   # Motor control for rotating left
                else:
                    self.stop_motor(self.motor_1_a, self.motor_1_b)
                    self.stop_motor(self.motor_2_a, self.motor_2_b)  # Stop motors when within range

                    break  # Exit loop once the angle is within the target range

                # Adding a small delay to avoid busy looping and reduce CPU load
                time.sleep(0.01)

        # Function to calculate the target angle based on marker_id and current rotation
        def calculate_angle(marker_id, rotation):
            marker_angles = {
                1: 90,         # East
                2: 135,        # South-East
                3: 180,        # Southpasinduanuradhaperera/fgs
                4: -135,       # South-West
                5: -90,        # West
                6: -45,        # North-West
                7: 0,          # North
                8: 45          # North-East
            }
            
            # Retrieve marker angle based on the given ID
            marker_angle = marker_angles.get(marker_id, None)
            if marker_angle is None:
                return 0
            
            # Adjust the marker angle based on the rotation offset
            marker_angle -= rotation

            # Normalize the angle to within -180 to 180 degrees
            marker_angle = -((marker_angle + 180) % 360 - 180)
            
            return marker_angle

        # Proceed with rotation only if a valid marker_id is provided
        if marker_id != 0:
            # Calculate the target angle for the given marker and rotation
            target_angle = calculate_angle(marker_id, rotation)
            print(f'Calculated target angle: {target_angle}')
            
            # Reset rotation state before starting
            self.rotation.reset()

            # Start the motor rotation process
            rotate_motor(target_angle)
            
            # self.Rotation_thread.join()
            print(f'Final angle is: {self.rotation._angleYaw}, target: {target_angle}')
        return target_angle

    def stop_game(self):
        self._stop = True
        self.rotation.stop_game()
        self.detection.stop_game()
        GPIO.cleanup()
        return True

    def search_1(self):
        self.move_into(5,reverse=True) # stop and return to game class for search
        print('search 1')
       
    def search_2(self):
        self.rotate_motor_search(28)
        self.move_into(8) # stop and return to game class for search
        print('search 2')

    def search_3(self):
        self.move_into(8, reverse=True)
        self.rotate_motor_search(-28)
        self.move_into(8)
        print('search 3')
        pass

    def search_4(self):
        self.move_into(5)
        print('search 5')
        pass

    def search_5(self):
        self.move_into(13, reverse=True)
        self.rotate_motor_search(-28)
        self.move_into(8)
        print('search 5')
        pass

    def search_6(self):
        self.move_into(8, reverse=True)
        self.rotate_motor_search(28)
        self.move_into(5)
        print('search 6')
        pass
    


if __name__ == '__main__':
    det = Detection()
    motor = Motor(det)

    try: 
        # motor.move_motor_forward(motor.motor_2_a,motor.motor_2_b)
        # motor.move_motor_forward(motor.motor_1_a,motor.motor_1_b)
        '''for i in range(100):
            print(i)'''
        
        # motor.move_left()
        # motor.move_left()
        # motor.move_left()
        # motor.move_left()
        # motor.move_left()
        # motor.move_left()
        # motor.move_left()
        # motor.move_left()
        # motor.move_left()
        # motor.move_left()
        '''for i in range(100):
            print(i)
            motor.move_right()'''
        time.sleep(1)
        motor.move_into(5)
            
        while True:
            # print(motor.encoder.calculate_rpm_differ())
            pass
    except KeyboardInterrupt:
        print('cleaning up GPIOs...')
        GPIO.cleanup()
#

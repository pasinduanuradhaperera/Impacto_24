import threading
import RPi.GPIO as GPIO
import time

# import custom packages 
from detection import Detection
from motor import Motor

class Game:
    def __init__(self):
        # Stop logic
        self._stop = False

        self.game = None

        # Define necessary objects 
        self.detect = Detection()
        self.motor = Motor(self.detect)

        # Marker identification thread
        self.marker_thread = None
        
        # Setup GPIO pins

        # physical buttons
        GPIO.setmode(GPIO.BCM)
        self.switch_pin = 9 # game switch
        self.switch_state = False # program does not started yet
        
        GPIO.setup(self.switch_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(self.switch_pin, GPIO.RISING, callback=self.switch_call)

    def switch_call(self,chanel):
        print('hi')           
        self.switch_state = True
        self.motor.led_off()
        self.motor.calibration(200)
        self.motor.led_on()

    def detect_thread(self):
        self.marker_thread = threading.Thread(target=self.detect.detect)
        self.marker_thread.daemon = True
        self.marker_thread.start()

    def game_thread(self):
        self.game = threading.Thread(target=self.start)
        self.game.daemon = True
        self.game.start()

    def start(self):
        # Calibrate the robot's MPU6050
        for i in range(3):            
            self.motor.led_on()
            time.sleep(0.2)
            self.motor.led_off()
            time.sleep(0.2)
        self.motor.calibration(200)
        self.detect_thread()

        print('Game started...')
        self.motor.led_on()
        state = 0
        try:
            while not self._stop:
                time.sleep(0.8)
                print(self.detect.data)

                if self.detect.data[3] == 0:  # Properly stopping the game
                    state = 0
                    self.motor.align()
                    self.motor.move_into(self.detect.data[1])
                    self.stop()

                elif 0 < self.detect.data[3] < 9:  # Running game in this block
                    state = 0
                    self.motor.motor_move()
                    self.motor.set_power()
                    self.detect.reset()
                elif self.detect.data[3] == -9:
                    if state == 0:
                        self.motor.search_1()
                        state = 1
                    elif state == 1:
                        self.motor.search_2()
                        state = 2
                    elif state == 2:
                        self.motor.search_3()
                        state = 3
                    elif state == 3:
                        self.motor.search_4()
                        state = 4
                    elif state == 4:
                        self.motor.search_5()
                        state = 5
                    elif state == 5:
                        self.motor.search_6()
                        state = 0
                    else:
                        print('nothing found ... error...')
                        state = 6

        except KeyboardInterrupt:
            print('Interrupt detected. Stopping the game...')
            self.stop()
        finally:
            GPIO.cleanup()
            print('Game Done and GPIO cleaned up.')

    def stop(self):
        self._stop = True
        self.detect.stop_game()


if __name__ == '__main__':
    game = Game()
    try:
        game.game_thread()
        while True:
            pass
    except KeyboardInterrupt:
        print('Cleaning up GPIOs due to KeyboardInterrupt...')
        GPIO.cleanup()
    finally:
        print('Final cleanup of GPIOs...')
        GPIO.cleanup()

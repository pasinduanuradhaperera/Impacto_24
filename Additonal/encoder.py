import RPi.GPIO as GPIO
import time

# motor 1, encoder 1 -> near buckconverter
# motor 2, encoder 2 -> near Other side

class Encoder:
    def __init__(self):
        # Define encoder pin numbers 
        self.encoder_1_a = 26
        self.encoder_1_b = 5
        self.encoder_2_a = 12
        self.encoder_2_b = 13

        self._gpio_setup()

        # Initialize state for each encoder after setting up GPIO
        self.encoder_1_a_state = GPIO.input(self.encoder_1_a)
        self.encoder_1_b_state = GPIO.input(self.encoder_1_b)
        self.encoder_2_a_state = GPIO.input(self.encoder_2_a)
        self.encoder_2_b_state = GPIO.input(self.encoder_2_b)

        # Direction flags
        self.encoder_1_direction = None
        self.encoder_2_direction = None

        self._creating_event_2()
        self._creating_event_1()

    def _gpio_setup(self):
        # Set up GPIO numbering mode 
        GPIO.setmode(GPIO.BCM)
        
        # Set up the encoder input pins with pull-up resistors
        # encoder 1
        GPIO.setup(self.encoder_1_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.encoder_1_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # encoder 2
        GPIO.setup(self.encoder_2_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.encoder_2_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
    def _creating_event_1(self):
        # Attach interrupt to encoder pins
        GPIO.add_event_detect(self.encoder_1_a, GPIO.BOTH, callback=self._encoder_1_callback)
        GPIO.add_event_detect(self.encoder_1_b, GPIO.BOTH, callback=self._encoder_1_callback)

    def _creating_event_2(self):
        # Attach interrupt to encoder pins
        GPIO.add_event_detect(self.encoder_2_a, GPIO.BOTH, callback=self._encoder_2_callback)
        GPIO.add_event_detect(self.encoder_2_b, GPIO.BOTH, callback=self._encoder_2_callback)

    def _encoder_1_callback(self, channel):
        # Read the current states of the A and B channels
        encoder_1_a_new_state = GPIO.input(self.encoder_1_a)
        encoder_1_b_new_state = GPIO.input(self.encoder_1_b)

        # Determine direction based on phase difference between A and B
        if encoder_1_a_new_state != self.encoder_1_a_state:  # A has changed
            if encoder_1_a_new_state == encoder_1_b_new_state:
                self.encoder_1_direction = "Forward"
            else:
                self.encoder_1_direction = "Backward"
        elif encoder_1_b_new_state != self.encoder_1_b_state:  # B has changed
            if encoder_1_a_new_state == encoder_1_b_new_state:
                self.encoder_1_direction = "Backward"
            else:
                self.encoder_1_direction = "Forward"

        # Update the previous states
        self.encoder_1_a_state = encoder_1_a_new_state
        self.encoder_1_b_state = encoder_1_b_new_state

        print(f'Encoder 1 Direction: {self.encoder_1_direction}')

    def _encoder_2_callback(self, channel):
        # Read the current states of the A and B channels
        encoder_2_a_new_state = GPIO.input(self.encoder_2_a)
        encoder_2_b_new_state = GPIO.input(self.encoder_2_b)

        # Determine direction based on phase difference between A and B
        if encoder_2_a_new_state != self.encoder_2_a_state:  # A has changed
            if encoder_2_a_new_state == encoder_2_b_new_state:
                self.encoder_2_direction = "Forward"
            else:
                self.encoder_2_direction = "Backward"
        elif encoder_2_b_new_state != self.encoder_2_b_state:  # B has changed
            if encoder_2_a_new_state == encoder_2_b_new_state:
                self.encoder_2_direction = "Backward"
            else:
                self.encoder_2_direction = "Forward"

        # Update the previous states
        self.encoder_2_a_state = encoder_2_a_new_state
        self.encoder_2_b_state = encoder_2_b_new_state

        print(f'Encoder 2 Direction: {self.encoder_2_direction}')

if __name__ == "__main__":
    encoder = Encoder()
    try:
        while True:
            time.sleep(0.1)  # Add a small delay to avoid busy waiting
    except KeyboardInterrupt:
        GPIO.cleanup()

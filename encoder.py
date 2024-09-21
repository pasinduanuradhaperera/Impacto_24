import time
import RPi.GPIO as GPIO

class Encoder:
    def __init__(self):
        # Define encoder pin numbers 
        self._encoder_1_a = 5
        self._encoder_1_b = 26
        self._encoder_2_a = 12
        self._encoder_2_b = 13

        # Initialize state for each encoder
        self._state_1 = 0
        self._state_2 = 0

        # Setup the encoder count
        self._encoder_1_count = 0
        self._encoder_2_count = 0

        # RPM tracking
        self._last_time_1 = time.time()
        self._last_time_2 = time.time()
        self._rpm_1 = 0
        self._rpm_2 = 0

        self._gpio_setup()
        self._creating_event_1()
        self._creating_event_2()

    def _gpio_setup(self):
        GPIO.setmode(GPIO.BCM)

        # Set up encoder input pins with pull-up resistors
        # Encoder 1
        GPIO.setup(self._encoder_1_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self._encoder_1_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Encoder 2
        GPIO.setup(self._encoder_2_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self._encoder_2_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def _creating_event_1(self):
        GPIO.add_event_detect(self._encoder_1_a, GPIO.RISING, callback=self._encoder_1_callback)
        GPIO.add_event_detect(self._encoder_1_b, GPIO.RISING, callback=self._encoder_1_callback)

    def _creating_event_2(self):
        GPIO.add_event_detect(self._encoder_2_a, GPIO.RISING, callback=self._encoder_2_callback)
        GPIO.add_event_detect(self._encoder_2_b, GPIO.RISING, callback=self._encoder_2_callback)

    def _encoder_1_callback(self, channel):
        current_time = time.time()
        time_diff = current_time - self._last_time_1

        if time_diff > 0:
            # Calculate RPM based on the time difference between pulses
            self._rpm_1 = round((1 / time_diff))
        self._last_time_1 = current_time 

        # Update encoder count
        self._encoder_1_count += 1

    def _encoder_2_callback(self, channel):
        current_time = time.time()
        time_diff = current_time - self._last_time_2

        if time_diff > 0:
            # Calculate RPM based on the time difference between pulses
            self._rpm_2 = round((1 / time_diff))
        self._last_time_2 = current_time

        # Update encoder count
        self._encoder_2_count += 1

    def reset_encoders(self):
        self._state_1 = 0
        self._state_2 = 0
        self._encoder_1_count = 0
        self._encoder_2_count = 0
        self._rpm_1 = 0
        self._rpm_2 = 0
        print('Encoder values reset.')
        return True

    def read_encoder_count(self):
        return self._encoder_1_count / 2, self._encoder_2_count / 2

    def calculate_rpm_differ(self):
        # Return the RPM difference between motor 1 and motor 2
        rpm_1 = self._rpm_1
        rpm_2 = self._rpm_2
        self._rpm_1 = 0
        self._rpm_2 = 0
        return rpm_1 - rpm_2

if __name__ == "__main__":
    encoder = Encoder()

    try:
        while True:
            # Continuously calculate RPM difference
            rpm_difference = encoder.calculate_rpm_differ()
            print(f"RPM Difference: {rpm_difference}")
            print(f'Encoder 1 count: {encoder.read_encoder_count()}')
            time.sleep(0.5)

    except KeyboardInterrupt:
        GPIO.cleanup()

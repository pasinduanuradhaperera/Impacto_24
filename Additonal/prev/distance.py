import smbus

class Distance:

    def __init__(self) -> None:                
        self.DEVICE_AS5600 = 0x36
        self.bus = smbus.SMBus(1)
        self.init_angle = self.ReadRawAngle()
        self.total_rotation = 0
        # self.wheel_radius = 3.483
        self.prev_pos = 0
        self.distance = 0

    def ReadRawAngle(self):
        read_bytes = self.bus.read_i2c_block_data(self.DEVICE_AS5600, 0x0C, 2)
        return (read_bytes[0] << 8) | read_bytes[1]

    # def ReadMagnitude(self):
    #     read_bytes = self.bus.read_i2c_block_data(self.DEVICE_AS5600, 0x1B, 2)
    #     return (read_bytes[0] << 8) | read_bytes[1]

    def calculate_distance(self):
        while True:
            cur_pos = self.ReadRawAngle()
            if cur_pos >= self.init_angle:
                cur_pos = cur_pos-self.init_angle
            else:
                cur_pos = 4095 + cur_pos - self.init_angle 
            # Calculate the change in position since the last reading
            pos_change = cur_pos - self.prev_pos
            
            # Adjust for wrapping around after a full rotation
            if pos_change < -1500:
                pos_change += 4096
            elif pos_change > 1500:
                pos_change -= 4096
            
            # Update total rotation based on the change in position
            self.total_rotation += pos_change / 4096.0
            
            print("Total Rotation:", self.total_rotation)
            
            # Store current position for the next iteration
            self.prev_pos = cur_pos
            self.distance = 21.50 * self.total_rotation
            print("Total Distance is: ",self.distance)

if __name__ == '__main__':
    dis = Distance()
    dis.calculate_distance()

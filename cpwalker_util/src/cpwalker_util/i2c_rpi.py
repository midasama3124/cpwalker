#!/usr/bin/python
import smbus 
import time
'''
Communication with the MD22 motor driver via I2C with the Raspberry Pi

A note about RPi I2C bus 1 (pins 3 and 5):
    First, these pins have pull-ups to 3.3V. I've tryed to use 10K pull-ups 
    to 5V externally which resulted in poor performance. 
    Second, when testing communication with two MD22s, the traction send_command
    returns an error about <0.01% of the times. This may be a problem or not, I'm
    yet to confirm. The fact is that the Pi has an I2C bug related to clock
    stretching. I believe this may be the cause of these errors. What I did to
    make it better was to change the /boot/config.txt adding:
        dtparam=i2c_baudrate=10000
    to slow down the baud rate in hope clock stretching would become rarer.
'''
class I2Cbus():
    def __init__(self,device):
        self.device = device
        
        # Raspberry-related constant
        self.I2C_PORT = 1    # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)
        # Open the bus
        self.bus = smbus.SMBus(self.I2C_PORT) 
        
        '''
        Working with the MD22
        Set MD22 mode to 0x00: 
            register 0x01, motor 1: speed, range (0,255); 
            register 0x02, motor 2: speed, range (0,255);
        '''
        if self.device == "traction":
            # Traction
            # Device address: 
            #   0x58 will be left shifted to add the read write bit - becomes 0xB0
            #   MD22 switches: On On On On 
            self.TRACTION_DEVICE_ADDRESS = 0x58
            self.TRACTION_DEVICE_REG_MODE = 0x00  # mode register address
            self.TRACTION_LEFT_WHEEL_REG = 0x01   # Define where each wheel/motor is connected# TODO: Check!
            self.TRACTION_RIGHT_WHEEL_REG = 0x02  # Define where each wheel/motor is connected# TODO: Check!
            
            self.traction_init()
        
        if self.device == "elevation":
            # Elevation
            # Device address: 
            #   0x5E will be left shifted to add the read write bit - becomes 0xBC
            #   MD22 switches: On Off Off On
            self.ELEVATION_DEVICE_ADDRESS = 0x5E   
            self.ELEVATION_DEVICE_REG_MODE = 0x00  # mode register address
            self.ELEVATION_MOTOR_REG = 0x01   # Define where each wheel/motor is connected# TODO: Check!
        
            self.elevation_init()


        '''
        Add more I2C modules as needed and write their methods
        
        '''

    def __del__(self):
        if self.device == "traction":
            # Set traction velocities to zero when destructed
            self.bus.write_byte_data(self.TRACTION_DEVICE_ADDRESS, self.TRACTION_LEFT_WHEEL_REG, 127)
            self.bus.write_byte_data(self.TRACTION_DEVICE_ADDRESS, self.TRACTION_RIGHT_WHEEL_REG, 127)
        if self.device == "elevation":
            # Set elevation velocity to zero when destructed
            self.bus.write_byte_data(self.ELEVATION_DEVICE_ADDRESS, self.ELEVATION_MOTOR_REG, 127)
        # Close the bus
        self.bus.close()

    def traction_init(self):
        # Set device mode
        self.bus.write_i2c_block_data(self.TRACTION_DEVICE_ADDRESS, self.TRACTION_DEVICE_REG_MODE, [0])
        # Initialize velocities to zero
        self.bus.write_byte_data(self.TRACTION_DEVICE_ADDRESS, self.TRACTION_LEFT_WHEEL_REG, 127)
        self.bus.write_byte_data(self.TRACTION_DEVICE_ADDRESS, self.TRACTION_RIGHT_WHEEL_REG, 127)
        print("[DEBUG] Traction I2C Initialized")

    def elevation_init(self):
        # Set device mode
        self.bus.write_i2c_block_data(self.ELEVATION_DEVICE_ADDRESS, self.ELEVATION_DEVICE_REG_MODE, [0])
        # Initialize motor velocity to zero
        self.bus.write_byte_data(self.ELEVATION_DEVICE_ADDRESS, self.ELEVATION_MOTOR_REG, 127)
        print("[DEBUG] Elevation I2C Initialized")

    def volts_to_int(self,value_volts):
        '''
        Given a value in volts that we want to output,
        we must map from our driver (MD22) voltage range (-24, +24)
        to our i2c command range (0, 255)
        '''
        return int(value_volts*5.3 + 127.5)

    def traction_send_command(self,left_wheel_signal_volts,right_wheel_signal_volts):
        # Map control signal into i2c outputs
        left_wheel_signal_i2c = self.volts_to_int(left_wheel_signal_volts)
        right_wheel_signal_i2c = self.volts_to_int(right_wheel_signal_volts)
        # Send commands to driver via i2c
        try:
            self.bus.write_byte_data(self.TRACTION_DEVICE_ADDRESS, self.TRACTION_LEFT_WHEEL_REG, int(left_wheel_signal_i2c))
            self.bus.write_byte_data(self.TRACTION_DEVICE_ADDRESS, self.TRACTION_RIGHT_WHEEL_REG, int(right_wheel_signal_i2c))            
        except:
            print("[DEBUG] {} Traction IO error".format(int(round(time.time()))))

    def traction_stop_robot(self):
        self.bus.write_byte_data(self.TRACTION_DEVICE_ADDRESS, self.TRACTION_LEFT_WHEEL_REG, 127)
        self.bus.write_byte_data(self.TRACTION_DEVICE_ADDRESS, self.TRACTION_RIGHT_WHEEL_REG, 127)
    
    def elevation_send_command(self,motor_signal_volts):
        # Map control signal into i2c outputs
        motor_signal_i2c = self.volts_to_int(motor_signal_volts)
        # Send commands to driver via i2c
        try:
            self.bus.write_byte_data(self.ELEVATION_DEVICE_ADDRESS, self.ELEVATION_MOTOR_REG, motor_signal_i2c)
        except:
            print("[DEBUG] Traction IO error")
        
    

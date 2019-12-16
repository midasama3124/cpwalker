#!/usr/bin/python
import smbus 

'''
Communication with the MD22 motor driver via I2C with the Raspberry Pi

'''

class I2Cbus():
    '''
    Working with the MD22
    Set MD22 mode to 0x00: 
        register 0x01, motor 1: speed, range (0,255); 
        register 0x02, motor 2: speed, range (0,255);
    '''
    self.DEVICE_ADDRESS = 0x58   # device address, will be left shifted to add the read write bit - becomes 0xB0
    self.DEVICE_REG_MODE = 0x00  # mode register address

    # Define where each wheel/motor is connected
    self.LEFT_WHEEL_REG = 0x01   # TODO: Check!
    self.RIGHT_WHEEL_REG = 0x02  # TODO: Check!

    # Raspberry-related constant
    self.I2C_PORT = 1    # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)

    def __init__(self):
        # Open the bus
        self.bus = smbus.SMBus(self.I2C_PORT) 

        # Set device mode
        self.bus.write_i2c_block_data(self.DEVICE_ADDRESS, self.DEVICE_REG_MODE, [0])
        
        # Initialize velocities to zero
        print 'Initializing wheel velocities to zero'
        self.bus.write_byte_data(self.DEVICE_ADDRESS, self.LEFT_WHEEL_REG, 127)
        self.bus.write_byte_data(self.DEVICE_ADDRESS, self.RIGHT_WHEEL_REG, 127)

    def __del__(self):
        # Set velocities to zero when destructed
        self.bus.write_byte_data(self.DEVICE_ADDRESS, self.LEFT_WHEEL_REG, 127)
        self.bus.write_byte_data(self.DEVICE_ADDRESS, self.RIGHT_WHEEL_REG, 127)
        
        # Close the bus
        self.bus.close()

    def volts_to_int(self,value_volts):
        '''
        Given a value in votls that we want to output,
        we must map from our driver (MD22) voltage range (-24, +24)
        to our i2c command range (0, 255)
        '''
        return int(value_volts*5.3 + 127.5)

    def send_command(self,left_wheel_signal_volts,right_wheel_signal_volts):
        # Map control signal into i2c outputs
        left_wheel_signal_i2c = self.volts_to_int(left_wheel_signal_volts)
        right_wheel_signal_i2c = self.volts_to_int(right_wheel_signal_volts)
            
        # Send commands to driver via i2c
        self.bus.write_byte_data(self.DEVICE_ADDRESS, self.LEFT_WHEEL_REG, left_wheel_signal_i2c)
        self.bus.write_byte_data(self.DEVICE_ADDRESS, self.RIGHT_WHEEL_REG, right_wheel_signal_i2c)            
        return '[DONE]'

    def stop_robot(self):
        self.bus.write_byte_data(self.DEVICE_ADDRESS, self.LEFT_WHEEL_REG, 127)
        self.bus.write_byte_data(self.DEVICE_ADDRESS, self.RIGHT_WHEEL_REG, 127)
        return '[DONE]'


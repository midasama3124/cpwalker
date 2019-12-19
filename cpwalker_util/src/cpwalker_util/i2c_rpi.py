#!/usr/bin/python
import smbus 

'''
Communication with the MD22 motor driver via I2C with the Raspberry Pi

'''
class I2Cbus():
    def __init__(self):

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
        # Traction
        self.TRACTION_DEVICE_ADDRESS = 0x58   # device address, will be left shifted to add the read write bit - becomes 0xB0 # TODO: Check!
        self.TRACTION_DEVICE_REG_MODE = 0x00  # mode register address
        self.TRACTION_LEFT_WHEEL_REG = 0x01   # Define where each wheel/motor is connected# TODO: Check!
        self.TRACTION_RIGHT_WHEEL_REG = 0x02  # Define where each wheel/motor is connected# TODO: Check!
        
        # Elevation
        self.ELEVATION_DEVICE_ADDRESS = 0x58   # device address, will be left shifted to add the read write bit - becomes 0xB0 # TODO: Check!
        self.ELEVATION_DEVICE_REG_MODE = 0x00  # mode register address
        self.ELEVATION_MOTOR_REG = 0x01   # Define where each wheel/motor is connected# TODO: Check!
        
        '''
        Add more I2C modules as needed and write their methods
        
        '''
        
        '''
        Initialize configurations
        '''
        self.traction_init()
        self.elevation_init()

    def __del__(self):
        # Set traction velocities to zero when destructed
        self.bus.write_byte_data(self.TRACTION_DEVICE_ADDRESS, self.TRACTION_LEFT_WHEEL_REG, 127)
        self.bus.write_byte_data(self.TRACTION_DEVICE_ADDRESS, self.TRACTION_RIGHT_WHEEL_REG, 127)
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
    
    def elevation_init(self):
        # Set device mode
        self.bus.write_i2c_block_data(self.ELEVATION_DEVICE_ADDRESS, self.ELEVATION_DEVICE_REG_MODE, [0])
        # Initialize motor velocity to zero
        self.bus.write_byte_data(self.ELEVATION_DEVICE_ADDRESS, self.ELEVATION_MOTOR_REG, 127)
    

    def volts_to_int(self,value_volts):
        '''
        Given a value in votls that we want to output,
        we must map from our driver (MD22) voltage range (-24, +24)
        to our i2c command range (0, 255)
        '''
        return int(value_volts*5.3 + 127.5)

    def traction_send_command(self,left_wheel_signal_volts,right_wheel_signal_volts):
        # Map control signal into i2c outputs
        left_wheel_signal_i2c = self.volts_to_int(left_wheel_signal_volts)
        right_wheel_signal_i2c = self.volts_to_int(right_wheel_signal_volts)
        # Send commands to driver via i2c
        self.bus.write_byte_data(self.TRACTION_DEVICE_ADDRESS, self.TRACTION_LEFT_WHEEL_REG, left_wheel_signal_i2c)
        self.bus.write_byte_data(self.TRACTION_DEVICE_ADDRESS, self.TRACTION_RIGHT_WHEEL_REG, right_wheel_signal_i2c)            

    def traction_stop_robot(self):
        self.bus.write_byte_data(self.TRACTION_DEVICE_ADDRESS, self.TRACTION_LEFT_WHEEL_REG, 127)
        self.bus.write_byte_data(self.TRACTION_DEVICE_ADDRESS, self.TRACTION_RIGHT_WHEEL_REG, 127)
    
    def elevation_send_command(self,motor_signal_volts):
        # Map control signal into i2c outputs
        motor_signal_i2c = self.volts_to_int(motor_signal_volts)
        # Send commands to driver via i2c
        self.bus.write_byte_data(self.ELEVATION_DEVICE_ADDRESS, self.ELEVATION_MOTOR_REG, motor_signal_i2c)

    

#!/usr/bin/python
import rospy
from simple_pid import PID
###from cpwalker_util.i2c_rpi import I2Cbus
from cpwalker_traction.velocity_util import VelocityUtils

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

'''
This node is resposible for the control of the motors from traction and elevation systems.
The main function instantiates the objects defined by the following two classes:
 - TractionMotorControl
 - ElevationMotorControl
Both systems are driven by MD22 drivers (one for each system) and we're using the I2C lib from our util package.
The rationale behind this node is that we want to avoid competition on the I2C bus by centralizing the write/read 
I2C operations into a single node in a synchronous manner. To add communication functionality with other I2C devices,
one should extend our I2C lib, write a specific class here and cope with the current workflow.
'''

class TractionMotorControl(object):
    '''
    Low level control for a differential drive robot with two actuated wheels (adapted from the UFES CloudWalker to the CPWalker)
    - Subscribes to the current velocity (within an odometry message) and a reference velocity (generated by a higher level controller)
    - Maps those velocities into separate wheel velocities
    - Each motor is controlled using its own PID parameters to have similar responses
    - The motors consider voltage inputs and wheel velocities in m/s
    - The object dealing with the I2C communication handles: 
            - receives as inputs the desired voltage outputs given by the PIDs
            - mapps those into (0,255) int values and output driver commands

    We're using the simple-pid library, which assumes you are tunning the PID parameters in the parallel form:
    C = kp*e + ki*dt*e + kd*de/dt
    '''
    def __init__(self, control_rate):    
        self.control_rate = control_rate
        self.rospy = rospy
        # Start the i2c bus
        ###    self.i2c_bus = I2Cbus("traction")

        # Parameter initialization
        self.initParameters()

        self.initSubscribers()
        if self.debug: self.initPublishers()
        
        # Controllers initialization
        self.initPIDs()
        self.vel_utils = VelocityUtils()

    def __del__(self):
        ###    self.i2c_bus.traction_stop_robot()
        print("For testing only - erase this line")

    def initParameters(self):
        self.debug = self.rospy.get_param("traction_control/debug", False)

        self.control_rate = self.rospy.get_param("traction_control/control_rate", 200)
        self.rate = self.rospy.Rate(self.control_rate)
        
        '''
        I'm following the given convention for ROS topic names:
        cmd_vel: the actual velocity command (geometry_msgs.Twist)
        aux_cmd_vel: an aux velocity command that may not be suitable for the actual output (geometry_msgs.Twist)
        odom: actual odometry readings which - ideally - consider multiple sensors (encoders and IMU, at least) (nav_msgs.Odometry)
        odom_raw: odometry estimated from encoders' readings (nav_msgs.Odometry)
        vel_raw: robot velocity as given by the encoders (ideally, should never be used; use odom instead) (geometry_msgs.Twist) 
        '''
        self.cmd_vel_topic = self.rospy.get_param("traction_general/cmd_vel_topic", "/cmd_vel")
        self.odom_topic = self.rospy.get_param("traction_general/odom_topic", "/odom_raw")

        # PID parameters for each wheel
        time_sample = 1.0/self.control_rate        
        self.controller_params_left = {
            "kp": self.rospy.get_param("traction_control/pid_params_left/P",10),
            "ki": self.rospy.get_param("traction_control/pid_params_left/I",0),
            "kd": self.rospy.get_param("traction_control/pid_params_left/D",0),
            "Ts": self.rospy.get_param("traction_control/pid_params_left/Ts",time_sample)}
        self.controller_params_right = {
            "kp": self.rospy.get_param("traction_control/pid_params_right/P",10),
            "ki": self.rospy.get_param("traction_control/pid_params_right/I",0),
            "kd": self.rospy.get_param("traction_control/pid_params_right/D",0),
            "Ts": self.rospy.get_param("traction_control/pid_params_right/Ts",time_sample)}
        
        self.reference_linear_vel = 0.0
        self.reference_angular_vel = 0.0
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        return

    def initSubscribers(self):
        # Subscribe to a current vel topic and a comand vel topic
        self.sub_odom = self.rospy.Subscriber(self.odom_topic,Odometry,self.callback_odom)    
        self.sub_cmd_vel = self.rospy.Subscriber(self.cmd_vel_topic,Twist,self.callback_high_level_control)
        return

    def initPublishers(self):
        self.left_control_signal = self.rospy.Publisher("left_control_signal", Float64, queue_size = 10)  
        self.right_control_signal = self.rospy.Publisher("right_control_signal", Float64, queue_size = 10)    
        return

    def initPIDs(self):
        # PID initiallization
        # Sample time equal to the control loop period
        # Output limits to keep our control signal away from the driver's +-24V boundaries (keeping it safe)
        self.pid_left  = PID(self.controller_params_left["kp"], self.controller_params_left["ki"], self.controller_params_left["kd"], setpoint=0,sample_time=self.controller_params_left["Ts"],output_limits=(-22, 22))
        self.pid_right = PID(self.controller_params_right["kp"], self.controller_params_right["ki"], self.controller_params_right["kd"], setpoint=0,sample_time=self.controller_params_right["Ts"],output_limits=(-22, 22))
        return

    def callback_high_level_control(self,msg):
        # From high level controller
        # Extract walker's reference linear and angular velocities
        self.reference_linear_vel = msg.linear.x
        self.reference_angular_vel = msg.angular.z
        return

    def callback_odom(self,msg):
        # From odometry
        # Extract walker's current linear and angular velocities
        self.current_linear_vel = msg.twist.twist.linear.x
        self.current_angular_vel = msg.twist.twist.angular.z
        return

    def pid_controller_left(self, reference_vel_wheel, current_vel_wheel):
        self.pid_left.setpoint = reference_vel_wheel
        control_signal = self.pid_left(current_vel_wheel)
        return control_signal

    def pid_controller_right(self, reference_vel_wheel, current_vel_wheel):
        self.pid_right.setpoint = reference_vel_wheel
        control_signal = self.pid_right(current_vel_wheel)
        return control_signal

    # Main loop
    def update_controller(self):
        # Get velocities for each wheel
        reference_left_wheel_vel, reference_right_wheel_vel = self.vel_utils.walker_to_wheels(self.reference_linear_vel, self.reference_angular_vel)
        current_left_wheel_vel, current_right_wheel_vel = self.vel_utils.walker_to_wheels(self.current_linear_vel, self.current_angular_vel)

        # Call PID (returned control signal in volts)
        self.left_wheel_signal_volts = self.pid_controller_left(reference_left_wheel_vel, current_left_wheel_vel)
        self.right_wheel_signal_volts = self.pid_controller_right(reference_right_wheel_vel, current_right_wheel_vel)
        
        if self.debug == True:
            self.left_control_signal.publish(Float64(data=self.left_wheel_signal_volts))
            self.right_control_signal.publish(Float64(data=self.right_wheel_signal_volts))

    def send_command(self):
    ###    self.i2c_bus.traction_send_command(self.left_wheel_signal_volts,self.right_wheel_signal_volts)
        return True

class ElevationMotorControl(object):
    '''
    Low level control for the elevation system
    We're using the simple-pid library, which assumes you are tunning the PID parameters in the parallel form:
    C = kp*e + ki*dt*e + kd*de/dt
    '''
    def __init__(self, control_rate):
        self.control_rate = control_rate
        self.rospy = rospy
        # Start the i2c bus
        ###        self.i2c_bus = I2Cbus("elevation")

        # Parameter initialization
        self.initParameters()

        self.initSubscriber()
        if self.debug: self.initPublisher()
        
        # Controllers initialization
        self.initPID()

    def __del__(self):
        #### self.i2c_bus.stop_robot()
        print("For testing only - erase this line")
        
    def initParameters(self):
        self.debug = self.rospy.get_param("elevation_control/debug", False)

        self.position_topic = self.rospy.get_param("elevation_control/position_topic", "/elevation_signal")
        self.elevation_setpoint_topic = self.rospy.get_param("elevation_control/elevation_setpoint_topic", "/elevation_setpoint")
        
        # PID parameters
        time_sample = 1.0/self.control_rate        
        self.elevation_controller_params = {
            "kp": self.rospy.get_param("elevation_control/pid_params/P",1),
            "ki": self.rospy.get_param("elevation_control/pid_params/I",0),
            "kd": self.rospy.get_param("elevation_control/pid_params/D",0),
            "Ts": self.rospy.get_param("elevation_control/pid_params/Ts",time_sample)}
         
        self.elevation_setpoint = 0.0
        self.elevation_position = 0.0
        return

    def initSubscriber(self):
        # Subscribe to a current vel topic and a comand vel topic
        self.sub_elevation_position = self.rospy.Subscriber(self.position_topic,Float64,self.callback_elevation_position)    
        self.sub_elevation_setpoint = self.rospy.Subscriber(self.elevation_setpoint_topic,Float64,self.callback_elevation_setpoint)
        return

    def initPublisher(self):
        self.elevation_control_signal = self.rospy.Publisher("elevation_control_signal",Float64,queue_size = 10)  
        return

    def initPID(self):
        # PID initiallization
        # Sample time equal to the control loop period
        # Output limits to keep our control signal away from the driver's +-24V boundaries (keeping it safe)
        self.elevation_pid  = PID(self.elevation_controller_params["kp"], self.elevation_controller_params["ki"], self.elevation_controller_params["kd"], setpoint=0,sample_time=self.elevation_controller_params["Ts"],output_limits=(-22, 22))
        return

    def callback_elevation_position(self,msg):
        # From sensors
        self.elevation_position = msg.data
        return

    def callback_elevation_setpoint(self,msg):
        # From high level controller
        self.elevation_setpoint = msg.data
        return

    def pid_controller(self, reference_vel_wheel, current_vel_wheel):
        self.elevation_pid.setpoint = reference_vel_wheel
        control_signal = self.elevation_pid(current_vel_wheel)
        return control_signal

    # Main loop
    def update_controller(self):
        # Call PID (returned control signal in volts)
        self.elevation_signal_volts = self.pid_controller(self.elevation_setpoint,self.elevation_position)
        
        if self.debug == True:
            self.elevation_control_signal.publish(Float64(data=self.elevation_signal_volts))
        
    def send_command(self):
    ###        self.i2c_bus.traction_send_command()
        return True

def main():
    rospy.init_node('motor_control_node', anonymous=True)
    rospy.loginfo("Starting Motor Control Node")

    # Check which motors are we going to work with
    is_traction = rospy.get_param("traction_general/is_active", False)
    is_elevation = rospy.get_param("elevation_general/is_active", False)

    rate_param = rospy.get_param("traction_control/control_rate", 100)
    rate = rospy.Rate(rate_param)

    if is_traction: traction_control = TractionMotorControl(rate_param)
    if is_elevation: elevation_control = ElevationMotorControl(rate_param)

    rospy.loginfo("[Motor Control] Entering control loop...")
    while not rospy.is_shutdown():
        if is_traction:
            traction_control.update_controller()
            traction_control.send_command()
        if is_elevation:
            elevation_control.update_controller()
            elevation_control.send_command()
    	rate.sleep()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

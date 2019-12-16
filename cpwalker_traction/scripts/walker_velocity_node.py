#!/usr/bin/python
import rospy
import message_filters

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

'''
This node compiles the velocity readings from each wheel (in rpm) into 
velocities of the walker frame (or base frame):
- linear vel: m/s
- angular vel: rad/s
'''
class WalkerVelocity(object):
    def __init__(self):
        # ROS initialization
        self.rospy = rospy
        self.rospy.init_node("cpwalker_vel_node", anonymous = True)
        self.rospy.loginfo("Starting CPWalker Vel Node")
        
        # Parameter initialization
        self.initParameters()
        
        self.initPublishers()
        self.initSubscribers()

        # Main loop
        self.main_controller()

    
    def initParameters(self):
        self.debug = self.rospy.get_param("~debug", False)
        
        '''
        I'm following the given convention for ROS topic names:
        vel_raw: robot velocity as given by the encoders (geometry_msgs.Twist)
        odom: actual odometry readings which - ideally - consider multiple sensors (encoders and IMU, at least) (nav_msgs.Odometry)
        '''
        self.right_wheel_vel_topic = self.rospy.get_param("traction_general/right_wheel_vel", "/right_wheel_vel")
        self.left_wheel_vel_topic = self.rospy.get_param("traction_general/left_wheel_vel", "/left_wheel_vel")
        
        self.current_vel_topic = self.rospy.get_param("~vel_topic", "/vel_raw")

        self.wheels_distance = self.rospy.get_param("traction_general/wheels_distance", 0.8) # TODO: Check!
        # We're not assuming both wheels are equal (likelly are)
        self.left_wheel_radius = self.rospy.get_param("traction_general/left_wheel_radius", 0.15) # TODO: Check! # In meters
        self.right_wheel_radius = self.rospy.get_param("traction_general/right_wheel_radius", 0.15) # TODO: Check! # In meters
        rpm_to_ms_const_left  = self.left_wheel_radius * 2 * 3.1415 / 60 
        rpm_to_ms_const_right  = self.right_wheel_radius * 2 * 3.1415 / 60 
        return

    def initPublishers(self):
        self.walker_vel = self.rospy.Publisher(self.current_vel_topic, Twist, queue_size = 100)    
        self.walker_vel_msg = Twist()
        return
    
    def rpm_to_meterspersecond_left(self,vel_in_rpm)
        vel_in_ms = rpm_to_ms_const_left * vel_in_rpm
        return vel_in_ms
    
    def rpm_to_meterspersecond_right(self,vel_in_rpm)
        vel_in_ms = rpm_to_ms_const_right * vel_in_rpm
        return vel_in_ms
    
    def wheels_to_walker(self,left_wheel_vel,right_wheel_vel)
        walker_linear_vel = (left_wheel_vel + right_wheel_vel) / 2
        walker_angular_vel = (right_wheel_vel - left_wheel_vel) / self.wheels_distance 
        return walker_linear_vel, walker_angular_vel

    # We're using message_filters to sync messages from different topics
    # The callback process pairs of messages that arrived at approximately the same time     
    def vel_callback(left_wheel_vel_sub, right_wheel_vel_sub):
        # Messages are sync'ed, OK
        # We have the velocity of each wheel in rpm
        # Lets put those in m/s
        left_wheel_vel_in_ms = rpm_to_meterspersecond_left(left_wheel_vel_sub)
        right_wheel_vel_in_ms = rpm_to_meterspersecond_right(right_wheel_vel_sub)

        # Get the velocities of the walker frame
        walker_linear_vel, walker_angular_vel = wheels_to_walker(left_wheel_vel_in_ms,right_wheel_vel_in_ms)

        # Create and publish the velocity message
        self.walker_vel_msg.linear.x = walker_linear_vel
        self.walker_vel_msg.angular.z = walker_angular_vel
        self.walker_vel.publish(self.walker_vel_msg)

    # Main loop
    def main_controller(self):
        left_wheel_vel_sub = message_filters.Subscriber(self.left_wheel_vel_topic, Float64)
        right_wheel_vel_sub = message_filters.Subscriber(self.right_wheel_vel_topic, Float64)

        self.ts = message_filters.ApproximateTimeSynchronizer([left_wheel_vel_sub, right_wheel_vel_sub], 10, 0. 1, allow_headerless=True)
        self.ts.registerCallback(vel_callback)
        
        while not self.rospy.is_shutdown():
            rospy.spin()

if __name__ == '__main__':
    try:
        wv = WalkerVelocity()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/python
import rospy
import message_filters

from cpwalker_traction.odometry import WalkerOdometry
from cpwalker_traction.velocity_util import VelocityUtils

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

'''
This node compiles the velocity readings from each wheel (in rpm) into 
velocities of the walker frame (or base frame) to publish the odometry
'''
class WalkerOdomNode(object):
    def __init__(self):
        # ROS initialization
        self.rospy = rospy
        self.rospy.init_node("cpwalker_odom_node", anonymous = False)
        self.rospy.loginfo("Starting CPWalker Odom Node")

        # Initialize odometry
        self.odom = WalkerOdometry()
        # Initialize vel util
        self.vel_util = VelocityUtils()

        # Parameter initialization
        self.initParameters()

        # Main loop
        self.main_controller()

    def initParameters(self):
        self.right_wheel_vel_topic = self.rospy.get_param("traction_general/right_wheel_vel", "/right_wheel_vel")
        self.left_wheel_vel_topic = self.rospy.get_param("traction_general/left_wheel_vel", "/left_wheel_vel")

    # We're using message_filters to sync messages from different topics
    # The callback process pairs of messages that arrived at approximately the same time     
    def vel_callback(self,left_wheel_vel_sub, right_wheel_vel_sub):
        # Messages are sync'ed, OK
        # We want the walker velocity, not each wheels' separately
        walker_linear_vel,walker_angular_vel = self.vel_util.get_walker_vel(left_wheel_vel_sub,right_wheel_vel_sub)
        # Given the current velocities, update and publish odometry
        self.odom.update_velocity(walker_linear_vel,walker_angular_vel)
        self.odom.update_odometry()
        self.odom.publish_odometry()

    # Main loop
    def main_controller(self):
        left_wheel_vel_sub = message_filters.Subscriber(self.left_wheel_vel_topic, Float64)
        right_wheel_vel_sub = message_filters.Subscriber(self.right_wheel_vel_topic, Float64)

        self.ts = message_filters.ApproximateTimeSynchronizer([left_wheel_vel_sub, right_wheel_vel_sub],10, 0.1,allow_headerless=True)
        self.ts.registerCallback(self.vel_callback)
        
        while not self.rospy.is_shutdown():
            rospy.spin()

if __name__ == '__main__':
    try:
        wv = WalkerOdomNode()
    except rospy.ROSInterruptException:
        pass

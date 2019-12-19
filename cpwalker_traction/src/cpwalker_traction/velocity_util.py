#!/usr/bin/python
import math
from math import sin, cos, pi

import rospy
import tf2_ros
import tf_conversions
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped

class VelocityUtils(object):
    def __init__(self):
        self.rospy = rospy
        self.init_params()
        if self.publish_vel_raw: self.init_publishers()

    def init_params(self):
        self.publish_vel_raw = self.rospy.get_param("traction_general/pub_vel_raw", False)
        self.current_vel_topic = self.rospy.get_param("traction_general/vel_topic", "/vel_raw")
        self.wheels_separation = self.rospy.get_param("traction_general/wheels_separation", 0.8) # TODO: Check!
        # We're not assuming both wheels are equal, though they should be (and likelly are)
        self.left_wheel_radius = self.rospy.get_param("traction_general/left_wheel_radius", 0.15) # TODO: Check! # In meters
        self.right_wheel_radius = self.rospy.get_param("traction_general/right_wheel_radius", 0.15) # TODO: Check! # In meters
        self.rpm_to_ms_const_left  = self.left_wheel_radius * 2 * 3.1415 / 60 
        self.rpm_to_ms_const_right  = self.right_wheel_radius * 2 * 3.1415 / 60 
        return
    # In case we want to publish the raw velocity
    def init_publishers(self):
        self.walker_vel = self.rospy.Publisher(self.current_vel_topic, Twist, queue_size = 100)    
        self.walker_vel_msg = Twist()
        return
    
    def rpm_to_meterspersecond_left(self,vel_in_rpm):
        vel_in_ms = self.rpm_to_ms_const_left * vel_in_rpm
        return vel_in_ms
    
    def rpm_to_meterspersecond_right(self,vel_in_rpm):
        vel_in_ms = self.rpm_to_ms_const_right * vel_in_rpm
        return vel_in_ms
    
    def wheels_to_walker(self,left_wheel_vel,right_wheel_vel):
        # From left and right wheels velocities (m/s)
        # To walker's linear (m/s) and angular (rad/s) velocities
        walker_linear_vel = (left_wheel_vel + right_wheel_vel) / 2
        walker_angular_vel = (right_wheel_vel - left_wheel_vel) / self.wheels_separation 
        return walker_linear_vel, walker_angular_vel

    def walker_to_wheels(self,lin,ang):
        # From walker's linear (m/s) and angular (rad/s) velocities
        # To left and right wheels velocities (m/s)
        left_wheel_vel = lin - ((ang * self.wheels_separation) / 2)
        right_wheel_vel = lin + ((ang * self.wheels_separation) / 2)
        return left_wheel_vel, right_wheel_vel

    def get_walker_vel(self,left_wheel_vel_sub,right_wheel_vel_sub):
        # We have the velocity of each wheel in rpm
        # Lets put those in m/s
        left_wheel_vel_in_ms = self.rpm_to_meterspersecond_left(left_wheel_vel_sub.data)
        right_wheel_vel_in_ms = self.rpm_to_meterspersecond_right(right_wheel_vel_sub.data)

        # Get the velocities of the walker frame
        walker_linear_vel, walker_angular_vel = self.wheels_to_walker(left_wheel_vel_in_ms,right_wheel_vel_in_ms)
        return walker_linear_vel, walker_angular_vel
    
    # In case we want to publish the raw velocity
    def publish_velocity(self,walker_linear_vel,walker_angular_vel):
        self.walker_vel_msg.linear.x = walker_linear_vel
        self.walker_vel_msg.angular.z = walker_angular_vel
        self.walker_vel.publish(self.walker_vel_msg)
        
#!/usr/bin/python
import rospy
import numpy as np 

from geometry_msgs.msg import Twist

'''
This node generates a signal that can be used to tune the PID controlers.
To tune each motor individually, mirror the topic using a topic_tools relay node and
use the walker_odom_node to consolidate the speed
'''

def main():
    rospy.init_node('signal_generator', anonymous=True)
    rospy.loginfo("Starting Signal Generator Node")

    # Check which motors are we going to work with
    is_traction = rospy.get_param("traction_general/is_active", True)
    # Check which signal we want: square or sine
    output_signal = rospy.get_param("traction_aux/signal", "square")
    # Get cmd_vel topic
    cmd_vel_topic = rospy.get_param("traction_general/cmd_vel_topic", "/cmd_vel")
    cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=5)
    cmd_vel = Twist()

    rate_param = rospy.get_param("traction_control/control_rate", 100)    
    rate = rospy.Rate(rate_param)

    total_time = 10
    frequency = 0.1
    amplitude = 0.6
    time_points = total_time * rate_param
    t = np.linspace(0,total_time,time_points+1)
    sine = np.sin(2*np.pi*t*frequency)
    if output_signal == "square":
        output = amplitude * np.sign(sine)
    elif output_signal == "sine":
        output = amplitude * sine
    
    i = 0
    
    while not rospy.is_shutdown():
        if i < time_points:
            cmd_vel.linear.x = output[i]
            cmd_vel.angular.z = 0.0
            cmd_vel_pub.publish(cmd_vel)
            i = i + 1
        else:
            i = 0
        rate.sleep()
    
    cmd_vel.linear.x = 0.0
    cmd_vel.angular.z = 0.0
    cmd_vel_pub.publish(cmd_vel)
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

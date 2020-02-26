#!/usr/bin/python
import rospy
import message_filters

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


'''
This node node is responsible for activating/deactivating the control of the elevation and traction motors.
We wait for the parameters responsible for starting/stopping each subsystem. 
We're working with the following assumptions:
    - The motor control node should wait for the overall system to be up and running;
    - The elevation control will take place first;
    - The traction control will only start after the elevation setpoint is reached.
'''

class SetpointRelay(object):
    def __init__(self):    
        self.rospy = rospy
        rospy.init_node('setpoint_relay', anonymous=False)
        self.rospy.loginfo("[Setpoint Relay] Started")

        self.initParameters()
        self.initSubscribers()
        self.initPublishers()
        self.main()

    def __del__(self):
        self.pub_cmd_vel.publish(self.stop_traction)
        self.pub_elevation_setpoint.publish(self.stop_elevation)
        pass

    def initParameters(self):
        self.verbose = rospy.get_param("traction_hw/verbose", True)

        self.update_rate = self.rospy.get_param("relay/update_rate", 1)
        self.rate = self.rospy.Rate(self.update_rate)

        # Flags to activate subsystems
        self.traction_control_subsystem = False
        self.elevation_control_subsystem = False
        # Let's keep the current state saved to log changes
        self.previous_traction_flag = False
        self.previous_elevation_flag = False
        self.updated = False
        
        # Empty messages
        self.stop_traction = Twist()
        self.stop_elevation = Float64()
        '''
        I'm following the given convention for ROS topic names:
        cmd_vel: the actual velocity command (geometry_msgs.Twist)
        aux_cmd_vel: an aux velocity command that may not be suitable for the actual output (geometry_msgs.Twist)
        elevation_setpoint: the actual command
        aux_elevation_setpoint: the output from the elevation acquisition/processing subsystem
        '''
        self.input_cmd_vel_topic = self.rospy.get_param("traction_general/aux_cmd_vel_topic", "/aux_cmd_vel")
        self.input_elevation_setpoint_topic = self.rospy.get_param("elevation_control/aux_elevation_setpoint_topic", "/aux_elevation_setpoint")
        
        self.output_cmd_vel_topic = self.rospy.get_param("traction_general/cmd_vel_topic", "/cmd_vel")
        self.output_elevation_setpoint_topic = self.rospy.get_param("elevation_control/elevation_setpoint_topic", "/elevation_setpoint")
        return

    def update_flags(self):
        self.traction_control_subsystem = self.rospy.get_param("relay/traction_flag", False)
        self.elevation_control_subsystem = self.rospy.get_param("relay/elevation_flag", False)
        # We want to keep the state and log/print if paramters are changed
        if (self.previous_traction_flag == self.traction_control_subsystem) and (self.previous_elevation_flag == self.elevation_control_subsystem):
            self.updated = False
        else:
            self.updated = True
        self.previous_traction_flag = self.traction_control_subsystem
        self.previous_elevation_flag = self.elevation_control_subsystem

    def initSubscribers(self):
        self.sub_cmd_vel = self.rospy.Subscriber(self.input_cmd_vel_topic, Twist, self.callback_traction)
        self.sub_elevation_setpoint = self.rospy.Subscriber(self.input_elevation_setpoint_topic, Float64, self.callback_elevation)
        return

    def initPublishers(self):
        self.pub_cmd_vel = self.rospy.Publisher(self.output_cmd_vel_topic, Twist, queue_size = 100)  
        self.pub_elevation_setpoint = self.rospy.Publisher(self.output_elevation_setpoint_topic, Float64, queue_size = 100)    
        return

    def callback_traction(self,msg):
        if self.traction_control_subsystem:
            self.pub_cmd_vel.publish(msg)
        return

    def callback_elevation(self,msg):
        if self.elevation_control_subsystem:
            self.pub_elevation_setpoint.publish(msg)
        return

    def main(self):
        while not rospy.is_shutdown():
            self.update_flags()
            if self.updated:
                self.rospy.loginfo("[Setpoint Relay] Parameter update after command")
                self.rospy.loginfo("[Setpoint Relay] Traction:  {}".format(self.traction_control_subsystem))
                self.rospy.loginfo("[Setpoint Relay] Elevation: {}".format(self.elevation_control_subsystem))
        	self.rate.sleep()

if __name__ == '__main__':
    try:
        relay = SetpointRelay()
    except:
        pass
 

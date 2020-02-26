#!/usr/bin/python
import rospy
from std_msgs.msg import Float64
import time

class ElevationSetpoint(object):
    def __init__(self):    
        self.rospy = rospy
        rospy.init_node('elevation_setpoint_input', anonymous=False)
        self.rospy.loginfo("[Elevation Setpoint Input] Started")

        self.init_params()
        self.init_var()
        self.init_pubs_()
        self.main()

    def __del__(self):
        pass

    def init_params(self):
        self.flag_manual_param = self.rospy.get_param("elevation_control/elevation_setpoint_manual", False)
        # Sets the setpoint value for the elevation system control
        self.setpoint_value = self.rospy.get_param("elevation_control/elevation_setpoint_value", 0.0)
        
        self.setpoint_topic = self.rospy.get_param("elevation_control/aux_elevation_setpoint_topic", "/aux_elevation_setpoint")
        
        self.control_rate = self.rospy.get_param("elevation_control/control_rate", 100)
        self.rate = self.rospy.Rate(self.control_rate)

    def init_var(self):
        # Set control flag
        self.flag_manual = self.flag_manual_param
        self.setpoint_msg = Float64(self.setpoint_value)

    def init_pubs_(self):
        self.elevation_setpoint_pub = rospy.Publisher(self.setpoint_topic, Float64, queue_size=100)

    def update_params(self):
        self.new_setpoint_value = self.rospy.get_param("elevation_control/elevation_setpoint_value", 0.0)
        if self.new_setpoint_value == self.setpoint_value:
            pass
        else:
            self.setpoint_value = self.new_setpoint_value
            self.setpoint_msg = Float64(self.setpoint_value)

    def main(self):
        while not rospy.is_shutdown():
            # Are we setting the setpoint value manually?
            if self.flag_manual:
                self.update_params()
                self.elevation_setpoint_pub.publish(self.setpoint_msg)
            else:
                self.elevation_setpoint_pub.publish(self.setpoint_msg)
            self.rate.sleep()

if __name__ == '__main__':
    #try:
    setpoint = ElevationSetpoint()
#    except:
#        pass

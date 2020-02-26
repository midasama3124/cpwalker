#!/usr/bin/python
import rospy
from std_msgs.msg import Float64
import time

class ElevationOffset(object):
    def __init__(self):    
        self.rospy = rospy
        rospy.init_node('elevation_offset', anonymous=False)
        self.rospy.loginfo("[Elevation Offset] Started")

        self.init_params()
        self.init_var()
        self.init_pubs_()
        self.init_subs_()
        self.main()

    def __del__(self):
        pass

    def init_params(self):
        self.output_weight_topic = self.rospy.get_param("elevation_control/elevation_weight_topic", "/elevation_weight")
        
        self.update_rate = self.rospy.get_param("elevation_hw/offset_rate", 1)
        self.rate = self.rospy.Rate(self.update_rate)
        
        self.flag_manual_param = self.rospy.get_param("elevation_hw/offset_manual", False)
        self.manual_offset_value = self.rospy.get_param("elevation_hw/manual_offset_value", 0.0)

    def init_var(self):
        # Set control flags
        self.flag_manual = self.flag_manual_param
        self.flag_publish = False
        self.flag_started = False
        # Initialize control variables
        self.time_counter = 0.0
        self.total_time = 1.0       # Total time in which we'll perform readings to extract an offset
        self.messages_arrived = 0.0
        self.cummulative_sum = 0.0
        self.offset = 0.0

    def init_pubs_(self):
        self.elevation_signal_pub = rospy.Publisher(self.output_weight_topic, Float64, queue_size=100)

    def init_subs_(self):
        self.gauge_weight_sub = self.rospy.Subscriber("elevation_gauge_weight", Float64, self.callback_gauge)

    def callback_gauge(self, msg):
        # Register when the first message arrives
        if self.flag_started == False:
            self.flag_started = True
            self.time_counter = time.time()  
        # If we're not read to publish, we'll sum the incoming messages to extract an average later 
        if self.flag_publish == False:
            self.cummulative_sum = self.cummulative_sum + msg.data
            self.messages_arrived = self.messages_arrived + 1
        # If we can publish, offset the message and publishs
        else:
            weight_offseted = msg.data - self.offset
            weight_msg = Float64(weight_offseted)
            self.elevation_signal_pub.publish(weight_msg)

    def main(self):
        while not rospy.is_shutdown():
            # Are we setting the offset value manually?
            if self.flag_manual and not self.flag_publish:
                # If so, overwrite control flags and set the offset value directly
                self.flag_started = True
                self.offset = self.manual_offset_value
                self.flag_publish = True
                self.rospy.loginfo("[Elevation Offset] Value set manually")
                self.rospy.loginfo("[Elevation Offset] Offset:  {}".format(self.offset))
            else:
                # If not, check if process started
                if self.flag_started and not self.flag_publish:
                    # Check if enough time has passed and get the average reading
                    if (time.time() - self.time_counter) > self.total_time:
                        self.offset = self.cummulative_sum / self.messages_arrived
                        self.flag_publish = True
                        self.rospy.loginfo("[Elevation Offset] Value set automatically")
                        self.rospy.loginfo("[Elevation Offset] Offset:  {}".format(self.offset))
           	self.rate.sleep()

if __name__ == '__main__':
    #try:
    offset = ElevationOffset()
#    except:
#        pass
